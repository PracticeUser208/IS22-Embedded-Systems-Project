/*
 *	subsystem3.c
 *	Subsystem 3 -- Line and Barcode: creates its objects and starts its
 *	task (see subsystem3.h).
 *
 *	Objects (TEAM.md sections 5 and 6):
 *	  line_cmd_q  message buffer  VC -> S3, 4-byte words (IF-17)
 *	  steer_mtx   mutex           guards the latest steering (IF-19)
 *	  line_task   task            samples the IR sensors (task_line.c)
 */

#include "subsystem3.h"

#if !TEMPLATE_DEMO

#define S3_CMD_DEPTH		8	/* 4-byte commands */
#define S3_STEER_LOCK_MS	10	/* bounded wait on steer_mtx */

EXPORT ID	line_cmd_q;		/* IF-17, replaces the VC's weak 0 */
EXPORT ID	steer_mtx;		/* IF-19 */
EXPORT ER	s3_ir_status = E_NOEXS;

LOCAL S3_STEER	s3_steer_now;		/* guarded by steer_mtx */

EXPORT ER subsystem3_init(void)
{
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)(S3_CMD_DEPTH * 8);	/* 4-byte message + header */
	cmbf.maxmsz = (INT)sizeof(UW);
	line_cmd_q = tk_cre_mbf(&cmbf);
	if (line_cmd_q < E_OK) {
		return line_cmd_q;
	}

	/* TA_INHERIT: the VC (higher priority) never waits long behind
	   line_task while it holds the lock (priority inheritance) */
	cmtx.mtxatr = TA_INHERIT;
	steer_mtx = tk_cre_mtx(&cmtx);
	if (steer_mtx < E_OK) {
		return steer_mtx;
	}

	s3_steer_now.state = 4U;		/* SUSPENDED until the VC says go */
	s3_ir_status = ir_init();		/* ADC pins + "adca" driver */
	return E_OK;				/* a sensor problem is reported by
						   line_task as SENSOR_FAULT */
}

EXPORT ER subsystem3_start(void)
{
	ID tid = app_start_task(line_task, S3_PRI_LINE, 0, "line_task");

	return (tid < E_OK) ? tid : E_OK;
}

EXPORT ER s3_steer_get(S3_STEER *p_steer)
{
	ER er;

	if ((p_steer == NULL) || (steer_mtx <= 0)) {
		return E_PAR;
	}
	er = tk_loc_mtx(steer_mtx, S3_STEER_LOCK_MS);
	if (er < E_OK) {
		return er;
	}
	*p_steer = s3_steer_now;
	(void)tk_unl_mtx(steer_mtx);
	return E_OK;
}

EXPORT void s3_steer_put(const S3_STEER *p_steer)
{
	if (tk_loc_mtx(steer_mtx, S3_STEER_LOCK_MS) == E_OK) {
		s3_steer_now = *p_steer;
		(void)tk_unl_mtx(steer_mtx);
	}
}

#else /* TEMPLATE_DEMO: the template's consumer example */

EXPORT ID		mbx_samples;	/* IF-01 */
EXPORT ID		mtx_stats;	/* IF-04 */
EXPORT APP_STATS	app_stats;

EXPORT ER subsystem3_init(void)
{
	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};

	/* Mailbox: passes a POINTER to a message (zero copy).  The message
	   memory must stay valid until the receiver is done with it, which
	   is why subsystem 2 takes it from its memory pool. */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;	/* FIFO waiters, FIFO messages */
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	/* Mutex: only one task at a time may touch app_stats.  TA_INHERIT
	   (priority inheritance) avoids priority inversion: a low-priority
	   holder is temporarily raised to the waiter's priority. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	return (mtx_stats < E_OK) ? mtx_stats : E_OK;
}

EXPORT ER subsystem3_start(void)
{
	/* Pinned to core 0 (TP_PRC1): opposite core to the producer */
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	return (tid < E_OK) ? tid : E_OK;
}

#endif /* TEMPLATE_DEMO */

/* ======== from drv_ir.c ======== */
/*
 *	drv_ir.c
 *	Subsystem 3 -- IR sensors on the ADC (see drv_ir.h).
 *
 *	Uses the kernel's own ADC device driver ("adca", device/adc), which
 *	is built in and started at boot.  The shared device configuration
 *	leaves the ADC pins untouched (DEVCONF_ADC_PIN_INIT_x FALSE), so this
 *	driver makes GP26-GP28 analog inputs itself -- the same two register
 *	writes the kernel driver would do -- instead of editing the shared
 *	device/ folder (README: drivers live in the subsystem's folder).
 */

#include <tk/tkernel.h>

LOCAL ID	ir_dd = E_NOEXS;	/* ADC device descriptor */

LOCAL void ir_pin_analog(UW pin)
{
	/* No digital function, digital input buffer and pulls off */
	out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
	clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
}

EXPORT ER ir_init(void)
{
	ir_pin_analog(LF_PIN_LEFT);
	ir_pin_analog(LF_PIN_CENTRE);
	ir_pin_analog(LF_PIN_BARCODE);

	ir_dd = tk_opn_dev((UB*)"adca", TD_READ);
	return (ir_dd < E_OK) ? (ER)ir_dd : E_OK;
}

EXPORT ER ir_read(UH raw[IR_COUNT])
{
	INT	ch;

	if (ir_dd < E_OK) {
		return E_NOEXS;
	}
	for (ch = 0; ch < IR_COUNT; ch++) {
		UW	v = 0U;
		SZ	asz = 0;
		ER	er = tk_srea_dev(ir_dd, (W)ch, &v, 1, &asz);

		if (er < E_OK) {
			return er;
		}
		raw[ch] = (UH)(v & 0x0FFFU);
	}
	return E_OK;
}

/* ======== from s3_ir.c ======== */
#include <stddef.h>

static bool cal_valid(s3_ir_cal_t const * p_cal)
{
    bool b_valid = false;
    if (NULL != p_cal)
    {
        uint32_t span = (p_cal->black > p_cal->white)
            ? (uint32_t)p_cal->black - p_cal->white
            : (uint32_t)p_cal->white - p_cal->black;
        b_valid = (span > (2u * p_cal->hysteresis));
    }
    return (b_valid);
}

bool s3_ir_calibrate(s3_ir_cal_t * p_cal, uint16_t white, uint16_t black,
                     uint16_t min_span, uint16_t hysteresis)
{
    s3_ir_cal_t candidate = {white, black, hysteresis};
    uint32_t span = (black > white) ? (uint32_t)black - white
                                          : (uint32_t)white - black;
    bool b_ok = (NULL != p_cal) && (min_span > 0u)
        && (span >= min_span) && cal_valid(&candidate);
    if (b_ok)
    {
        *p_cal = candidate;
    }
    return (b_ok);
}

bool s3_ir_classify(s3_ir_cal_t const * p_cal, uint16_t raw,
                    bool b_known, bool * p_black)
{
    if ((NULL == p_black) || !cal_valid(p_cal))
    {
        return (false);
    }

    int32_t direction = (p_cal->black > p_cal->white) ? 1 : -1;
    int32_t midpoint = ((int32_t)p_cal->white + (int32_t)p_cal->black) / 2;
    int32_t distance = ((int32_t)raw - midpoint) * direction;
    int32_t band = (int32_t)p_cal->hysteresis;
    if (!b_known)
    {
        *p_black = (distance >= 0);
    }
    else if (distance > band)
    {
        *p_black = true;
    }
    else if (distance < -band)
    {
        *p_black = false;
    }
    else
    {
        /* Retain the previous classification inside the hysteresis band. */
    }
    return (true);
}

bool s3_ir_normalize(s3_ir_cal_t const * p_cal, uint16_t raw,
                     uint16_t * p_blackness)
{
    if ((NULL == p_blackness) || !cal_valid(p_cal))
    {
        return (false);
    }

    int32_t span = (int32_t)p_cal->black - (int32_t)p_cal->white;
    int32_t value = (((int32_t)raw - (int32_t)p_cal->white) * 1000) / span;
    if (value < 0)
    {
        value = 0;
    }
    else if (value > 1000)
    {
        value = 1000;
    }
    else
    {
        /* In calibrated range. */
    }
    *p_blackness = (uint16_t)value;
    return (true);
}
/* End of file. */

/* ======== from s3_line.c ======== */
#include <stddef.h>

bool s3_line_evaluate(s3_line_config_t const * p_config, bool b_left_dark,
                      bool b_centre_dark, s3_line_result_t * p_result)
{
    if (NULL == p_result)
    {
        return (false);
    }
    p_result->steering = 0;
    p_result->flags = 0u;
    if ((NULL == p_config)
        || ((p_config->present_mask & 0xf0u) != 0u)
        || ((p_config->steering_valid_mask & 0xf0u) != 0u)
        || (((uint32_t)p_config->steering_valid_mask
             & ~(uint32_t)p_config->present_mask) != 0u))
    {
        return (false);
    }

    uint8_t pattern = (uint8_t)((b_left_dark ? 1u : 0u)
                               | (b_centre_dark ? 2u : 0u));
    uint32_t selected = 1u << pattern;
    p_result->flags = pattern;
    if ((p_config->present_mask & selected) != 0u)
    {
        p_result->flags |= 4u;
    }
    if ((p_config->steering_valid_mask & selected) != 0u)
    {
        p_result->flags |= 8u;
        p_result->steering = p_config->steering[pattern];
    }
    return (true);
}

bool s3_line_is_present(s3_line_result_t const * p_result)
{
    return ((NULL != p_result) && ((p_result->flags & 4u) != 0u));
}

bool s3_line_has_steering(s3_line_result_t const * p_result)
{
    return ((NULL != p_result) && ((p_result->flags & 8u) != 0u));
}
/* End of file. */

/* ======== from s3_pulse.c ======== */
#include <stddef.h>

#define ACTIVE 1u
#define INVALID 2u
#define DARK 4u
#define FIRST_DARK 8u

static bool invalidate(s3_pulse_t * p_capture)
{
    p_capture->flags = (uint8_t)(((uint32_t)p_capture->flags | INVALID) & ~ACTIVE);
    return (false);
}

static bool check_time(s3_pulse_t * p_capture, uint32_t now_us)
{
    if ((NULL == p_capture) || ((p_capture->flags & ACTIVE) == 0u))
    {
        return (false);
    }
    if (((now_us - p_capture->last_sample_us) > p_capture->max_gap_us)
        || ((now_us - p_capture->last_edge_us) > p_capture->max_width_us))
    {
        return (invalidate(p_capture));
    }
    p_capture->last_sample_us = now_us;
    return (true);
}

static bool append_run(s3_pulse_t * p_capture, uint32_t now_us)
{
    uint32_t width = now_us - p_capture->last_edge_us;
    if ((width == 0u) || (p_capture->count >= S3_PULSE_CAPACITY))
    {
        return (invalidate(p_capture));
    }
    p_capture->widths_us[p_capture->count] = width;
    p_capture->count++;
    p_capture->last_edge_us = now_us;
    return (true);
}

bool s3_pulse_begin(s3_pulse_t * p_capture, uint32_t now_us, bool b_dark,
                    uint32_t max_gap_us, uint32_t max_width_us)
{
    if (NULL == p_capture)
    {
        return (false);
    }
    p_capture->flags = INVALID;
    p_capture->count = 0u;
    if ((max_gap_us == 0u) || (max_gap_us >= UINT32_C(0x80000000))
        || (max_width_us == 0u) || (max_width_us >= UINT32_C(0x80000000)))
    {
        return (false);
    }
    p_capture->last_edge_us = now_us;
    p_capture->last_sample_us = now_us;
    p_capture->max_gap_us = max_gap_us;
    p_capture->max_width_us = max_width_us;
    p_capture->flags = (uint8_t)(ACTIVE | (b_dark ? DARK | FIRST_DARK : 0u));
    return (true);
}

bool s3_pulse_sample(s3_pulse_t * p_capture, uint32_t now_us, bool b_dark)
{
    if (!check_time(p_capture, now_us))
    {
        return (false);
    }
    if (b_dark != ((p_capture->flags & DARK) != 0u))
    {
        if (!append_run(p_capture, now_us))
        {
            return (false);
        }
        p_capture->flags ^= DARK;
    }
    return (true);
}

bool s3_pulse_finish(s3_pulse_t * p_capture, uint32_t now_us)
{
    if (!check_time(p_capture, now_us) || !append_run(p_capture, now_us))
    {
        return (false);
    }
    p_capture->flags = (uint8_t)((uint32_t)p_capture->flags & ~ACTIVE);
    return (true);
}

bool s3_pulse_matches(s3_pulse_t const * p_capture, bool b_first_dark,
                      uint32_t const * p_min_us, uint32_t const * p_max_us,
                      uint8_t count)
{
    if ((NULL == p_capture) || (NULL == p_min_us) || (NULL == p_max_us)
        || (count == 0u) || (count > S3_PULSE_CAPACITY)
        || (count != p_capture->count)
        || ((p_capture->flags & (ACTIVE | INVALID)) != 0u)
        || (b_first_dark != ((p_capture->flags & FIRST_DARK) != 0u)))
    {
        return (false);
    }
    for (uint8_t index = 0u; index < count; index++)
    {
        if ((p_min_us[index] == 0u) || (p_min_us[index] > p_max_us[index])
            || (p_capture->widths_us[index] < p_min_us[index])
            || (p_capture->widths_us[index] > p_max_us[index]))
        {
            return (false);
        }
    }
    return (true);
}
/* End of file. */

/* ======== from s3_state.c ======== */
#include <stddef.h>

#define STATE_MASK 7u
#define CALIBRATED 8u

static void set_state(s3_state_t * p_state, uint8_t value)
{
    p_state->packed = (uint8_t)((p_state->packed & CALIBRATED) | value);
}

void s3_state_init(s3_state_t * p_state)
{
    if (NULL != p_state)
    {
        p_state->packed = S3_STATE_INIT;
        p_state->search_start_us = 0u;
    }
}

uint8_t s3_state_get(s3_state_t const * p_state)
{
    return ((NULL == p_state) ? S3_STATE_FAULT
                               : (uint8_t)(p_state->packed & STATE_MASK));
}

bool s3_state_can_track(s3_state_t const * p_state)
{
    uint8_t state = s3_state_get(p_state);
    return ((state == S3_STATE_FOLLOW) || (state == S3_STATE_CAPTURE)
            || (state == S3_STATE_DECODE));
}

uint8_t s3_state_step(s3_state_t * p_state, s3_state_input_t input,
                      uint32_t now_us)
{
    uint8_t state = s3_state_get(p_state);
    if (NULL == p_state)
    {
        return (S3_STATE_EVT_NONE);
    }
    if (input == S3_STATE_RESET)
    {
        s3_state_init(p_state);
        return (S3_STATE_EVT_NONE);
    }
    if (state == S3_STATE_FAULT)
    {
        return (S3_STATE_EVT_NONE);
    }
    if ((state > S3_STATE_FAULT) || (input == S3_STATE_SENSOR_FAULT))
    {
        set_state(p_state, S3_STATE_FAULT);
        return (S3_STATE_EVT_FAULT);
    }
    if (input == S3_STATE_SUSPEND)
    {
        set_state(p_state, S3_STATE_SUSPENDED);
        return (S3_STATE_EVT_NONE);
    }
    if ((state == S3_STATE_SEARCH)
        && ((now_us - p_state->search_start_us) >= S3_STATE_SEARCH_US))
    {
        set_state(p_state, S3_STATE_FAULT);
        return (S3_STATE_EVT_TIMEOUT);
    }
    if ((state == S3_STATE_INIT) && (input == S3_STATE_CALIBRATED))
    {
        p_state->packed = CALIBRATED | S3_STATE_FOLLOW;
    }
    else if ((state == S3_STATE_SUSPENDED) && (input == S3_STATE_RESUME_SEARCH))
    {
        if ((p_state->packed & CALIBRATED) == 0u)
        {
            set_state(p_state, S3_STATE_INIT);
        }
        else
        {
            set_state(p_state, S3_STATE_SEARCH);
            p_state->search_start_us = now_us;
        }
    }
    else if (s3_state_can_track(p_state) && (input == S3_STATE_LINE_LOST))
    {
        set_state(p_state, S3_STATE_SEARCH);
        p_state->search_start_us = now_us;
        return (S3_STATE_EVT_LOST);
    }
    else if ((state == S3_STATE_SEARCH) && (input == S3_STATE_LINE_FOUND))
    {
        set_state(p_state, S3_STATE_FOLLOW);
        return (S3_STATE_EVT_REACQUIRED);
    }
    else if ((state == S3_STATE_FOLLOW) && (input == S3_STATE_BARCODE_START))
    {
        set_state(p_state, S3_STATE_CAPTURE);
    }
    else if ((state == S3_STATE_CAPTURE) && (input == S3_STATE_BARCODE_END))
    {
        set_state(p_state, S3_STATE_DECODE);
    }
    else if ((state == S3_STATE_DECODE) && (input == S3_STATE_DECODE_DONE))
    {
        set_state(p_state, S3_STATE_FOLLOW);
    }
    else
    {
        /* Ignore inputs inappropriate to this state; never implicitly reset. */
    }
    return (S3_STATE_EVT_NONE);
}
/* End of file. */

/* ======== from s3_code39.c ======== */
/*
 *	s3_code39.c
 *	Subsystem 3 (Line and Barcode) -- Code 39 decoding (see s3_code39.h).
 */

#include <stddef.h>

#define C39_CHARS		C39_TABLE_SIZE
#define C39_ELEMS		9u	/* per character            */
#define C39_STRIDE		10u	/* + the gap after it       */
#define C39_WIDE		3u	/* wide elements per character */

/* Wide elements need to be clearly wider than narrow ones:
   narrowest wide >= 1.2 x widest narrow (Code 39 uses 2:1 to 3:1).
   8 Nov: 1.3 -> 1.2, tuned on the 154 recorded passes of the 4 Nov
   logs (with the 25 / 75 % second-look tries, bc_rec.c): K read 11
   times instead of 6, never a wrong letter.  Simulated: no wrong
   letter with the speed swinging up to +-50 % within the barcode
   (2 at +-60 %); 1.1 read 13 but gave wrong letters at +-50 %. */
#define C39_SEP_NUM		12u
#define C39_SEP_DEN		10u
#define C39_WIDTH_MAX		0x0FFFFFFFu	/* any longer is not a bar */

/* One 9-bit pattern per character, first element = most significant
   bit, 1 = wide (the standard Code 39 table) */
static const uint16_t c39_pat[C39_CHARS] = {
	0x034u, 0x121u, 0x061u, 0x160u, 0x031u, 0x130u, 0x070u, 0x025u,	/* 0-7 */
	0x124u, 0x064u,							/* 8-9 */
	0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00Du, 0x10Cu,	/* A-H */
	0x04Cu, 0x01Cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u,	/* I-P */
	0x007u, 0x106u, 0x046u, 0x016u, 0x181u, 0x0C1u, 0x1C0u, 0x091u,	/* Q-X */
	0x190u, 0x0D0u,							/* Y-Z */
	0x085u, 0x184u, 0x0C4u, 0x094u, 0x0A8u, 0x0A2u, 0x08Au, 0x02Au	/* - . space * $ / + % */
};
static const char c39_chr[C39_CHARS + 1u] =
	"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%";

/* One character from its 9 widths.  Returns the character (0 if none)
   and the width of its narrowest wide element. */
static char c39_char(const uint32_t *p_g, uint32_t *p_min_wide)
{
	bool		wide[C39_ELEMS] = { false };
	uint32_t	min_wide = UINT32_MAX;
	uint32_t	max_narrow = 0u;
	uint16_t	pat = 0u;
	uint32_t	k;
	uint32_t	i;

	for (k = 0u; k < C39_WIDE; k++) {	/* the 3 widest */
		uint32_t best = C39_ELEMS;

		for (i = 0u; i < C39_ELEMS; i++) {
			if (!wide[i] && ((best == C39_ELEMS) || (p_g[i] > p_g[best]))) {
				best = i;
			}
		}
		wide[best] = true;
	}
	for (i = 0u; i < C39_ELEMS; i++) {
		pat = (uint16_t)(pat << 1);
		if (wide[i]) {
			pat |= 1u;
			if (p_g[i] < min_wide) {
				min_wide = p_g[i];
			}
		} else if (p_g[i] > max_narrow) {
			max_narrow = p_g[i];
		} else {
			/* narrower */
		}
	}
	*p_min_wide = min_wide;
	if ((min_wide * C39_SEP_DEN) < (max_narrow * C39_SEP_NUM)) {
		return '\0';			/* wide and narrow not separable */
	}
	for (i = 0u; i < C39_CHARS; i++) {
		if (c39_pat[i] == pat) {
			return c39_chr[i];
		}
	}
	return '\0';
}

/* One direction */
static uint8_t c39_decode_dir(const uint32_t *p_w, uint8_t count, char *p_text,
			      uint8_t cap)
{
	uint32_t	chars;
	uint32_t	c;
	uint8_t		n = 0u;

	if ((count < C39_MIN_ELEMENTS) || (count > C39_MAX_ELEMENTS) ||
	    ((((uint32_t)count + 1u) % C39_STRIDE) != 0u)) {
		return 0u;
	}
	chars = ((uint32_t)count + 1u) / C39_STRIDE;	/* including the '*'s */
	if ((chars - 2u) >= (uint32_t)cap) {
		return 0u;			/* no room for the text + NUL */
	}
	for (c = 0u; c < chars; c++) {
		uint32_t	min_wide;
		char		ch = c39_char(&p_w[c * C39_STRIDE], &min_wide);
		bool		edge = (c == 0u) || (c == (chars - 1u));

		if ((ch == '\0') || (edge != (ch == '*'))) {
			return 0u;		/* bad character, or '*' misplaced */
		}
		if ((c + 1u) < chars) {
			uint32_t gap = p_w[(c * C39_STRIDE) + C39_ELEMS];

			if ((gap * 2u) > (min_wide * 3u)) {
				return 0u;	/* gap wider than 1.5 wide: not one symbol */
			}
		}
		if (!edge) {
			p_text[n] = ch;
			n++;
		}
	}
	p_text[n] = '\0';
	return n;
}

uint8_t c39_decode(const uint32_t *p_widths, uint8_t count, char *p_text,
		   uint8_t cap)
{
	uint32_t	rev[C39_MAX_ELEMENTS];
	uint8_t		n;
	uint32_t	i;

	if ((p_widths == NULL) || (p_text == NULL) || (cap < 2u)) {
		return 0u;
	}
	p_text[0] = '\0';
	for (i = 0u; i < count; i++) {
		if ((p_widths[i] == 0u) || (p_widths[i] > C39_WIDTH_MAX)) {
			return 0u;		/* keeps the x13 sums inside 32 bits */
		}
	}
	n = c39_decode_dir(p_widths, count, p_text, cap);
	if ((n == 0u) && (count >= C39_MIN_ELEMENTS) && (count <= C39_MAX_ELEMENTS)) {
		for (i = 0u; i < count; i++) {		/* crossed the other way */
			rev[i] = p_widths[(uint32_t)count - 1u - i];
		}
		n = c39_decode_dir(rev, count, p_text, cap);
	}
	if (n == 0u) {
		p_text[0] = '\0';
	}
	return n;
}

bool c39_table(uint8_t idx, char *p_c, uint16_t *p_pat)
{
	if (idx >= C39_CHARS) {
		return false;
	}
	if (p_c != NULL) {
		*p_c = c39_chr[idx];
	}
	if (p_pat != NULL) {
		*p_pat = c39_pat[idx];
	}
	return true;
}

/* ======== from line_follow.c ======== */
/*
 *	line_follow.c
 *	Subsystem 3 (Line and Barcode) -- line following logic (see
 *	line_follow.h and line_cfg.h).  Pure logic, tested by line_check.c.
 */

#include <stddef.h>

/* s3_line pattern index: bit 0 left dark, bit 1 centre dark */
#define PAT_NONE	0u
#define PAT_LEFT	1u
#define PAT_CENTRE	2u
#define PAT_BOTH	3u
#define PAT_BIT(n)	((uint8_t)(1u << (n)))

/* Internal sensor -> ADC channel (line_cfg.h): LF_LEFT is the left one
   of the middle pair (the "centre" sensor), LF_CENTRE the right one,
   LF_BARCODE the side sensor */
static const uint8_t lf_ch_of[LF_SENSORS] = {
	(uint8_t)LF_CH_PAIR_LEFT, (uint8_t)LF_CH_PAIR_RIGHT, (uint8_t)LF_CH_SIDE
};

uint8_t lf_ir_by_pin(uint8_t ir_bits)
{
	uint8_t out = 0u;
	uint32_t i;

	for (i = 0u; i < LF_SENSORS; i++) {
		if ((ir_bits & (1u << i)) != 0u) {
			out |= (uint8_t)(1u << lf_ch_of[i]);
		}
	}
	return out;
}

static int16_t clamp_steer(int32_t v)
{
	if (v > LF_STEER_HARD) {
		v = LF_STEER_HARD;
	} else if (v < -LF_STEER_HARD) {
		v = -LF_STEER_HARD;
	} else {
		/* in range */
	}
	return (int16_t)v;
}

/* Proportional edge term: centre darker than half -> steer left (+).
   Within LF_EDGE_DEADBAND of half dark: 0, straight on (no hunting
   around the edge); beyond it, proportional to the distance past the
   dead band, so the steering starts from 0 without a jump. */
static int32_t edge_term(uint16_t centre_level)
{
	int32_t e = (int32_t)centre_level - (int32_t)LF_EDGE_TARGET;

	if (e > (int32_t)LF_EDGE_DEADBAND) {
		e -= (int32_t)LF_EDGE_DEADBAND;
	} else if (e < -(int32_t)LF_EDGE_DEADBAND) {
		e += (int32_t)LF_EDGE_DEADBAND;
	} else {
		e = 0;
	}
	return (e * LF_KP_PER_1000) / 1000;
}

/* LF_MODE_CENTRE: the line between the two sensors (line_cfg.h).
   Steering from the difference of their darkness; *p_present = either
   sensor on or next to the line. */
static int32_t lf_centre(lf_t *p_lf, uint32_t now_ms, bool *p_present)
{
	int32_t l = (int32_t)p_lf->level[LF_LEFT];
	int32_t c = (int32_t)p_lf->level[LF_CENTRE];
	int32_t e = l - c;

	if ((l < (int32_t)LF_NEAR_LEVEL) && (c < (int32_t)LF_NEAR_LEVEL)) {
		uint32_t limit = LF_STRADDLE_MS;	/* 0 = turn at once */

		if ((limit > 0u) && p_lf->contact && ((now_ms - p_lf->contact_ms) < limit)) {
			*p_present = true;	/* the line between them: straight */
			return 0;
		}
		*p_present = false;		/* gone off to a side */
		if (p_lf->last_side > 0) {
			return (int32_t)LF_STEER_HARD;
		}
		if (p_lf->last_side < 0) {
			return -(int32_t)LF_STEER_HARD;
		}
		return 0;
	}
	*p_present = true;
	p_lf->contact = true;
	p_lf->contact_ms = now_ms;
	if (e > (int32_t)LF_C_SIDE_LEVEL) {
		p_lf->last_side = 1;		/* the line is to the left  */
	} else if (e < -(int32_t)LF_C_SIDE_LEVEL) {
		p_lf->last_side = -1;		/* the line is to the right */
	} else {
		/* in the middle: keep the side seen last */
	}
	if (e > (int32_t)LF_C_DEADBAND) {
		e -= (int32_t)LF_C_DEADBAND;
	} else if (e < -(int32_t)LF_C_DEADBAND) {
		e += (int32_t)LF_C_DEADBAND;
	} else {
		e = 0;
	}
	return (e * LF_C_KP_PER_1000) / 1000;
}

/* Crossing (or a mark beside the line): see line_cfg.h.  TRUE while
   the car should hold its steering. */
static bool lf_crossing(lf_t *p_lf, uint32_t now_ms)
{
	bool left = p_lf->dark[LF_LEFT];
	bool both_clear = !left && (p_lf->level[LF_CENTRE] < LF_DARK_LEVEL);

	if (p_lf->cross) {
		if (!both_clear) {
			p_lf->clear = false;
		} else if (!p_lf->clear) {
			p_lf->clear = true;
			p_lf->clear_ms = now_ms;
		} else {
			/* still clear */
		}
		if ((p_lf->clear && ((now_ms - p_lf->clear_ms) >= LF_CROSS_CLEAR_MS)) ||
		    ((now_ms - p_lf->cross_ms) > LF_CROSS_MAX_MS)) {
			p_lf->cross = false;	/* through it (or not a crossing) */
		}
	} else if (left && !p_lf->left_was_dark && p_lf->edge_seen &&
		   ((now_ms - p_lf->edge_ms) <= LF_CROSS_WINDOW_MS)) {
		p_lf->cross = true;		/* left went dark next to the edge */
		p_lf->cross_ms = now_ms;
		p_lf->clear = false;
	} else {
		/* normal */
	}
	p_lf->left_was_dark = left;
	return p_lf->cross;
}

void lf_init(lf_t *p_lf)
{
	uint32_t i;

	if (p_lf == NULL) {
		return;
	}
	for (i = 0u; i < LF_SENSORS; i++) {
		p_lf->lo[i] = (uint16_t)LF_ADC_MAX;
		p_lf->hi[i] = 0u;
		p_lf->cal[i].white = 0u;
		p_lf->cal[i].black = 0u;
		p_lf->cal[i].hysteresis = 0u;
		p_lf->cls[i] = p_lf->cal[i];
		p_lf->cal_ok[i] = false;
		p_lf->dark[i] = false;
		p_lf->dark_known[i] = false;
		p_lf->level[i] = 0u;
		p_lf->filt[i] = 0u;
	}
	p_lf->have_sample = false;
	p_lf->last_side = 0;
	p_lf->mode = (uint8_t)LF_MODE;
	p_lf->contact = false;
	p_lf->contact_ms = 0u;
	p_lf->seen = false;
	p_lf->last_seen_ms = 0u;
	p_lf->edge_seen = false;
	p_lf->edge_ms = 0u;
	p_lf->left_was_dark = false;
	p_lf->cross = false;
	p_lf->cross_ms = 0u;
	p_lf->clear = false;
	p_lf->clear_ms = 0u;

	/* Buddy 3's pattern table: which patterns mean "line here", and the
	   coarse steering for each.  The centre-only pattern gets the fine
	   proportional term on top (lf_update). */
	p_lf->line_cfg.steering[PAT_NONE]   = 0;
	p_lf->line_cfg.steering[PAT_LEFT]   = (int16_t)LF_STEER_HARD;
	p_lf->line_cfg.steering[PAT_CENTRE] = 0;
	p_lf->line_cfg.steering[PAT_BOTH]   = (int16_t)LF_STEER_HARD;
	p_lf->line_cfg.present_mask = (uint8_t)(PAT_BIT(PAT_LEFT) | PAT_BIT(PAT_CENTRE) |
						PAT_BIT(PAT_BOTH));
	p_lf->line_cfg.steering_valid_mask = p_lf->line_cfg.present_mask;
}

/* Lightest and darkest value seen become white and black once they are
   LF_MIN_SPAN apart (s3_ir checks the span and the hysteresis). */
static void lf_calibrate(lf_t *p_lf, uint32_t i)
{
	uint16_t span;
	uint16_t white;
	uint16_t black;

	if (p_lf->hi[i] <= p_lf->lo[i]) {
		return;
	}
	span = (uint16_t)(p_lf->hi[i] - p_lf->lo[i]);
	if (span < LF_MIN_SPAN) {
		return;
	}
	white = (LF_BLACK_IS_HIGH != 0) ? p_lf->lo[i] : p_lf->hi[i];
	black = (LF_BLACK_IS_HIGH != 0) ? p_lf->hi[i] : p_lf->lo[i];
	if (s3_ir_calibrate(&p_lf->cal[i], white, black, (uint16_t)LF_MIN_SPAN,
			    (uint16_t)(span / LF_HYST_DIV))) {
		/* black/white decision: threshold at LF_BLACK_PCT % of the
		   span (s3_ir decides at the middle of white..black2); the
		   side (barcode) sensor at LF_BC_BLACK_PCT % */
		uint32_t pct  = (i == LF_BARCODE) ? LF_BC_BLACK_PCT : LF_BLACK_PCT;
		uint32_t hdiv = (i == LF_BARCODE) ? LF_BC_HYST_DIV : LF_CLS_HYST_DIV;
		int32_t black2 = (int32_t)white +
			(((int32_t)black - (int32_t)white) * (int32_t)(2u * pct)) / 100;

		if (!s3_ir_calibrate(&p_lf->cls[i], white, (uint16_t)black2, 1u,
				     (uint16_t)(span / hdiv))) {
			p_lf->cls[i] = p_lf->cal[i];
		}
		p_lf->cal_ok[i] = true;
	}
}

/* A sensor that has not seen both colours yet borrows the centre
   sensor's calibration (same module type). */
static const s3_ir_cal_t *lf_cal_for(const lf_t *p_lf, uint32_t i)
{
	if (p_lf->cal_ok[i]) {
		return &p_lf->cal[i];
	}
	if (p_lf->cal_ok[LF_CENTRE]) {
		return &p_lf->cal[LF_CENTRE];
	}
	return NULL;
}

/* The same for the black/white decision (LF_BLACK_PCT) */
static const s3_ir_cal_t *lf_cls_for(const lf_t *p_lf, uint32_t i)
{
	if (p_lf->cal_ok[i]) {
		return &p_lf->cls[i];
	}
	if (p_lf->cal_ok[LF_CENTRE]) {
		return &p_lf->cls[LF_CENTRE];
	}
	return NULL;
}

void lf_update(lf_t *p_lf, const uint16_t raw[LF_SENSORS], uint32_t now_ms,
	       lf_out_t *p_out)
{
	uint32_t		i;
	s3_line_result_t	res;
	int32_t			steer = 0;
	bool			present;
	uint8_t			pat;

	if (p_out == NULL) {
		return;
	}
	p_out->steer_mm_s = 0;
	p_out->search_dir = -1;
	p_out->present = false;
	p_out->calibrated = false;
	p_out->ir_bits = 0u;
	if ((p_lf == NULL) || (raw == NULL)) {
		return;
	}

	/* 1. Smooth (average with the previous sample) and calibrate.  Not
	   the barcode sensor: its narrow bars last only 2-3 samples. */
	for (i = 0u; i < LF_SENSORS; i++) {
		/* which pin each sensor is on (line_cfg.h LF_CH_xxx) */
		uint32_t src = lf_ch_of[i];
		uint16_t r = (raw[src] > LF_ADC_MAX) ? (uint16_t)LF_ADC_MAX : raw[src];

		p_lf->filt[i] = (p_lf->have_sample && (i != LF_BARCODE))
			? (uint16_t)(((uint32_t)p_lf->filt[i] + r + 1u) / 2u) : r;
		if (p_lf->filt[i] < p_lf->lo[i]) {
			p_lf->lo[i] = p_lf->filt[i];
		}
		if (p_lf->filt[i] > p_lf->hi[i]) {
			p_lf->hi[i] = p_lf->filt[i];
		}
		lf_calibrate(p_lf, i);
	}
	p_lf->have_sample = true;

	/* 2. Black or white (Buddy 3's s3_ir, with hysteresis) */
	for (i = 0u; i < LF_SENSORS; i++) {
		const s3_ir_cal_t *p_cal = lf_cal_for(p_lf, i);
		const s3_ir_cal_t *p_cls = lf_cls_for(p_lf, i);

		if ((p_cal != NULL) && (p_cls != NULL) &&
		    s3_ir_classify(p_cls, p_lf->filt[i], p_lf->dark_known[i], &p_lf->dark[i]) &&
		    s3_ir_normalize(p_cal, p_lf->filt[i], &p_lf->level[i])) {
			p_lf->dark_known[i] = true;
		} else {
			p_lf->dark_known[i] = false;
			p_lf->dark[i] = false;
			p_lf->level[i] = 0u;
		}
		if (p_lf->dark[i]) {
			p_out->ir_bits |= (uint8_t)(1u << i);
		}
	}

	p_out->calibrated = p_lf->cal_ok[LF_CENTRE];
	if (!p_out->calibrated) {
		return;				/* no steering until calibrated */
	}

	/* 3. Where is the line? (Buddy 3's s3_line pattern table) */
	(void)s3_line_evaluate(&p_lf->line_cfg, p_lf->dark[LF_LEFT],
			       p_lf->dark[LF_CENTRE], &res);
	pat = (uint8_t)(res.flags & 3u);
	present = s3_line_is_present(&res) ||
		  (p_lf->level[LF_CENTRE] >= LF_NEAR_LEVEL);
	if (s3_line_has_steering(&res)) {
		steer = res.steering;
	}

	/* 4. Steering.  Centre mode: the line between the sensors.  Edge
	   mode: at a crossing (lf_crossing, line_cfg.h) drive straight
	   through, and do not take it as the line's side. */
	if (p_lf->mode == (uint8_t)LF_MODE_CENTRE) {
		steer = lf_centre(p_lf, now_ms, &present);
	} else if (lf_crossing(p_lf, now_ms)) {
		steer = 0;
		present = true;
	} else if ((pat == PAT_LEFT) || (pat == PAT_BOTH)) {
		/* left sensor on the line: hard left (table value) */
		p_lf->last_side = 1;
	} else if (p_lf->level[LF_CENTRE] >= LF_NEAR_LEVEL) {
		/* centre on or next to the line's left edge: proportional
		   trim around half dark.  The line's body is to the right. */
		p_lf->last_side = -1;
		steer += edge_term(p_lf->level[LF_CENTRE]);
	} else if (p_lf->last_side > 0) {
		steer = LF_STEER_HARD;		/* it went off to the left */
	} else {
		/* centre white, the line's body to the right: the car has
		   drifted left off the edge (or the line bends right) */
		steer = edge_term(p_lf->level[LF_CENTRE]);
	}

	/* remember the last time the centre rode the edge */
	if (!p_lf->cross && !p_lf->dark[LF_LEFT] &&
	    (p_lf->level[LF_CENTRE] >= LF_NEAR_LEVEL) &&
	    (p_lf->level[LF_CENTRE] < LF_DARK_LEVEL)) {
		p_lf->edge_seen = true;
		p_lf->edge_ms = now_ms;
	}

	if (present) {
		p_lf->seen = true;
		p_lf->last_seen_ms = now_ms;
	}
	p_out->steer_mm_s = clamp_steer(steer);
	p_out->present = present;
	p_out->search_dir = (p_lf->last_side > 0) ? (int8_t)1 : (int8_t)-1;
}

void lf_assume_on_line(lf_t *p_lf, uint32_t now_ms)
{
	if (p_lf != NULL) {
		p_lf->contact = true;
		p_lf->contact_ms = now_ms;
		p_lf->seen = true;
		p_lf->last_seen_ms = now_ms;
	}
}

bool lf_lost(const lf_t *p_lf, uint32_t now_ms)
{
	if ((p_lf == NULL) || !p_lf->seen) {
		return true;
	}
	return ((now_ms - p_lf->last_seen_ms) > LF_LOST_MS);
}

void lf_jct_reset(lf_jct_t *p_j, uint32_t t_us, bool side_dark)
{
	if (p_j != NULL) {
		p_j->white_us = t_us;
		p_j->dark_us = t_us;
		p_j->was_dark = side_dark;
		p_j->armed = false;
		p_j->on_line = false;
		p_j->candidate = false;
	}
}

bool lf_jct_step(lf_jct_t *p_j, uint32_t t_us, bool side_dark, bool on_line,
		 uint32_t run_us, uint32_t max_us, uint32_t gap_us)
{
	if (p_j == NULL) {
		return false;
	}
	if (side_dark != p_j->was_dark) {
		if (side_dark) {			/* white -> dark */
			p_j->dark_us = t_us;
			p_j->armed = ((t_us - p_j->white_us) >= gap_us);
			p_j->on_line = false;
			p_j->candidate = false;		/* too little white after */
		} else {				/* dark -> white */
			uint32_t len = t_us - p_j->dark_us;

			p_j->white_us = t_us;
			p_j->candidate = p_j->armed && p_j->on_line &&
					 (len >= run_us) && (len <= max_us);
		}
		p_j->was_dark = side_dark;
	}
	if (side_dark) {
		p_j->on_line = p_j->on_line || on_line;
	} else if (p_j->candidate && ((t_us - p_j->white_us) >= gap_us)) {
		p_j->candidate = false;
		return true;
	} else {
		/* white */
	}
	return false;
}

/* ======== from bc_reader.c ======== */
/*
 *	bc_reader.c
 *	Subsystem 3 (Line and Barcode) -- reading a barcode off the side
 *	sensor (see bc_reader.h).
 */

#include <stddef.h>

void bc_reader_init(bc_reader_t *p_rd)
{
	if (p_rd != NULL) {
		p_rd->pulse.count = 0u;
		p_rd->pulse.flags = 0u;
		p_rd->min_us = 0u;
		p_rd->max_us = 0u;
		p_rd->seen = 0u;
		p_rd->active = false;
		p_rd->was_dark = false;
	}
}

/* White for this long after the last bar = the barcode is over */
static uint32_t bc_quiet_us(const bc_reader_t *p_rd)
{
	uint32_t q = p_rd->max_us * 2u;

	if ((p_rd->min_us * 5u) > q) {
		q = p_rd->min_us * 5u;
	}
	if (q < LF_BC_QUIET_MIN_US) {
		q = LF_BC_QUIET_MIN_US;
	} else if (q > LF_BC_QUIET_MAX_US) {
		q = LF_BC_QUIET_MAX_US;
	} else {
		/* in range */
	}
	return q;
}

uint8_t bc_reader_sample(bc_reader_t *p_rd, uint32_t now_us, bool dark,
			 bool allow_start)
{
	uint8_t result = BC_RD_NONE;

	if (p_rd == NULL) {
		return BC_RD_NONE;
	}
	if (!p_rd->active) {
		if (allow_start && dark && !p_rd->was_dark &&
		    s3_pulse_begin(&p_rd->pulse, now_us, true, LF_BC_GAP_US,
				   LF_BC_RUN_MAX_US)) {
			p_rd->active = true;	/* the first bar */
			p_rd->min_us = LF_BC_RUN_MAX_US;
			p_rd->max_us = 0u;
			p_rd->seen = 0u;
			result = BC_RD_START;
		}
	} else if (!s3_pulse_sample(&p_rd->pulse, now_us, dark)) {
		p_rd->active = false;
		result = BC_RD_ABORT;
	} else {
		while (p_rd->seen < p_rd->pulse.count) {	/* finished elements */
			uint32_t w = p_rd->pulse.widths_us[p_rd->seen];

			if (w < p_rd->min_us) {
				p_rd->min_us = w;
			}
			if (w > p_rd->max_us) {
				p_rd->max_us = w;
			}
			p_rd->seen++;
		}
		if (!dark && (p_rd->pulse.count > 0u) &&
		    ((now_us - p_rd->pulse.last_edge_us) > bc_quiet_us(p_rd))) {
			p_rd->active = false;	/* widths end with the last bar */
			result = BC_RD_DONE;
		} else {
			result = BC_RD_BUSY;
		}
	}
	p_rd->was_dark = dark;
	return result;
}

void bc_reader_cancel(bc_reader_t *p_rd)
{
	if (p_rd != NULL) {
		p_rd->active = false;
	}
}

bool bc_reader_active(const bc_reader_t *p_rd)
{
	return (p_rd != NULL) && p_rd->active;
}

uint8_t bc_reader_decode(const bc_reader_t *p_rd, char *p_text, uint8_t cap,
			 uint8_t *p_count)
{
	if (p_count != NULL) {
		*p_count = (p_rd != NULL) ? p_rd->pulse.count : 0u;
	}
	if ((p_rd == NULL) || (p_text == NULL) || (cap < 2u)) {
		return 0u;
	}
	return c39_decode(p_rd->pulse.widths_us, p_rd->pulse.count, p_text, cap);
}

static char bc_upper(char c)
{
	return ((c >= 'a') && (c <= 'z')) ? (char)(c - ('a' - 'A')) : c;
}

bool bc_letter_in(const char *p_letters, char c)
{
	if ((p_letters == NULL) || (c == '\0')) {
		return false;
	}
	for (; *p_letters != '\0'; p_letters++) {
		if (bc_upper(*p_letters) == bc_upper(c)) {
			return true;
		}
	}
	return false;
}

/* ======== from bc_rec.c ======== */
/*
 *	bc_rec.c
 *	Subsystem 3 (Line and Barcode) -- second look at a barcode that was
 *	not read (see bc_rec.h).
 */

#include <stddef.h>

void bc_rec_init(bc_rec_t *p_rec)
{
	if (p_rec != NULL) {
		p_rec->n = 0u;
		p_rec->pre_head = 0u;
		p_rec->pre_n = 0u;
		p_rec->t0_us = 0u;
		p_rec->last_act_us = 0u;
		p_rec->dropouts = 0u;
		p_rec->mask_ok = false;
		p_rec->active = false;
		p_rec->overflow = false;
	}
}

void bc_rec_sample(bc_rec_t *p_rec, uint32_t now_us, uint16_t raw)
{
	if (p_rec == NULL) {
		return;
	}
	if (p_rec->active) {
		uint32_t ms = (now_us - p_rec->t0_us) / 1000u;

		if ((p_rec->n < BC_REC_MAX) && (ms <= 0xFFFFu)) {
			p_rec->raw[p_rec->n] = raw;
			p_rec->ms[p_rec->n] = (uint16_t)ms;
			p_rec->n++;
		} else {
			p_rec->overflow = true;
		}
	} else {
		p_rec->pre_raw[p_rec->pre_head] = raw;
		p_rec->pre_us[p_rec->pre_head] = now_us;
		p_rec->pre_head = (uint16_t)((p_rec->pre_head + 1u) % BC_REC_PRE);
		if (p_rec->pre_n < BC_REC_PRE) {
			p_rec->pre_n++;
		}
	}
}

void bc_rec_start(bc_rec_t *p_rec)
{
	uint32_t i;
	uint32_t k;

	if (p_rec == NULL) {
		return;
	}
	p_rec->n = 0u;
	p_rec->overflow = false;
	p_rec->mask_ok = false;		/* dead[] was for the last one */
	p_rec->dropouts = 0u;
	if (p_rec->pre_n == 0u) {
		p_rec->active = false;		/* nothing to start from */
		return;
	}
	k = ((uint32_t)p_rec->pre_head + BC_REC_PRE - p_rec->pre_n) % BC_REC_PRE;	/* oldest */
	p_rec->t0_us = p_rec->pre_us[k];
	for (i = 0u; i < p_rec->pre_n; i++) {
		uint32_t ms = (p_rec->pre_us[k] - p_rec->t0_us) / 1000u;

		p_rec->raw[i] = p_rec->pre_raw[k];
		p_rec->ms[i] = (ms <= 0xFFFFu) ? (uint16_t)ms : 0xFFFFu;
		k = (k + 1u) % BC_REC_PRE;
	}
	p_rec->n = p_rec->pre_n;
	p_rec->pre_n = 0u;
	p_rec->pre_head = 0u;
	p_rec->active = true;
}

void bc_rec_stop(bc_rec_t *p_rec)
{
	if (p_rec != NULL) {
		p_rec->active = false;
		p_rec->pre_n = 0u;		/* the ring starts again from now */
		p_rec->pre_head = 0u;
	}
}

uint8_t bc_rec_decode(const bc_rec_t *p_rec, uint16_t white, uint16_t black,
		      uint32_t pct, uint32_t hyst_div, bc_reader_t *p_rd,
		      char *p_text, uint8_t cap, uint8_t *p_count)
{
	/* white after the recording, so the reader sees the end */
	const uint32_t	tail = (LF_BC_QUIET_MAX_US / 10000u) + 2u;
	int32_t		dir = (black > white) ? 1 : -1;
	int32_t		span = ((int32_t)black - (int32_t)white) * dir;
	int32_t		thr;
	int32_t		band;
	bool		dark = false;
	bool		known = false;
	uint32_t	i;
	uint8_t		n = 0u;

	if (p_count != NULL) {
		*p_count = 0u;
	}
	if ((p_text == NULL) || (cap < 2u)) {
		return 0u;
	}
	p_text[0] = '\0';
	if ((p_rec == NULL) || (p_rd == NULL) || p_rec->overflow || (p_rec->n == 0u) ||
	    (span <= 0) || (hyst_div == 0u) || (pct == 0u) || (pct >= 100u)) {
		return 0u;
	}
	thr = (int32_t)white + ((((int32_t)black - (int32_t)white) * (int32_t)pct) / 100);
	band = span / (int32_t)hyst_div;

	bc_reader_init(p_rd);
	for (i = 0u; i < ((uint32_t)p_rec->n + tail); i++) {
		uint16_t	raw;
		uint32_t	ms;
		int32_t		d;
		uint8_t		r;

		if (i < p_rec->n) {
			raw = p_rec->raw[i];
			ms = p_rec->ms[i];
		} else {
			raw = white;
			ms = (uint32_t)p_rec->ms[p_rec->n - 1u] + ((i - p_rec->n + 1u) * 10u);
		}
		d = ((int32_t)raw - thr) * dir;
		if ((i < p_rec->n) && bc_rec_is_dead(p_rec, i)) {
			/* a drop-out: keep what it was (white if nothing yet) */
		} else if (!known) {
			dark = (d >= 0);
			known = true;
		} else if (d > band) {
			dark = true;
		} else if (d < -band) {
			dark = false;
		} else {
			/* inside the hysteresis band: keep */
		}
		r = bc_reader_sample(p_rd, p_rec->t0_us + (ms * 1000u), dark, true);
		if (r == BC_RD_DONE) {
			n = bc_reader_decode(p_rd, p_text, cap, p_count);
			if (n > 0u) {
				return n;
			}
			/* a mark before the code: keep going */
		}
	}
	return n;
}

uint8_t bc_rec_watch(bc_rec_t *p_rec, uint32_t now_us, uint16_t raw, bool act,
		     bool allow, uint32_t tail_us)
{
	uint8_t ev = BC_REC_EV_NONE;

	if (p_rec == NULL) {
		return BC_REC_EV_NONE;
	}
	bc_rec_sample(p_rec, now_us, raw);
	if (!allow) {
		if (p_rec->active) {
			bc_rec_stop(p_rec);
			ev = BC_REC_EV_CANCELLED;
		}
	} else if (act) {
		p_rec->last_act_us = now_us;
		if (!p_rec->active) {
			bc_rec_start(p_rec);	/* this sample is in the ring: kept */
			ev = p_rec->active ? BC_REC_EV_STARTED : BC_REC_EV_NONE;
		}
	} else if (p_rec->active && ((now_us - p_rec->last_act_us) > tail_us)) {
		bc_rec_stop(p_rec);
		ev = BC_REC_EV_ENDED;
	} else {
		/* nothing new */
	}
	if ((ev == BC_REC_EV_NONE) && p_rec->active && (p_rec->n >= BC_REC_MAX)) {
		bc_rec_stop(p_rec);		/* full: read what there is */
		ev = BC_REC_EV_ENDED;
	} else {
		/* nothing new */
	}
	return ev;
}

bool bc_rec_levels(const bc_rec_t *p_rec, uint16_t *p_lo, uint16_t *p_hi)
{
	uint16_t	lo = 0xFFFFu;
	uint16_t	hi = 0u;
	uint32_t	i;

	if ((p_rec == NULL) || (p_rec->n == 0u)) {
		return false;
	}
	for (i = 0u; i < p_rec->n; i++) {
		if (bc_rec_is_dead(p_rec, i)) {
			continue;
		}
		if (p_rec->raw[i] < lo) {
			lo = p_rec->raw[i];
		}
		if (p_rec->raw[i] > hi) {
			hi = p_rec->raw[i];
		}
	}
	if (lo > hi) {
		return false;			/* all drop-outs */
	}
	if (p_lo != NULL) {
		*p_lo = lo;
	}
	if (p_hi != NULL) {
		*p_hi = hi;
	}
	return true;
}

bool bc_rec_is_dead(const bc_rec_t *p_rec, uint32_t i)
{
	return (p_rec != NULL) && p_rec->mask_ok && (i < p_rec->n) &&
	       ((p_rec->dead[i / 8u] & (uint8_t)(1u << (i % 8u))) != 0u);
}

static uint32_t bc_absdiff(uint16_t a, uint16_t b)
{
	return (a > b) ? ((uint32_t)a - b) : ((uint32_t)b - a);
}

/* The fall into raw[i] (the first low sample) is sudden: within 1 or 2
   samples, by at least jump, and 4 x the step just before it */
static bool bc_sudden_fall(const uint16_t *p_r, uint32_t i, uint32_t jump)
{
	uint32_t k;

	for (k = 1u; (k <= 2u) && (k <= i); k++) {
		uint32_t top = i - k;
		uint32_t s = (p_r[top] > p_r[i]) ? ((uint32_t)p_r[top] - p_r[i]) : 0u;
		uint32_t d = (top > 0u) ? bc_absdiff(p_r[top], p_r[top - 1u]) : 0u;

		if ((k == 2u) && ((p_r[i - 1u] > p_r[top]) || (p_r[i - 1u] < p_r[i]))) {
			continue;		/* not one fall */
		}
		if ((s >= jump) && (s >= (4u * d))) {
			return true;
		}
	}
	return false;
}

/* The rise after raw[j] (the last low sample) is sudden */
static bool bc_sudden_rise(const uint16_t *p_r, uint32_t n, uint32_t j, uint32_t jump)
{
	uint32_t k;

	for (k = 1u; (k <= 2u) && ((j + k) < n); k++) {
		uint32_t top = j + k;
		uint32_t s = (p_r[top] > p_r[j]) ? ((uint32_t)p_r[top] - p_r[j]) : 0u;
		uint32_t d = ((top + 1u) < n) ? bc_absdiff(p_r[top + 1u], p_r[top]) : 0u;

		if ((k == 2u) && ((p_r[j + 1u] > p_r[top]) || (p_r[j + 1u] < p_r[j]))) {
			continue;
		}
		if ((s >= jump) && (s >= (4u * d))) {
			return true;
		}
	}
	return false;
}

uint16_t bc_rec_find_dropouts(bc_rec_t *p_rec, uint16_t dead_raw, uint16_t jump)
{
	uint32_t	n;
	uint32_t	i = 0u;
	uint16_t	runs = 0u;

	if (p_rec == NULL) {
		return 0u;
	}
	n = p_rec->n;
	for (i = 0u; i < sizeof p_rec->dead; i++) {
		p_rec->dead[i] = 0u;
	}
	p_rec->mask_ok = true;
	i = 0u;
	while ((dead_raw > 0u) && (i < n)) {
		uint32_t j = i;

		if (p_rec->raw[i] >= dead_raw) {
			i++;
			continue;
		}
		while ((j < n) && (p_rec->raw[j] < dead_raw)) {
			j++;
		}
		/* [i, j): low.  Touching an end of the recording: the other
		   edge has to be sudden. */
		if (!((i == 0u) && (j == n)) &&
		    ((i == 0u) || bc_sudden_fall(p_rec->raw, i, jump)) &&
		    ((j == n) || bc_sudden_rise(p_rec->raw, n, j - 1u, jump))) {
			uint32_t k;

			for (k = i; k < j; k++) {
				p_rec->dead[k / 8u] |= (uint8_t)(1u << (k % 8u));
			}
			if (runs < 0xFFFFu) {
				runs++;
			}
		}
		i = j;
	}
	p_rec->dropouts = runs;
	return runs;
}

uint8_t bc_rec_decode_auto(const bc_rec_t *p_rec, bool cal_ok,
			   uint16_t cal_white, uint16_t cal_black,
			   uint32_t cal_pct, bool black_is_high,
			   uint32_t min_span, uint32_t hyst_div,
			   bc_reader_t *p_rd, char *p_text, uint8_t cap,
			   uint8_t *p_count, uint8_t *p_how)
{
	static const uint8_t	pcts[BC_REC_OWN_TRIES] = { 50u, 35u, 65u, 25u, 75u };
	static const uint8_t	hows[BC_REC_OWN_TRIES] = {
		BC_REC_HOW_OWN50, BC_REC_HOW_OWN35, BC_REC_HOW_OWN65,
		BC_REC_HOW_OWN25, BC_REC_HOW_OWN75 };
	uint16_t	lo = 0u;
	uint16_t	hi = 0u;
	uint8_t		n = 0u;
	uint8_t		cnt = 0u;
	uint32_t	k;

	if (p_how != NULL) {
		*p_how = BC_REC_HOW_NONE;
	}
	if (cal_ok) {
		n = bc_rec_decode(p_rec, cal_white, cal_black, cal_pct, hyst_div,
				  p_rd, p_text, cap, &cnt);
		if (n > 0u) {
			if (p_count != NULL) {
				*p_count = cnt;
			}
			if (p_how != NULL) {
				*p_how = BC_REC_HOW_CAL;
			}
			return n;
		}
	}
	if (!bc_rec_levels(p_rec, &lo, &hi) || ((uint32_t)(hi - lo) < min_span)) {
		return 0u;
	}
	for (k = 0u; k < BC_REC_OWN_TRIES; k++) {
		n = black_is_high
			? bc_rec_decode(p_rec, lo, hi, pcts[k], hyst_div, p_rd, p_text, cap, &cnt)
			: bc_rec_decode(p_rec, hi, lo, pcts[k], hyst_div, p_rd, p_text, cap, &cnt);
		if (n > 0u) {
			if (p_count != NULL) {
				*p_count = cnt;
			}
			if (p_how != NULL) {
				*p_how = hows[k];
			}
			return n;
		}
	}
	return 0u;
}
