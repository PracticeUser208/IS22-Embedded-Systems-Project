/*
 *	subsystem5.c
 *	Subsystem 5 -- Obstacle Scan.  Creates the kernel objects and starts
 *	the tasks (subsystem5_init / subsystem5_start), and holds the
 *	drivers and the planner that the scan task calls:
 *	  GPIO interrupt dispatcher   one IRQ line for all pins; Motion
 *	                              Control's encoders use it too
 *	  HC-SR04+ ultrasonic driver  GP4 / GP5
 *	  scan servo                  GP12
 *	  scan table + profile & plan width, clearance, side choice
 *	The task itself is in task_obstacle.c; see subsystem5.h for the
 *	four files.
 */

#include "subsystem5.h"
#include <tk/syslib.h>

#define S5_MBF_BYTES	(S5_Q_DEPTH * 8)	/* 4-byte message + header */

/* scan_q is ours (we receive it).  The others belong to other owners;
   these weak definitions are replaced at link time by the owner's own
   "EXPORT ID xxx;" as soon as one exists, and stay 0 until then, in
   which case subsystem5_init() creates a stand-in. */
EXPORT ID scan_q;
__attribute__((weak)) ID obst_q = 0;		/* Vehicle Controller */
__attribute__((weak)) ID flg_estop_req = 0;	/* Motion Control creates
						   it (subsystem2.c)  */
__attribute__((weak)) ID flg_sysfault = 0;	/* team, shared header */

LOCAL ID s5_cre_mbf(void)
{
	T_CMBF cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S5_MBF_BYTES;
	cmbf.maxmsz = (INT)sizeof(UW);
	return tk_cre_mbf(&cmbf);
}

LOCAL ID s5_cre_flg(void)
{
	T_CFLG cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	return tk_cre_flg(&cflg);
}

EXPORT ER subsystem5_init(void)
{
	ER er;

	scan_q = s5_cre_mbf();
	if (scan_q < E_OK) {
		return scan_q;
	}

	/* usermain() runs every other _init() first, so if an owner exists
	   its object is already there.  Otherwise make a stand-in. */
	if (obst_q <= 0) {
		obst_q = s5_cre_mbf();
		tm_printf((UB*)"S5: obst_q stand-in created (no Vehicle Controller yet)\n");
	}
	if (flg_estop_req <= 0) {
		flg_estop_req = s5_cre_flg();
		tm_printf((UB*)"S5: ESTOP_REQ stand-in created (no owner yet)\n");
	}
	if (flg_sysfault <= 0) {
		flg_sysfault = s5_cre_flg();
		tm_printf((UB*)"S5: SYSFAULT stand-in created (no owner yet)\n");
	}
	if ((obst_q < E_OK) || (flg_estop_req < E_OK) || (flg_sysfault < E_OK)) {
		return E_LIMIT;
	}

	er = s5_scan_task_init();	/* pins, echo interrupt, servo */
	if (er < E_OK) {
		tm_printf((UB*)"S5: echo interrupt not set up (%d): sensor fault\n", er);
	}
	return E_OK;	/* a hardware problem shows as SENSOR_FAULT, so the
			   rest of the system still starts (FR20) */
}

EXPORT ER subsystem5_start(void)
{
	ID tid;

	tid = app_start_task(s5_scan_task, S5_PRI_SCAN, S5_SCAN_CORE, "s5_scan");
	if (tid < E_OK) {
		return tid;
	}
#if S5_STUB_VC
	tid = app_start_task(s5_stub_vc_task, S5_PRI_STUB_VC, 0, "s5_stub_vc");
	if (tid < E_OK) {
		return tid;
	}
#endif
	return E_OK;
}

/* =====================================================================
 *  GPIO interrupt dispatcher (was gpio_irq.c)
 * =====================================================================
 *	Shared GPIO edge-interrupt dispatcher.  See subsystem5.h for why.
 *
 *	Register map (RP2040 datasheet, IO_BANK0):
 *	  INTR0..3        0x0F0  raw status; edge bits are write-1-to-clear
 *	  PROC0_INTE0..3  0x100  enable, core 0
 *	  PROC0_INTS0..3  0x120  status after masking, core 0
 *	  PROC1_INTE0..3  0x130  enable, core 1
 *	  PROC1_INTS0..3  0x150  status after masking, core 1
 *	Each register covers 8 GPIOs, 4 bits per GPIO:
 *	  bit 0 level low, bit 1 level high, bit 2 edge low, bit 3 edge high.
 *
 *	The NVIC is per core: EnableInt() enables IRQ 13 on the core that
 *	calls it, so the handler always runs there.  The core is recorded at
 *	the first attach, and that core's INTE/INTS registers are used for
 *	every pin.
 */

#define IOB_INTR0		(IO_BANK0_BASE + 0x0F0U)
#define IOB_INTE0(core)		(IO_BANK0_BASE + 0x100U + ((core) * 0x30U))
#define IOB_INTS0(core)		(IO_BANK0_BASE + 0x120U + ((core) * 0x30U))
#define IOB_ALIAS_SET		0x2000U	/* atomic bit-set alias */
#define IOB_ALIAS_CLR		0x3000U	/* atomic bit-clear alias */

#define GIRQ_NUM_GPIO		30U
#define GIRQ_EDGE_MASK		(GIRQ_EDGE_FALL | GIRQ_EDGE_RISE)

#define TIMER_RAWL_REG		(TIMER_TIMERAWL)

typedef struct {
	UB	pin;
	UB	used;
	GIRQ_FN	fn;
} GIRQ_SLOT;

LOCAL GIRQ_SLOT	girq_tbl[GIRQ_MAX_PINS];
LOCAL UW	girq_core;		/* 0 or 1: core that owns IRQ 13 */
LOCAL BOOL	girq_installed = FALSE;

EXPORT UW gpio_irq_now_us(void)
{
	return in_w(TIMER_RAWL_REG);
}

/*
 * The one IO_IRQ_BANK0 handler.  Reads the masked status for every pin
 * that has a callback, clears the edges it saw, then calls back.
 */
LOCAL void gpio_irq_handler(UINT intno)
{
	UW	now = in_w(TIMER_RAWL_REG);	/* timestamp first */
	UINT	i;

	for (i = 0U; i < GIRQ_MAX_PINS; i++) {
		if (girq_tbl[i].used != 0U) {
			UINT	pin   = girq_tbl[i].pin;
			UW	reg   = IOB_INTS0(girq_core) + ((pin >> 3) * 4U);
			UW	shift = (pin & 7U) * 4U;
			UW	ev    = (in_w(reg) >> shift) & GIRQ_EDGE_MASK;

			if (ev != 0U) {
				/* write-1-to-clear the edges just seen */
				out_w(IOB_INTR0 + ((pin >> 3) * 4U), ev << shift);
				girq_tbl[i].fn(pin, ev, now);
			}
		}
	}
	ClearInt(intno);
}

EXPORT ER gpio_irq_attach(UINT pin, UW edges, GIRQ_FN fn)
{
	UINT	i;
	INT	free_slot = -1;
	UW	shift;
	UW	reg_off;

	if ((pin >= GIRQ_NUM_GPIO) || (fn == NULL) ||
	    ((edges & GIRQ_EDGE_MASK) == 0U) ||
	    ((edges & ~GIRQ_EDGE_MASK) != 0U)) {
		return E_PAR;
	}

	for (i = 0U; i < GIRQ_MAX_PINS; i++) {
		if (girq_tbl[i].used != 0U) {
			if (girq_tbl[i].pin == pin) {
				return E_OBJ;		/* one owner per pin */
			}
		} else if (free_slot < 0) {
			free_slot = (INT)i;
		}
	}
	if (free_slot < 0) {
		return E_LIMIT;
	}

	if (girq_installed == FALSE) {
		const T_DINT dint = {
			.intatr = TA_HLNG,
			.inthdr = (FP)gpio_irq_handler,
		};
		ER er = tk_def_int(GIRQ_INTNO, &dint);

		if (er < E_OK) {
			return er;
		}
		girq_core = in_w(SIO_CPUID_ADDR) & 1U;	/* core taking IRQ 13 */
		ClearInt(GIRQ_INTNO);
		EnableInt(GIRQ_INTNO, GIRQ_INTPRI);
		girq_installed = TRUE;
	}

	shift   = (pin & 7U) * 4U;
	reg_off = (pin >> 3) * 4U;

	/* Fill the slot before enabling the pin's interrupt */
	girq_tbl[free_slot].pin  = (UB)pin;
	girq_tbl[free_slot].fn   = fn;
	girq_tbl[free_slot].used = 1U;
	__asm__ volatile ("dmb" ::: "memory");	/* slot visible to the ISR first */

	out_w(IOB_INTR0 + reg_off, GIRQ_EDGE_MASK << shift);	/* drop stale edges */
	out_w(IOB_INTE0(girq_core) + reg_off + IOB_ALIAS_SET,
	      (edges & GIRQ_EDGE_MASK) << shift);		/* atomic enable */
	return E_OK;
}

/* =====================================================================
 *  HC-SR04+ ultrasonic driver (was drv_ultrasonic.c)
 * =====================================================================
 *	HC-SR04+ driver: trigger on GP4, echo timed by an edge interrupt on GP5.
 *
 *	Race-free hand-off between the echo ISR and the task (Tribunal 1:
 *	"the echo-interrupt data must stay race-free"):
 *	  - us_armed_seq is written only by the task, before it triggers
 *	  - us_rise_us / us_rise_seen are written by the ISR, and by the task
 *	    only while ECHO is low (no edge can be in flight then)
 *	  - the ISR publishes the result as ONE aligned 32-bit store
 *	      31..24 sequence number of the ping it belongs to
 *	      16     valid
 *	      15..0  pulse width in microseconds (clamped at 65535)
 *	    so the task never sees half an update, and a late echo from an
 *	    older ping is rejected because its sequence number is stale
 *	  - the event flag (tk_set_flg in the ISR, tk_wai_flg in the task)
 *	    orders the two across cores
 */

#define US_FLG_DONE		0x01U

#define US_TRIG_US		12U	/* datasheet minimum is 10 us        */
#define US_MIN_WIDTH_US		100U	/* shorter is a glitch, not an echo  */
#define US_MAX_WIDTH_US		23300U	/* ~400 cm, the sensor's rated max   */
#define US_RES_VALID		0x00010000U

/* Speed of sound 343 m/s: cm = us / 58.3, done as (us * 10 + 291) / 583 */
#define US_UM_PER_US_X10	583U

#define RESETS_ALIAS_CLR	0x3000U
#define WATCHDOG_TICK_REG	(0x40058000U + 0x2CU)
#define WATCHDOG_TICK_ENABLE	(1U << 9)
#define WATCHDOG_TICK_RUNNING	(1U << 10)
#define WATCHDOG_TICK_CYCLES	0x1FFU
#define XOSC_MHZ		12U	/* clk_ref = 12 MHz crystal (cpu_clock.c) */

LOCAL ID		us_flgid;
LOCAL UB		us_seq;			/* task only */
LOCAL volatile UB	us_armed_seq;		/* task writes, ISR reads */
LOCAL volatile UW	us_rise_us;		/* ISR */
LOCAL volatile BOOL	us_rise_seen;		/* ISR (task only while ECHO low) */
LOCAL volatile UW	us_result;		/* ISR writes, task reads */
LOCAL UW		us_high_since;		/* task: when ECHO was first seen high */
LOCAL BOOL		us_high_seen;
LOCAL BOOL		us_timer_running;	/* FALSE: never trigger (would spin) */
EXPORT UW		us_last_width_us;	/* last raw echo width, for printing */

/* ---------------------------------------------------------------------
 *  Echo edge callback.  Interrupt context: a few lines, no waiting.
 * --------------------------------------------------------------------- */
LOCAL void us_echo_cb(UINT pin, UW events, UW now_us)
{
	(void)pin;

	if ((events & GIRQ_EDGE_RISE) != 0U) {
		us_rise_us   = now_us;
		us_rise_seen = TRUE;
	}
	if (((events & GIRQ_EDGE_FALL) != 0U) && (us_rise_seen != FALSE)) {
		UW width = now_us - us_rise_us;

		if (width > 0xFFFFU) {
			width = 0xFFFFU;
		}
		us_rise_seen = FALSE;
		us_result = ((UW)us_armed_seq << 24) | US_RES_VALID | width;
		(void)tk_set_flg(us_flgid, US_FLG_DONE);
	}
}

/* ---------------------------------------------------------------------
 *  The echo is timed with the 1 MHz hardware timer, so check it really
 *  runs at 1 MHz.  The timer counts the watchdog tick, which divides the
 *  12 MHz reference clock by TICK.CYCLES:
 *    1. read the TICK register: running, 12 cycles = 1 MHz.  If not,
 *       start it (the pico-sdk does the same at start-up).  This is the
 *       check that decides.
 *    2. as a second opinion, measure it against kernel time over 200 ms.
 *       Kernel time can lose ticks while the WiFi start-up holds the CPU,
 *       which makes the timer look FAST (car log 25 Oct: 1582 kHz once,
 *       999 kHz right after), so a fast reading is measured again and,
 *       if the register is right, only reported.  A SLOW reading cannot
 *       come from lost ticks, so that one is a real fault.
 *  These steps only turn things ON.  Returns TRUE if the timer is good.
 * --------------------------------------------------------------------- */
LOCAL UW us_now_ms(void)
{
	SYSTIM t;

	(void)tk_get_tim(&t);
	return t.lo;
}

LOCAL UW us_timer_khz(void)
{
	UW t0_ms = us_now_ms();
	UW t0 = gpio_irq_now_us();
	UW dt_ms;

	(void)tk_dly_tsk(200);
	dt_ms = us_now_ms() - t0_ms;
	return (dt_ms == 0U) ? 0U : ((gpio_irq_now_us() - t0) / dt_ms);
}

LOCAL BOOL us_tick_ok(UW tick)
{
	return (((tick & WATCHDOG_TICK_RUNNING) != 0U) &&
		((tick & WATCHDOG_TICK_CYCLES) == XOSC_MHZ)) ? TRUE : FALSE;
}

EXPORT BOOL us_timer_check(void)
{
	UW tick;
	UW khz;
	INT i;

	if ((in_w(RESETS_RESET) & RESETS_RESET_TIMER) != 0U) {
		out_w(RESETS_RESET + RESETS_ALIAS_CLR, RESETS_RESET_TIMER);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER) == 0U) {
			;	/* a few cycles */
		}
	}

	/* 1. the register */
	tick = in_w(WATCHDOG_TICK_REG);
	if (us_tick_ok(tick) == FALSE) {
		out_w(WATCHDOG_TICK_REG, WATCHDOG_TICK_ENABLE | XOSC_MHZ);
		tm_printf((UB*)"S5: 1 us tick was off (TICK=0x%03x): started it from the 12 MHz clock\n",
			  tick & 0x7FFU);
	}

	/* 2. the measurement (up to 3 tries if it reads fast) */
	khz = 0U;
	for (i = 0; i < 3; i++) {
		khz = us_timer_khz();
		if (khz <= 1100U) {
			break;
		}
	}
	tick = in_w(WATCHDOG_TICK_REG);
	tm_printf((UB*)"S5: echo timer runs at %d kHz (should be 1000), TICK=0x%03x %s\n",
		  khz, tick & 0x7FFU, (us_tick_ok(tick) != FALSE) ? "ok" : "WRONG");
	if ((khz > 1100U) && (us_tick_ok(tick) != FALSE)) {
		tm_printf((UB*)"S5: (reads fast because kernel time lost ticks at start-up; "
			  "the register is right, so carrying on)\n");
	}

	us_timer_running = ((us_tick_ok(tick) != FALSE) && (khz >= 900U)) ? TRUE : FALSE;
	return us_timer_running;
}

EXPORT ER us_init(void)
{
	T_CFLG	cflg = {0};
	ER	er;

	gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
	gpio_set_val(US_TRIG_PIN, 0U);
	gpio_set_pin(US_ECHO_PIN, GPIO_MODE_IN);	/* pad pull-down stays on */

	cflg.flgatr  = TA_TFIFO | TA_WSGL;
	cflg.iflgptn = 0U;
	us_flgid = tk_cre_flg(&cflg);
	if (us_flgid < E_OK) {
		return us_flgid;
	}

	er = gpio_irq_attach(US_ECHO_PIN, GIRQ_EDGE_RISE | GIRQ_EDGE_FALL,
			     us_echo_cb);
	return er;
}

/* Busy-wait a few microseconds on the hardware timer (trigger pulse only) */
LOCAL void us_wait_us(UW us)
{
	UW t0 = gpio_irq_now_us();

	while ((gpio_irq_now_us() - t0) < us) {
		;
	}
}

EXPORT UH us_ping(void)
{
	UINT	ptn = 0U;
	ER	er;
	UW	res;
	UW	width;

	if (us_timer_running == FALSE) {
		return US_MAKE(US_NONE, 0U);	/* cannot time an echo */
	}

	/* ECHO still high: the last pulse has not finished (some modules hold
	   it ~40 ms when nothing is in range).  Skip, and call it stuck only
	   if it stays high for US_STUCK_MS. */
	if (gpio_get_val(US_ECHO_PIN) != 0U) {
		UW now = gpio_irq_now_us();

		if (us_high_seen == FALSE) {
			us_high_seen  = TRUE;
			us_high_since = now;
		} else if ((now - us_high_since) >= ((UW)US_STUCK_MS * 1000U)) {
			return US_MAKE(US_STUCK, 0U);
		}
		return US_MAKE(US_BUSY, 0U);
	}
	us_high_seen = FALSE;

	/* Arm: ECHO is low, so no edge is in flight */
	us_seq++;
	us_rise_seen = FALSE;
	us_armed_seq = us_seq;
	(void)tk_clr_flg(us_flgid, ~US_FLG_DONE);

	gpio_set_val(US_TRIG_PIN, 1U);
	us_wait_us(US_TRIG_US);
	gpio_set_val(US_TRIG_PIN, 0U);

	er = tk_wai_flg(us_flgid, US_FLG_DONE, TWF_ORW | TWF_BITCLR, &ptn,
			US_ECHO_TMO_MS);
	if (er == E_TMOUT) {
		/* Echo started but never ended in time: nothing within ~4 m.
		   Echo never started: no sensor (or no power) on the pins. */
		return (us_rise_seen != FALSE) ? US_MAKE(US_OPEN, 0U)
					      : US_MAKE(US_NONE, 0U);
	}
	if (er < E_OK) {
		return US_MAKE(US_NONE, 0U);
	}

	res = us_result;
	if ((UB)(res >> 24) != us_seq) {
		return US_MAKE(US_NONE, 0U);		/* stale echo */
	}
	width = res & 0xFFFFU;
	us_last_width_us = width;
	if (width < US_MIN_WIDTH_US) {
		return US_MAKE(US_NONE, 0U);		/* glitch */
	}
	if (width > US_MAX_WIDTH_US) {
		return US_MAKE(US_OPEN, 0U);
	}
	return US_MAKE(US_OK, (width * 10U + (US_UM_PER_US_X10 / 2U)) / US_UM_PER_US_X10);
}

/* =====================================================================
 *  Scan servo (was drv_servo.c)
 * =====================================================================
 *	50 Hz servo PWM on GP12 using the board helpers in bsp/libbsp.h plus
 *	the slice clock divider, which the helpers do not set.
 *
 *	clk_sys is 125 MHz (cpu_clock.c: PLL SYS 125 MHz).  Divider 125 gives
 *	a 1 MHz count, so TOP = 19999 is a 20 ms period and the compare value
 *	is the pulse width in microseconds.
 *
 *	The kernel only releases the PWM block from reset when its physical
 *	timer is enabled (USE_PTMR, off in config.h), so this driver releases
 *	it.  It only ever CLEARS the reset bit, using the atomic clear alias,
 *	so it is safe if Motion Control's motor PWM did the same first.
 */

#define CLK_SYS_MHZ		125U
#define SERVO_SLICE		((SERVO_PIN >> 1) & 7U)		/* = 6 */
#define SERVO_DIV_REG		(PWM_BASE + PWM_CHx_DIV + (SERVO_SLICE * 0x14U))
#define SERVO_TOP		19999U

LOCAL W servo_deg_now;

EXPORT ER servo_init(void)
{
	if ((in_w(RESETS_RESET) & RESETS_RESET_PWM) != 0U) {
		out_w(RESETS_RESET + RESETS_ALIAS_CLR, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
			;	/* a few cycles */
		}
	}

	out_w(SERVO_DIV_REG, CLK_SYS_MHZ << 4);		/* INT field, FRAC 0 */
	(void)pwm_set_wrap(SERVO_PIN, SERVO_TOP);
	(void)pwm_set_cc(SERVO_PIN, (UW)SERVO_CENTER_US);
	(void)pwm_set_pin(SERVO_PIN);
	(void)pwm_set_enabled(SERVO_PIN, TRUE);
	servo_deg_now = 0;
	return E_OK;
}

EXPORT void servo_set_deg(W deg)
{
	W us;

	if (deg > SERVO_LIMIT_LEFT_DEG) {
		deg = SERVO_LIMIT_LEFT_DEG;
	} else if (deg < SERVO_LIMIT_RIGHT_DEG) {
		deg = SERVO_LIMIT_RIGHT_DEG;
	}

	/* rounded: us = centre + dir * deg * 1000 / 90 */
	us = (SERVO_DIR * deg * SERVO_US_PER_90DEG);
	us = (us >= 0) ? ((us + 45) / 90) : ((us - 45) / 90);
	us += SERVO_CENTER_US;
	if (us < SERVO_MIN_US) {
		us = SERVO_MIN_US;
	} else if (us > SERVO_MAX_US) {
		us = SERVO_MAX_US;
	}

	(void)pwm_set_cc(SERVO_PIN, (UW)us);
	servo_deg_now = deg;
}

EXPORT BOOL servo_deg_ok(W deg)
{
	return ((deg <= SERVO_LIMIT_LEFT_DEG) && (deg >= SERVO_LIMIT_RIGHT_DEG)) ? TRUE : FALSE;
}

EXPORT W servo_get_deg(void)
{
	return servo_deg_now;
}

EXPORT INT servo_settle_ms(W delta_deg)
{
	if (delta_deg < 0) {
		delta_deg = -delta_deg;
	}
	return (INT)(40 + (3 * delta_deg));
}

/* =====================================================================
 *  Scan table and profile & plan (was obstacle_plan.c)
 * =====================================================================
 *	Profile & plan: obstacle width, room on each side, which side to take.
 *
 *	Geometry (car frame, cm): y forward, x to the LEFT.  A reading of d cm
 *	at angle a (+ = left) is the point x = d sin a, y = d cos a.
 *
 *	1. The obstacle is the nearest reading within +-S5_AHEAD_DEG that is
 *	   closer than S5_OBJECT_CM.  None: the path is clear.
 *	2. Grow it slot by slot to the left and to the right while each new
 *	   reading is within S5_JUMP_CM of the last one.  A bigger jump, an
 *	   "open" reading or the end of the table is an edge.
 *	3. Width = x(left edge) - x(right edge).  The HC-SR04 beam is ~15 deg
 *	   wide, so each edge is seen a few degrees early; the edge angles
 *	   are pulled in by S5_BEAM_CORR_DEG before the width is worked out.
 *	4. Room on a side = the smallest sideways gap between the obstacle's
 *	   edge and anything seen beyond that edge, counting only things no
 *	   deeper than the obstacle + S5_DEPTH_CM (further away they do not
 *	   block the car going round).  Measured but empty = S5_CLEAR_OPEN_CM.
 *	   Nothing measured beyond the edge (the obstacle runs off the scan,
 *	   or past the servo's safe travel) = unknown, so 0.
 *	5. Take the side with more room if it is at least S5_MIN_CLEAR_CM,
 *	   else BLOCKED.  A tie goes left.
 *	All integer maths (the M0+ has no FPU).
 */

LOCAL const UH s5_sin_tbl[91] = {
	0, 71, 143, 214, 286, 357, 428, 499, 570, 641,
	711, 782, 852, 921, 991, 1060, 1129, 1198, 1266, 1334,
	1401, 1468, 1534, 1600, 1666, 1731, 1796, 1860, 1923, 1986,
	2048, 2110, 2171, 2231, 2290, 2349, 2408, 2465, 2522, 2578,
	2633, 2687, 2741, 2793, 2845, 2896, 2946, 2996, 3044, 3091,
	3138, 3183, 3228, 3271, 3314, 3355, 3396, 3435, 3474, 3511,
	3547, 3582, 3617, 3650, 3681, 3712, 3742, 3770, 3798, 3824,
	3849, 3873, 3896, 3917, 3937, 3956, 3974, 3991, 4006, 4021,
	4034, 4046, 4056, 4065, 4074, 4080, 4086, 4090, 4094, 4095,
	4096,
};

EXPORT W s5_sin_q12(W deg)
{
	if (deg > 90) {
		deg = 90;
	} else if (deg < -90) {
		deg = -90;
	}
	return (deg >= 0) ? (W)s5_sin_tbl[deg] : -(W)s5_sin_tbl[-deg];
}

LOCAL W s5_cos_q12(W deg)
{
	if (deg < 0) {
		deg = -deg;
	}
	return (deg > 90) ? 0 : (W)s5_sin_tbl[90 - deg];
}

LOCAL W s5_abs(W v)
{
	return (v < 0) ? -v : v;
}

/* lateral (x) and forward (y) position of a reading, in cm */
LOCAL W s5_x_cm(W d, W deg) { return (d * s5_sin_q12(deg)) / 4096; }
LOCAL W s5_y_cm(W d, W deg) { return (d * s5_cos_q12(deg)) / 4096; }

EXPORT void s5_scan_clear(S5_SCAN *s)
{
	INT i;

	for (i = 0; i < S5_SCAN_SLOTS; i++) {
		s->cm[i] = S5_CM_NONE;
	}
}

EXPORT INT s5_slot_of(W deg)
{
	W off = deg - S5_SCAN_MIN_DEG;

	if ((deg < S5_SCAN_MIN_DEG) || (deg > S5_SCAN_MAX_DEG) ||
	    ((off % S5_SCAN_STEP_DEG) != 0)) {
		return -1;
	}
	return (INT)(off / S5_SCAN_STEP_DEG);
}

EXPORT W s5_deg_of(INT slot)
{
	return S5_SCAN_MIN_DEG + ((W)slot * S5_SCAN_STEP_DEG);
}

EXPORT void s5_scan_put(S5_SCAN *s, W deg, UH cm)
{
	INT slot = s5_slot_of(deg);

	if (slot >= 0) {
		s->cm[slot] = s5_clamp9(cm);
	}
}

EXPORT UH s5_scan_get(const S5_SCAN *s, W deg)
{
	INT slot = s5_slot_of(deg);

	return (slot >= 0) ? s->cm[slot] : S5_CM_NONE;
}

LOCAL BOOL s5_is_hit(UH cm)	/* a real reading nearer than S5_OBJECT_CM */
{
	return ((cm != S5_CM_NONE) && (cm != S5_CM_OPEN) &&
		(cm <= (UH)S5_OBJECT_CM)) ? TRUE : FALSE;
}

EXPORT BOOL s5_fine_range(const S5_SCAN *s, W *from_deg, W *to_deg)
{
	W deg;
	W lo = S5_SCAN_MAX_DEG + 1;
	W hi = S5_SCAN_MIN_DEG - 1;

	for (deg = S5_SCAN_MIN_DEG; deg <= S5_SCAN_MAX_DEG; deg += S5_SCAN_STEP_DEG) {
		if (s5_is_hit(s5_scan_get(s, deg)) != FALSE) {
			if (deg < lo) { lo = deg; }
			if (deg > hi) { hi = deg; }
		}
	}
	if (lo > hi) {
		return FALSE;
	}
	lo -= S5_COARSE_STEP_DEG;
	hi += S5_COARSE_STEP_DEG;
	*from_deg = (lo < S5_SCAN_MIN_DEG) ? S5_SCAN_MIN_DEG : lo;
	*to_deg   = (hi > S5_SCAN_MAX_DEG) ? S5_SCAN_MAX_DEG : hi;
	return TRUE;
}

/* Smallest sideways gap beyond one edge.  dir = +1 left, -1 right. */
LOCAL UH s5_side_clear(const S5_SCAN *s, INT edge_slot, INT dir,
		       W edge_x, W max_depth)
{
	INT	i;
	W	best = S5_CLEAR_OPEN_CM;
	BOOL	seen = FALSE;		/* anything measured beyond the edge? */

	for (i = edge_slot + dir; (i >= 0) && (i < S5_SCAN_SLOTS); i += dir) {
		UH	cm = s->cm[i];
		W	deg = s5_deg_of(i);
		W	gap;

		if (cm == S5_CM_NONE) {
			continue;	/* not measured (e.g. past the servo limit) */
		}
		seen = TRUE;
		if (cm == S5_CM_OPEN) {
			continue;	/* measured, nothing there */
		}
		if (s5_y_cm((W)cm, deg) > max_depth) {
			continue;	/* far behind: does not block */
		}
		gap = (W)dir * (s5_x_cm((W)cm, deg) - edge_x);
		if (gap < 0) {
			gap = 0;
		}
		if (gap < best) {
			best = gap;
		}
	}
	/* Nothing measured beyond the edge (it runs off the scan, or the servo
	   cannot turn that far): the room there is unknown, so count it as none */
	return (seen != FALSE) ? (UH)best : 0U;
}

EXPORT void s5_plan(const S5_SCAN *s, S5_PLAN *p)
{
	INT	i;
	INT	near_slot = -1;
	INT	left, right;
	UH	prev;
	W	a_left, a_right, x_left, x_right, width;

	p->result = S5_PLAN_CLEAR;
	p->side = S5_SIDE_NONE;
	p->left_deg = 0;
	p->right_deg = 0;
	p->dist_cm = S5_CM_OPEN;
	p->width_cm = 0U;
	p->clear_left_cm = S5_CLEAR_OPEN_CM;
	p->clear_right_cm = S5_CLEAR_OPEN_CM;

	/* 1. nearest hit straight-ish ahead (ties: the one nearer 0 deg) */
	for (i = 0; i < S5_SCAN_SLOTS; i++) {
		W deg = s5_deg_of(i);

		if ((s5_abs(deg) <= S5_AHEAD_DEG) && (s5_is_hit(s->cm[i]) != FALSE)) {
			BOOL nearer = (near_slot < 0) || (s->cm[i] < s->cm[near_slot]);
			BOOL tie_more_ahead = (near_slot >= 0) &&
				(s->cm[i] == s->cm[near_slot]) &&
				(s5_abs(deg) < s5_abs(s5_deg_of(near_slot)));

			if ((nearer != FALSE) || (tie_more_ahead != FALSE)) {
				near_slot = i;
			}
		}
	}
	if (near_slot < 0) {
		return;				/* CLEAR */
	}
	p->dist_cm = s->cm[near_slot];

	/* 2. grow left (+ angle = higher slot) and right */
	left = near_slot;
	prev = s->cm[near_slot];
	for (i = near_slot + 1; i < S5_SCAN_SLOTS; i++) {
		UH cm = s->cm[i];
		if (cm == S5_CM_NONE) { continue; }		/* not measured */
		if ((cm == S5_CM_OPEN) || (s5_abs((W)cm - (W)prev) > S5_JUMP_CM)) {
			break;
		}
		left = i;
		prev = cm;
	}
	right = near_slot;
	prev = s->cm[near_slot];
	for (i = near_slot - 1; i >= 0; i--) {
		UH cm = s->cm[i];
		if (cm == S5_CM_NONE) { continue; }
		if ((cm == S5_CM_OPEN) || (s5_abs((W)cm - (W)prev) > S5_JUMP_CM)) {
			break;
		}
		right = i;
		prev = cm;
	}
	p->left_deg  = (B)s5_deg_of(left);
	p->right_deg = (B)s5_deg_of(right);

	/* 3. width, edges pulled in by the beam correction */
	a_left  = s5_deg_of(left) - S5_BEAM_CORR_DEG;
	a_right = s5_deg_of(right) + S5_BEAM_CORR_DEG;
	if (a_left < a_right) {			/* narrower than the beam */
		a_left = (a_left + a_right) / 2;
		a_right = a_left;
	}
	x_left  = s5_x_cm((W)s->cm[left], a_left);
	x_right = s5_x_cm((W)s->cm[right], a_right);
	width = x_left - x_right;
	if (width < 1) {
		width = 1;
	}
	p->width_cm = (UH)((width > (W)S5_WIDTH_MAX8) ? (W)S5_WIDTH_MAX8 : width);

	/* 4. room each side, measured from the uncorrected edge points */
	{
		W max_depth = (W)p->dist_cm + S5_DEPTH_CM;
		W edge_xl = s5_x_cm((W)s->cm[left], s5_deg_of(left));
		W edge_xr = s5_x_cm((W)s->cm[right], s5_deg_of(right));

		p->clear_left_cm  = s5_side_clear(s, left, +1, edge_xl, max_depth);
		p->clear_right_cm = s5_side_clear(s, right, -1, edge_xr, max_depth);
	}

	/* 5. choose */
	if ((p->clear_left_cm < (UH)S5_MIN_CLEAR_CM) &&
	    (p->clear_right_cm < (UH)S5_MIN_CLEAR_CM)) {
		p->result = S5_PLAN_BLOCKED;
		p->side = S5_SIDE_NONE;
	} else {
		p->result = S5_PLAN_BYPASS;
		p->side = (p->clear_left_cm >= p->clear_right_cm) ? S5_SIDE_LEFT
								  : S5_SIDE_RIGHT;
	}
}
