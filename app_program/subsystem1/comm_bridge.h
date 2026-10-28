/*
 *	comm_bridge.h
 *	Subsystem 1 (Buddy 1) -- hooks between subsystem 1 and the MQTT client
 *	that runs inside the kernel's WiFi service task (mqtt_utk.c).
 *
 *	Why: TEAM.md section 3 forbids any application task from calling
 *	cyw43 / lwIP / MQTT functions.  So:
 *	  - comm_task (ours) encodes frames and queues them in mbf_s1_tx;
 *	  - mqtt_utk.c (kernel WiFi task) calls the s1_bridge_* hooks below:
 *	    it takes queued frames and publishes them, and hands every
 *	    received command to us.  Our hook code never calls MQTT itself.
 *	The hooks run in the WiFi service task, so they never block: every
 *	kernel call inside them uses TMO_POL.
 *
 *	Kernel-free on purpose (stdint only): mqtt_utk.c must not include
 *	tk/tkernel.h (type clash with the pico-sdk headers it uses).
 */

#ifndef COMM_BRIDGE_H
#define COMM_BRIDGE_H

#include <stdint.h>
#include "telem_format.h"

/* Topic IDs inside mbf_s1_tx (1st byte of each queued message) */
#define S1_TX_ID_TM		0u
#define S1_TX_ID_HB		1u
#define S1_TX_MSG_MAX		(1u + TF_TM_LEN)	/* id + largest frame */

/* ---- Called by mqtt_utk.c (WiFi service task) ----------------------- */

/* Next frame to publish.  Returns its length (0 = nothing queued) and
   sets *pp_topic.  Never blocks. */
uint16_t s1_bridge_take_tx(char const **pp_topic, uint8_t *p_buf, uint16_t cap);

/* A complete message arrived on TF_TOPIC_CMD.  Decodes it, forwards it to
   remote_cmd_q, and writes the ACK frame to publish on TF_TOPIC_ACK.
   Returns the ACK length (0 = nothing to send).  Never blocks. */
uint16_t s1_bridge_on_cmd(uint8_t const *p_data, uint16_t len,
                          uint8_t *p_ack, uint16_t cap);

/* MQTT session state changes (1 = connected and subscribed, 0 = lost) */
void s1_bridge_set_online(uint8_t b_online);

/* ---- Read by comm_task (single writer: the WiFi service task) ------ */
uint8_t  s1_bridge_is_online(void);
uint32_t s1_bridge_reconnects(void);

#endif /* COMM_BRIDGE_H */