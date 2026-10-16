/**
 * crsf.h - CRSF (ExpressLRS/Crossfire) receiver parser and arm/e-stop logic.
 *
 * State machine: DISARMED -> ARMED when the arm switch is on and was seen off
 * since boot / e-stop; ARMED -> DISARMED when it goes off; either -> ESTOP on
 * failsafe (link lost / low LQ) or the e-stop switch; ESTOP -> DISARMED only
 * with the arm switch off, no failsafe and the e-stop released. ESTOP never
 * goes straight to ARMED.
 *
 * Pure C, no firmware dependencies, so it is unit-tested on the host
 * (test/test_crsf.c). Channel meaning follows the robochief radio setup
 * (robochief-ros2-control docs/crsf_bringup.md), 1-based:
 *   ch1 strafe  ch2 forward  ch4 yaw   ch5 arm   ch6 mode   ch9 e-stop
 */
#ifndef CRSF_H_
#define CRSF_H_

#include <stdint.h>
#include <stdbool.h>

#define CRSF_BAUD             420000
#define CRSF_NUM_CHANNELS     16
#define CRSF_LINK_TIMEOUT_MS  250   // no RC frame for this long = link lost

#define CRSF_MODE_MANUAL      0
#define CRSF_MODE_STABILIZED  1
#define CRSF_MODE_AUTO        2

typedef struct {
  // parser
  uint8_t  buf[64];
  uint8_t  idx;

  // from the receiver
  uint16_t ch[CRSF_NUM_CHANNELS];  // raw CRSF units: 172..1811, centre 992
  uint8_t  rssi;                   // uplink RSSI, -dBm
  uint8_t  lq;                     // uplink link quality, 0..100
  uint32_t frames;                 // good RC frames
  uint32_t crcErrors;
  uint32_t lastRcMs;
  bool     haveRc;
  bool     haveStats;

  // derived by crsfTick()
  uint8_t  link;                   // fresh RC frames and link quality >= lqMin
  uint8_t  armed;                  // state ARMED
  uint8_t  estop;                  // state ESTOP (armed = estop = 0 is DISARMED)
  uint8_t  mode;                   // CRSF_MODE_*
  uint8_t  armSeenOff;             // arm switch seen off since boot / e-stop / last arming
  uint16_t ageMs;                  // time since last RC frame, saturated
  float    fwd, strafe, yaw;       // -1..1, zero unless armed, MANUAL/STABILIZED and linked
  float    deadband;               // tune: stick deadband, fraction of full travel
  uint8_t  lqMin;                  // tune: link quality below this is a failsafe
} crsf_t;

void crsfInit(crsf_t *c);

/** Feed one byte from the UART. True when it completed a valid frame. */
bool crsfFeed(crsf_t *c, uint8_t byte, uint32_t nowMs);

/** Evaluate link timeout, arm/e-stop and stick outputs. Call at >= 50 Hz. */
void crsfTick(crsf_t *c, uint32_t nowMs);

/**
 * Status colour for an RGB indicator: ESTOP red blinking, DISARMED red,
 * ARMED yellow / green / blue for MANUAL / STABILIZED / AUTO.
 * `full` is the channel value at full brightness.
 */
void crsfLedColor(const crsf_t *c, uint8_t full, uint32_t nowMs, uint8_t rgb[3]);

/**
 * Operating-mode string for the radio's "Flight mode" field:
 * "ESTOP", "DISARMED", or, while ARMED, the drive mode ("MAN", "STAB", "AUTO").
 */
const char *crsfModeString(const crsf_t *c);

/** Telemetry frames for the receiver (sent on TX1). Return the frame length; `out` needs 64 bytes. */
uint8_t crsfBuildBattery(uint8_t *out, float volts);      // type 0x08; current/capacity/remaining are 0
uint8_t crsfBuildFlightMode(uint8_t *out, const char *s); // type 0x21

#endif
