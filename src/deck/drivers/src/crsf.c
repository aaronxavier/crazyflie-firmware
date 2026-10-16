/**
 * crsf.c - see crsf.h.
 *
 * Frame: [addr 0xC8][len][type][payload...][crc8], len counts type..crc.
 * crc8 is poly 0xD5 over type+payload.
 */
#include <string.h>

#include "crsf.h"

#define ADDR_FC          0xC8
#define TYPE_LINK_STATS  0x14   // 10-byte payload
#define TYPE_RC_CHANNELS 0x16   // 22-byte payload: 16 x 11 bit, LSB first
#define TYPE_BATTERY     0x08   // telemetry: u16 dV, u16 dA, u24 mAh, u8 %  (big endian)
#define TYPE_FLIGHT_MODE 0x21   // telemetry: NUL-terminated string

#define CH_STRAFE 0
#define CH_FWD    1
#define CH_YAW    3
#define CH_ARM    4
#define CH_MODE   5
#define CH_ESTOP  8

#define CRSF_CENTRE 992
#define CRSF_HALF   819.5f
// ponytail: fixed switch thresholds (~1250/1750 us), no per-switch calibration;
// the ELRS switch outputs sit at 172/992/1811 so there is a lot of margin.
#define SW_LOW   600
#define SW_HIGH  1400

static uint8_t crc8(const uint8_t *p, uint8_t n)
{
  uint8_t crc = 0;
  while (n--) {
    crc ^= *p++;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void crsfInit(crsf_t *c)
{
  memset(c, 0, sizeof(*c));
  c->mode = CRSF_MODE_MANUAL;
  c->deadband = 0.03f;
  c->lqMin = 10;
  c->estop = 1;           // no link yet is a failsafe; the first tick would set it anyway
}

static void unpackChannels(crsf_t *c, const uint8_t *p)
{
  uint32_t acc = 0;
  int bits = 0, n = 0;
  for (int i = 0; i < 22; i++) {
    acc |= (uint32_t)p[i] << bits;
    bits += 8;
    while (bits >= 11 && n < CRSF_NUM_CHANNELS) {
      c->ch[n++] = acc & 0x7FF;
      acc >>= 11;
      bits -= 11;
    }
  }
}

bool crsfFeed(crsf_t *c, uint8_t b, uint32_t nowMs)
{
  if (c->idx == 0) {
    if (b == ADDR_FC) {
      c->buf[c->idx++] = b;
    }
    return false;
  }
  if (c->idx == 1 && (b < 2 || b > 62)) {  // implausible length: resync
    c->idx = 0;
    return false;
  }

  c->buf[c->idx++] = b;
  uint8_t len = c->buf[1];
  if (c->idx < len + 2) {
    return false;
  }
  c->idx = 0;

  // ponytail: on a CRC error the whole frame is dropped, even if a real sync
  // byte was inside it. Fine at >= 50 Hz frame rates; resync-in-place if not.
  if (crc8(&c->buf[2], len - 1) != c->buf[len + 1]) {
    c->crcErrors++;
    return false;
  }

  uint8_t type = c->buf[2];
  const uint8_t *payload = &c->buf[3];
  if (type == TYPE_RC_CHANNELS && len == 24) {
    unpackChannels(c, payload);
    c->frames++;
    c->haveRc = true;
    c->lastRcMs = nowMs;
  } else if (type == TYPE_LINK_STATS && len == 12) {
    c->rssi = payload[0];
    c->lq = payload[2];
    c->haveStats = true;
  }
  return true;
}

static float stick(const crsf_t *c, uint16_t raw)
{
  float x = ((float)raw - CRSF_CENTRE) / CRSF_HALF;
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  float a = x < 0 ? -x : x;
  if (a < c->deadband) return 0.0f;
  a = (a - c->deadband) / (1.0f - c->deadband);  // rescale: no jump at the edge
  return x < 0 ? -a : a;
}

void crsfTick(crsf_t *c, uint32_t nowMs)
{
  uint32_t age = c->haveRc ? nowMs - c->lastRcMs : 0xFFFF;
  c->ageMs = age > 0xFFFF ? 0xFFFF : (uint16_t)age;

  // Failsafe = stale RC frames, or low link quality. The LQ check matters when
  // the receiver's failsafe is "set position": it keeps streaming RC frames,
  // but its link stats (a few per second) still report the loss.
  bool lowLq = c->haveStats && c->lq < c->lqMin;
  c->link = c->haveRc && age < CRSF_LINK_TIMEOUT_MS && !lowLq;

  bool armSw = c->link && c->ch[CH_ARM] > SW_HIGH;
  bool armOff = c->link && !armSw;
  bool estopSw = c->link && c->ch[CH_ESTOP] > SW_HIGH;

  if (!c->link || estopSw) {          // from any state (also covers boot: no link yet)
    c->estop = 1;
    c->armed = 0;
    c->armSeenOff = 0;
  } else if (c->estop) {              // leave only with arm off, no failsafe, e-stop released
    if (armOff) {
      c->estop = 0;
      c->armSeenOff = 1;
    }
  } else if (!armSw) {                // DISARMED / ARMED -> DISARMED
    c->armed = 0;
    c->armSeenOff = 1;
  } else if (c->armSeenOff) {         // DISARMED -> ARMED
    c->armed = 1;
    c->armSeenOff = 0;
  }

  c->fwd = c->strafe = c->yaw = 0.0f;
  if (c->link) {
    uint16_t m = c->ch[CH_MODE];
    c->mode = m < SW_LOW ? CRSF_MODE_MANUAL : m > SW_HIGH ? CRSF_MODE_AUTO : CRSF_MODE_STABILIZED;
  }
  if (c->armed && c->mode != CRSF_MODE_AUTO) {  // AUTO has no source yet: zero
    c->fwd = stick(c, c->ch[CH_FWD]);
    c->strafe = stick(c, c->ch[CH_STRAFE]);
    c->yaw = stick(c, c->ch[CH_YAW]);
  }
}

#define BLINK_HALF_PERIOD_MS 250

void crsfLedColor(const crsf_t *c, uint8_t full, uint32_t nowMs, uint8_t rgb[3])
{
  uint8_t r = 0, g = 0, b = 0;
  if (c->estop) {                             // red blinking
    if ((nowMs / BLINK_HALF_PERIOD_MS) % 2 == 0) r = full;
  } else if (!c->armed) {                     // DISARMED: red
    r = full;
  } else if (c->mode == CRSF_MODE_MANUAL) {   // yellow
    r = full;
    g = (uint16_t)full * 3 / 5;               // tune: green die is stronger, 3/5 reads as yellow not lime
  } else if (c->mode == CRSF_MODE_STABILIZED) {
    g = full;
  } else {
    b = full;                                 // AUTO
  }
  rgb[0] = r; rgb[1] = g; rgb[2] = b;
}

const char *crsfModeString(const crsf_t *c)
{
  if (c->estop) return "ESTOP";
  if (!c->armed) return "DISARMED";
  return c->mode == CRSF_MODE_MANUAL ? "MAN" : c->mode == CRSF_MODE_STABILIZED ? "STAB" : "AUTO";
}

// Telemetry frame: [0xC8][len][type][payload][crc8], len counts type..crc.
static uint8_t finishFrame(uint8_t *out, uint8_t type, const uint8_t *payload, uint8_t n)
{
  out[0] = ADDR_FC;
  out[1] = n + 2;
  out[2] = type;
  memcpy(&out[3], payload, n);
  out[3 + n] = crc8(&out[2], n + 1);
  return n + 4;
}

uint8_t crsfBuildBattery(uint8_t *out, float volts)
{
  uint16_t dv = volts <= 0 ? 0 : volts >= 6553.5f ? 0xFFFF : (uint16_t)(volts * 10.0f + 0.5f);  // 0.1 V, big endian
  uint8_t p[8] = {(uint8_t)(dv >> 8), (uint8_t)dv, 0, 0, 0, 0, 0, 0};
  return finishFrame(out, TYPE_BATTERY, p, sizeof(p));
}

uint8_t crsfBuildFlightMode(uint8_t *out, const char *str)
{
  uint8_t p[32];
  uint8_t n = 0;
  while (str[n] && n < sizeof(p) - 1) {
    p[n] = (uint8_t)str[n];
    n++;
  }
  p[n++] = 0;                                 // NUL-terminated
  return finishFrame(out, TYPE_FLIGHT_MODE, p, n);
}
