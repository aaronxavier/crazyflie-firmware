/* Host test for crsf.c: gcc -Wall -Isrc/deck/drivers/interface test/crsf_host_test.c src/deck/drivers/src/crsf.c -lm && ./a.out
 * bit-by-bit packer so the test does not just mirror the parser. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "crsf.h"

static uint8_t crc(const uint8_t *p, int n)
{
  uint8_t c = 0;
  for (int i = 0; i < n; i++) {
    c ^= p[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 0xD5 : c << 1;
  }
  return c;
}

static int rcFrame(uint8_t *f, const uint16_t ch[16])
{
  memset(f, 0, 26);
  f[0] = 0xC8; f[1] = 24; f[2] = 0x16;
  for (int i = 0; i < 16; i++)            // channel i occupies bits [11i, 11i+11)
    for (int b = 0; b < 11; b++)
      if (ch[i] >> b & 1) f[3 + (11 * i + b) / 8] |= 1 << ((11 * i + b) % 8);
  f[25] = crc(&f[2], 23);
  return 26;
}

static uint32_t t;
static uint16_t ch[16];
static void feed(crsf_t *c, const uint8_t *f, int n) { for (int i = 0; i < n; i++) crsfFeed(c, f[i], t); }
static void send(crsf_t *c, int ms)
{
  uint8_t f[26]; int n = rcFrame(f, ch);
  feed(c, f, n); t += ms; crsfTick(c, t);
}
static void stats(crsf_t *c, uint8_t lq)
{
  uint8_t ls[14] = {0xC8, 12, 0x14, 70, 72, lq, 0, 0, 0, 0, 0, 0, 0, 0};
  ls[13] = crc(&ls[2], 11);
  feed(c, ls, 14);
}

#define LO 172
#define MID 992
#define HI 1811
#define ARM 4
#define MODE 5
#define ESTOP 8

#define DISARMED(c) (!(c).estop && !(c).armed)
#define ARMED(c)    (!(c).estop && (c).armed)
#define ESTOPPED(c) ((c).estop && !(c).armed)

static void sticksCentred(void)
{
  for (int i = 0; i < 16; i++) ch[i] = MID;
  ch[ARM] = LO; ch[MODE] = LO; ch[ESTOP] = LO;
}

/* A receiver that just booted and sees a healthy link with arm off: DISARMED. */
static void bringUp(crsf_t *c)
{
  crsfInit(c); sticksCentred();
  send(c, 10);
}

int main(void)
{
  crsf_t c;

  /* ---- parser ---- */
  crsfInit(&c);
  uint16_t pat[16]; for (int i = 0; i < 16; i++) pat[i] = 172 + 97 * i;
  memcpy(ch, pat, sizeof ch); send(&c, 10);
  for (int i = 0; i < 16; i++) assert(c.ch[i] == pat[i]);   // distinct values survive packing
  assert(c.frames == 1 && c.link);

  uint8_t f[26], junk[] = {0x00, 0xC8, 0xFF, 0x12};
  feed(&c, junk, sizeof junk);
  rcFrame(f, ch); f[25] ^= 1; feed(&c, f, 26);              // bad CRC
  assert(c.crcErrors == 1);
  send(&c, 10); assert(c.frames == 2);

  stats(&c, 98); assert(c.rssi == 70 && c.lq == 98);

  /* ---- boot: no link yet is a failsafe -> ESTOP; healthy link + arm off -> DISARMED ---- */
  crsfInit(&c); crsfTick(&c, 10);
  assert(ESTOPPED(c));
  sticksCentred(); send(&c, 10);
  assert(DISARMED(c));

  /* ---- DISARMED -> ARMED -> DISARMED ---- */
  ch[ARM] = HI; send(&c, 10); assert(ARMED(c));
  ch[ARM] = LO; send(&c, 10); assert(DISARMED(c));
  ch[ARM] = HI; send(&c, 10); assert(ARMED(c));

  /* ---- boot with the arm switch already on: never arms until seen off ---- */
  crsfInit(&c); sticksCentred(); ch[ARM] = HI; send(&c, 10);
  assert(ESTOPPED(c));                                      // arm on, so ESTOP does not clear
  ch[ARM] = LO; send(&c, 10); assert(DISARMED(c));
  ch[ARM] = HI; send(&c, 10); assert(ARMED(c));

  /* ---- ARMED -> ESTOP via ch9; ESTOP never goes straight to ARMED ---- */
  bringUp(&c); ch[ARM] = HI; send(&c, 10); assert(ARMED(c));
  ch[ESTOP] = HI; send(&c, 10); assert(ESTOPPED(c));
  ch[ESTOP] = LO; send(&c, 10); assert(ESTOPPED(c));        // released, arm still on: stays
  ch[ARM] = LO; send(&c, 10); assert(DISARMED(c));          // arm off + released: DISARMED
  ch[ARM] = HI; send(&c, 10); assert(ARMED(c));

  /* ESTOP needs ch9 released, even with the arm switch off */
  ch[ESTOP] = HI; send(&c, 10); assert(ESTOPPED(c));
  ch[ARM] = LO; send(&c, 10); assert(ESTOPPED(c));
  ch[ESTOP] = LO; send(&c, 10); assert(DISARMED(c));

  /* ---- DISARMED -> ESTOP via ch9 ---- */
  ch[ESTOP] = HI; send(&c, 10); assert(ESTOPPED(c));

  /* ch8 is not the e-stop any more */
  bringUp(&c); ch[7] = HI; ch[ARM] = HI; send(&c, 10); assert(ARMED(c));

  /* ---- link lost: ARMED -> ESTOP, and again from DISARMED ---- */
  bringUp(&c); ch[ARM] = HI; send(&c, 10); assert(ARMED(c));
  t += CRSF_LINK_TIMEOUT_MS + 1; crsfTick(&c, t);
  assert(ESTOPPED(c) && !c.link);
  send(&c, 10); assert(ESTOPPED(c));                        // link back, arm still on
  ch[ARM] = LO; send(&c, 10); assert(DISARMED(c));
  t += CRSF_LINK_TIMEOUT_MS + 1; crsfTick(&c, t); assert(ESTOPPED(c));

  /* ---- low LQ is a failsafe too (receiver in "set position": frames keep coming) ---- */
  bringUp(&c); stats(&c, 99); ch[ARM] = HI; send(&c, 10); assert(ARMED(c));
  stats(&c, 0); send(&c, 10); assert(ESTOPPED(c) && !c.link);
  ch[ARM] = LO; send(&c, 10); assert(ESTOPPED(c));          // LQ still bad: stays
  stats(&c, 99); send(&c, 10); assert(DISARMED(c));
  stats(&c, 5); send(&c, 10); assert(ESTOPPED(c));          // below the default lqMin of 10

  /* ---- sticks: only when ARMED and not AUTO ---- */
  bringUp(&c); ch[MODE] = MID; ch[1] = HI; ch[3] = LO; send(&c, 10);
  assert(c.fwd == 0 && c.yaw == 0);                         // disarmed
  ch[ARM] = HI; send(&c, 10);
  assert(fabsf(c.fwd - 1.0f) < 1e-3f && c.yaw < -0.99f);
  ch[1] = MID + 10; send(&c, 10); assert(c.fwd == 0.0f);    // inside the deadband
  ch[1] = HI; ch[MODE] = HI; send(&c, 10);
  assert(c.mode == CRSF_MODE_AUTO && c.fwd == 0 && c.yaw == 0);
  ch[MODE] = LO; send(&c, 10); assert(c.mode == CRSF_MODE_MANUAL && c.fwd > 0.99f);
  ch[ESTOP] = HI; send(&c, 10); assert(c.fwd == 0 && c.yaw == 0);

  /* ---- LED colours ---- */
  uint8_t rgb[3];
  bringUp(&c);
  crsfLedColor(&c, 100, 0, rgb); assert(rgb[0] == 100 && !rgb[1] && !rgb[2]);   // DISARMED: solid red
  crsfLedColor(&c, 100, 250, rgb); assert(rgb[0] == 100);
  ch[ARM] = HI; send(&c, 10);
  crsfLedColor(&c, 100, 0, rgb); assert(rgb[0] == 100 && rgb[1] == 60 && !rgb[2]);   // MANUAL: yellow
  ch[MODE] = MID; send(&c, 10);
  crsfLedColor(&c, 100, 0, rgb); assert(!rgb[0] && rgb[1] == 100 && !rgb[2]);        // STABILIZED: green
  ch[MODE] = HI; send(&c, 10);
  crsfLedColor(&c, 100, 0, rgb); assert(!rgb[0] && !rgb[1] && rgb[2] == 100);        // AUTO: blue
  ch[ESTOP] = HI; send(&c, 10); assert(ESTOPPED(c));
  crsfLedColor(&c, 100, 0, rgb);   assert(rgb[0] == 100 && !rgb[1] && !rgb[2]);      // ESTOP: red, blinking
  crsfLedColor(&c, 100, 250, rgb); assert(!rgb[0] && !rgb[1] && !rgb[2]);
  crsfLedColor(&c, 100, 500, rgb); assert(rgb[0] == 100);

  /* ---- mode string ---- */
  assert(!strcmp(crsfModeString(&c), "ESTOP"));
  ch[ESTOP] = LO; ch[ARM] = LO; send(&c, 10); assert(!strcmp(crsfModeString(&c), "DISARMED"));
  ch[ARM] = HI; ch[MODE] = LO; send(&c, 10); assert(!strcmp(crsfModeString(&c), "MAN"));
  ch[MODE] = MID; send(&c, 10); assert(!strcmp(crsfModeString(&c), "STAB"));
  ch[MODE] = HI; send(&c, 10); assert(!strcmp(crsfModeString(&c), "AUTO"));

  /* ---- telemetry frames ---- */
  uint8_t out[64];
  uint8_t n = crsfBuildBattery(out, 11.1f);                 // 111 dV = 0x006F
  assert(n == 12 && out[0] == 0xC8 && out[1] == 10 && out[2] == 0x08);
  assert(out[3] == 0x00 && out[4] == 0x6F && !out[5] && !out[6] && !out[7] && !out[8] && !out[9] && !out[10]);
  assert(out[11] == crc(&out[2], 9));
  n = crsfBuildBattery(out, 16.84f); assert(out[3] == 0x00 && out[4] == 168);           // rounds to 16.8 V
  n = crsfBuildBattery(out, -1.0f);  assert(out[3] == 0 && out[4] == 0);
  n = crsfBuildFlightMode(out, "ESTOP");
  assert(n == 10 && out[1] == 8 && out[2] == 0x21 && !memcmp(&out[3], "ESTOP", 6));
  assert(out[9] == crc(&out[2], 7));

  /* our own telemetry frames must not confuse the parser */
  crsfInit(&c); feed(&c, out, n); assert(c.frames == 0 && c.crcErrors == 0);

  puts("crsf: all tests passed");
  return 0;
}
