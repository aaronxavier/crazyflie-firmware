/**
 * elrs_deck.c - ExpressLRS receiver deck ("rcElrs"), a self-contained deck driver.
 *
 * ELRS receiver -> CRSF @ 420 kbaud -> UART1 (RX1 = PC11, TX1 = PC10 on the
 * deck connector). Owns the whole receive side: parser, link/arm/e-stop state
 * machine (crsf.c), telemetry back to the radio, and the "crsf" log + param
 * groups. Flash this on any Bolt with the deck fitted and read the signals
 * with a Crazyflie client; nothing here needs the robot app.
 *
 * Arming: the transmitter's ARMED state drives supervisorRequestArming() (param
 * crsf.armCtl, default on). Arm is retried while the switch is held until the
 * supervisor grants it; disarm (switch down, e-stop, link loss) is immediate.
 * Only edges act, so arming from cfclient still works while the switch is down.
 *
 * The deck has no 1-wire memory, so it is force-loaded: CONFIG_DECK_FORCE="rcElrs".
 */
#include <stdint.h>

#include "deck.h"
#include "uart1.h"
#include "param.h"
#include "log.h"
#include "pm.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "ELRS"
#include "debug.h"

#include "crsf.h"
#include "supervisor.h"

#define TICK_MS 10
#define TELEMETRY_PERIOD_MS 250   // battery and flight mode alternate: 2 Hz each
// Priority 3: the UART1 driver only buffers 64 bytes (1.5 ms at 420 kbaud),
// so this task has to preempt the low-priority ones.
#define ELRS_TASK_PRI 3

static crsf_t rx;
static bool isInit;
static uint8_t armCtl = 1;

static uint32_t nowMs(void)
{
  return T2M(xTaskGetTickCount());
}

static void elrsTask(void *param)
{
  uint32_t lastTick = 0, lastTelemetry = 0;
  bool sendBattery = true;
  uint8_t wasLink = 0, wasArmed = 0, wasEstop = 0;
  bool wantArm = false;

  while (1) {
    uint8_t b;
    if (uart1GetDataWithTimeout(&b, M2T(TICK_MS))) {
      crsfFeed(&rx, b, nowMs());
    }

    // Tick on a timer, not per frame: with a dead receiver no bytes arrive at
    // all, and the link timeout still has to fire.
    uint32_t now = nowMs();
    if (now - lastTick >= TICK_MS) {
      lastTick = now;
      crsfTick(&rx, now);

      if (rx.link != wasLink)   DEBUG_PRINT("link %s\n", rx.link ? "up" : "LOST");
      if (rx.armed != wasArmed) DEBUG_PRINT("%s\n", rx.armed ? "ARMED" : "disarmed");
      if (rx.estop != wasEstop) DEBUG_PRINT("e-stop %s\n", rx.estop ? "LATCHED" : "cleared");

      if (armCtl) {
        if (rx.armed && !wasArmed) {
          wantArm = true;
        } else if (!rx.armed) {
          if (wasArmed) {
            supervisorRequestArming(false);
          }
          wantArm = false;
        }
        if (wantArm && supervisorRequestArming(true)) {
          wantArm = false;
        }
      }

      wasLink = rx.link;
      wasArmed = rx.armed;
      wasEstop = rx.estop;
    }

    // Telemetry back to the radio (RX1/TX1 wire to the receiver's RX pad).
    // ponytail: ELRS limits the telemetry rate (ratio 1:2..1:128); frames beyond
    // that are dropped by the receiver, so one frame per period is plenty.
    if (now - lastTelemetry >= TELEMETRY_PERIOD_MS) {
      lastTelemetry = now;
      uint8_t frame[64];
      uint8_t n = sendBattery ? crsfBuildBattery(frame, pmGetBatteryVoltage())
                              : crsfBuildFlightMode(frame, crsfModeString(&rx));
      sendBattery = !sendBattery;
      uart1SendData(n, frame);
    }
  }
}

static void elrsInit(DeckInfo *info)
{
  if (isInit) {
    return;
  }
  crsfInit(&rx);
  uart1Init(CRSF_BAUD);
  xTaskCreate(elrsTask, "ELRS", 400, NULL, ELRS_TASK_PRI, NULL);
  DEBUG_PRINT("CRSF on UART1 @ %d baud\n", CRSF_BAUD);
  isInit = true;
}

static bool elrsTest(void)
{
  return isInit;
}

static const DeckDriver elrs_deck = {
  .vid = 0,
  .pid = 0,
  .name = "rcElrs",

  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,

  .init = elrsInit,
  .test = elrsTest,
};

DECK_DRIVER(elrs_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, rcElrs, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(crsf)
PARAM_ADD(PARAM_FLOAT, deadband, &rx.deadband)  // stick deadband, 0..1 of travel
PARAM_ADD(PARAM_UINT8, lqMin, &rx.lqMin)        // link quality below this = failsafe (ESTOP)
PARAM_ADD(PARAM_UINT8, armCtl, &armCtl)         // 1: transmitter arm switch arms/disarms the supervisor
PARAM_GROUP_STOP(crsf)

// The signals. Raw channels are CRSF ticks (172..1811, centre 992); fwd/strafe/yaw
// are the deadbanded -1..1 sticks, zero unless ARMED and not AUTO.
LOG_GROUP_START(crsf)
LOG_ADD(LOG_UINT8, link, &rx.link)
LOG_ADD(LOG_UINT8, armed, &rx.armed)
LOG_ADD(LOG_UINT8, estop, &rx.estop)
LOG_ADD(LOG_UINT8, mode, &rx.mode)          // 0 manual, 1 stabilized, 2 auto
LOG_ADD(LOG_UINT8, lq, &rx.lq)
LOG_ADD(LOG_UINT8, rssi, &rx.rssi)          // -dBm
LOG_ADD(LOG_UINT16, age, &rx.ageMs)
LOG_ADD(LOG_FLOAT, fwd, &rx.fwd)
LOG_ADD(LOG_FLOAT, strafe, &rx.strafe)
LOG_ADD(LOG_FLOAT, yaw, &rx.yaw)
LOG_ADD(LOG_UINT16, ch1, &rx.ch[0])
LOG_ADD(LOG_UINT16, ch2, &rx.ch[1])
LOG_ADD(LOG_UINT16, ch3, &rx.ch[2])
LOG_ADD(LOG_UINT16, ch4, &rx.ch[3])
LOG_ADD(LOG_UINT16, ch5, &rx.ch[4])
LOG_ADD(LOG_UINT16, ch6, &rx.ch[5])
LOG_ADD(LOG_UINT16, ch7, &rx.ch[6])
LOG_ADD(LOG_UINT16, ch8, &rx.ch[7])
LOG_ADD(LOG_UINT16, ch9, &rx.ch[8])
LOG_ADD(LOG_UINT32, frames, &rx.frames)
LOG_ADD(LOG_UINT32, crcErr, &rx.crcErrors)
LOG_GROUP_STOP(crsf)
