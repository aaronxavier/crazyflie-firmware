// Standalone check for the external-pose-loss landing transitions.
// The rake/unity harness is currently broken (unrelated motors.h/stm32 include issue), so this is
// built directly:
//   gcc -I src/modules/interface -I src/config -I src/utils/interface \
//       test/modules/src/test_ext_pose_landing.c src/modules/src/supervisor_state_machine.c \
//       -o /tmp/t && /tmp/t
#include <assert.h>
#include <stdio.h>
#include "supervisor_state_machine.h"

#define FLYING (SUPERVISOR_CB_ARMED | SUPERVISOR_CB_IS_FLYING)

int main(void) {
  // Stale ext pose while flying -> land, and it wins over the commander watchdog warning
  assert(supervisorStateUpdate(supervisorStateFlying, FLYING | SUPERVISOR_CB_EXT_POSE_TIMEOUT)
         == supervisorStateExtPoseLanding);
  assert(supervisorStateUpdate(supervisorStateFlying,
                               FLYING | SUPERVISOR_CB_EXT_POSE_TIMEOUT | SUPERVISOR_CB_COMMANDER_WDT_WARNING)
         == supervisorStateExtPoseLanding);

  // ...but a tumble or emergency stop still wins over the landing
  assert(supervisorStateUpdate(supervisorStateFlying, FLYING | SUPERVISOR_CB_EXT_POSE_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED)
         == supervisorStateCrashed);
  assert(supervisorStateUpdate(supervisorStateFlying, FLYING | SUPERVISOR_CB_EXT_POSE_TIMEOUT | SUPERVISOR_CB_EMERGENCY_STOP)
         == supervisorStateExceptFreeFall);

  // Descending: stays put while the pose is still stale
  assert(supervisorStateUpdate(supervisorStateExtPoseLanding, FLYING | SUPERVISOR_CB_EXT_POSE_TIMEOUT)
         == supervisorStateExtPoseLanding);

  // Descent overran -> cut motors
  assert(supervisorStateUpdate(supervisorStateExtPoseLanding,
                               FLYING | SUPERVISOR_CB_EXT_POSE_TIMEOUT | SUPERVISOR_CB_EXT_POSE_LANDING_TIMEOUT)
         == supervisorStateExceptFreeFall);

  // Touched down
  assert(supervisorStateUpdate(supervisorStateExtPoseLanding,
                               SUPERVISOR_CB_ARMED | SUPERVISOR_CB_EXT_POSE_TIMEOUT)
         == supervisorStateLanded);

  // Mocap recovered -> back to flying
  assert(supervisorStateUpdate(supervisorStateExtPoseLanding, FLYING) == supervisorStateFlying);

  // Disarmed mid-descent
  assert(supervisorStateUpdate(supervisorStateExtPoseLanding, SUPERVISOR_CB_IS_FLYING | SUPERVISOR_CB_EXT_POSE_TIMEOUT)
         == supervisorStateReset);

  // Regression: no ext pose condition -> unchanged behaviour
  assert(supervisorStateUpdate(supervisorStateFlying, FLYING) == supervisorStateFlying);
  assert(supervisorStateUpdate(supervisorStateFlying, FLYING | SUPERVISOR_CB_COMMANDER_WDT_WARNING)
         == supervisorStateWarningLevelOut);

  printf("ext pose landing transitions OK\n");
  return 0;
}
