# Vehicle mass in kg — set to your actual all-up weight
CFLAGS += -DCF_MASS=0.1150f

# Enable the battery voltage compensation (without this, motors.c changes do nothing)
CFLAGS += -DCONFIG_ENABLE_THRUST_BAT_COMPENSATED

# Safety: require explicit arming before motors spin
CFLAGS += -DSTART_DISARMED

# Idle thrust when armed (prevents motors stopping mid-air)
CFLAGS += -DDEFAULT_IDLE_THRUST=5000

# Motor protocol — pick one
# CFLAGS += -DENABLE_DSHOT        # preferred for Bolt 1.1
CFLAGS += -DENABLE_ONESHOT125  # alternative