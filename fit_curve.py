import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('thrust_map.csv')

thrust = df['thrust_grams'].values
motor_v = df['motor_voltage'].values  # = (pwm/65535) * vbat

# Fit volts = f(thrust), quadratic — matching what the firmware expects
coeffs = np.polyfit(thrust, motor_v, 2)
p0, p1, p2_constant = coeffs

print(f"Raw fit:  p0={p0:.8f}  p1={p1:.8f}  p2(dropped)={p2_constant:.8f}")

# Drop the constant term (firmware forces zero thrust = zero voltage)
# The two remaining coefficients go directly into the firmware
print(f"\n// Paste these into motors.c:")
print(f"#define MOTOR_POLY_A  ({p0:.8f}f)  // quadratic term")
print(f"#define MOTOR_POLY_B  ({p1:.8f}f)  // linear term")

# Verify fit (with and without constant, so you can see the error introduced)
thrust_range = np.linspace(0, thrust.max(), 300)
v_with_const    = np.polyval([p0, p1, p2_constant], thrust_range)
v_without_const = np.polyval([p0, p1, 0.0],         thrust_range)

plt.scatter(thrust, motor_v, label='Measured', zorder=5)
plt.plot(thrust_range, v_with_const,    label='Quadratic fit (with const)', linestyle='--')
plt.plot(thrust_range, v_without_const, label='Quadratic fit (const=0)',    linestyle='-')
plt.xlabel('Thrust (g)')
plt.ylabel('Motor voltage (V)')
plt.legend()
plt.title('Thrust Map — Firmware Convention')
plt.savefig('thrust_map.png')
plt.show()

# Sanity check: reconstruct PWM% from a nominal vbat and compare to measured
vbat_nominal = df['vbat'].mean()
print(f"\nSanity check at Vbat = {vbat_nominal:.3f}V:")
print(f"{'Thrust(g)':>10} {'Measured PWM%':>14} {'Predicted PWM%':>15}")
for _, row in df.iterrows():
    v_needed   = p0 * row['thrust_grams']**2 + p1 * row['thrust_grams']
    pwm_pred   = (v_needed / row['vbat']) * 100
    pwm_meas   = (row['pwm'] / 65535.0) * 100
    print(f"{row['thrust_grams']:>10.1f} {pwm_meas:>14.2f} {pwm_pred:>15.2f}")