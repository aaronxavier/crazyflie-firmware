import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
import time
import csv

URI = 'radio://0/80/2M/E7E7E7E7B9'
cflib.crtp.init_drivers()

results = []  # will store (pwm, voltage, timestamp)

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf

    # Set up a log config to stream pm.vbat at 10 Hz
    log_conf = LogConfig(name='Battery', period_in_ms=100)
    log_conf.add_variable('pm.vbat', 'float')

    latest_voltage = [0.0]  # mutable container for callback

    def voltage_callback(timestamp, data, logconf):
        latest_voltage[0] = data['pm.vbat']

    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(voltage_callback)
    log_conf.start()

    # Enable direct motor power override
    cf.param.set_value('motorPowerSet.enable', 1)

    pwm_steps = list(range(0, 62001, 10000))  # 0 to ~95% in steps of ~7.6%

    for pwm in pwm_steps:
        cf.param.set_value('motorPowerSet.m1', pwm)
        cf.param.set_value('motorPowerSet.m2', pwm)
        cf.param.set_value('motorPowerSet.m3', pwm)
        cf.param.set_value('motorPowerSet.m4', pwm)

        time.sleep(3.5)  # let thrust and voltage settle

        v = latest_voltage[0]
        motor_voltage = (pwm / 65535.0) * v
        print(f"PWM: {pwm:5d} | Vbat: {v:.3f} V | Motor V: {motor_voltage:.3f} V | << read scale now >>")
        
        # You manually enter thrust_grams here, or expand this to read a serial scale
        thrust_grams = float(input("  Thrust (grams): "))
        results.append((pwm, v, motor_voltage, thrust_grams))

    cf.param.set_value('motorPowerSet.enable', 0)
    log_conf.stop()

# Save to CSV
with open('thrust_map.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['pwm', 'vbat', 'motor_voltage', 'thrust_grams'])
    writer.writerows(results)

print("\nSaved to thrust_map.csv")
