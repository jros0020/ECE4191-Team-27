from gpiozero import DistanceSensor, PWMOutputDevice
from datetime import datetime
import time

sensor = DistanceSensor(max_distance = 4, echo = 20, trigger = 21)
driver1_pwm1 = PWMOutputDevice(pin = 18, initial_value = 0, frequency = 10000)
driver1_pwm2 = PWMOutputDevice(pin = 23, initial_value = 0, frequency = 10000)
driver2_pwm1 = PWMOutputDevice(pin = 17, initial_value = 0, frequency = 1000)
driver2_pwm2 = PWMOutputDevice(pin = 27, initial_value = 0, frequency = 1000)

# Kp = 0.1
# Kd = -0.01
# goal = 5
# curr_error = 1e6
# prev_error = 0
# prev_time = time.time()
# distance = sensor.distance * 100

# def control_sat(control_signal):
#     if control_signal > 1:
#         control_signal = 1
#         return control_signal
#     elif control_signal < 0:
#         control_signal = 0
#         return control_signal
#     else: 
#         control_signal = control_signal
#         return control_signal
    
# while curr_error > 1:
#     distance = sensor.distance * 100
#     print('Distance: ', distance)
#     curr_error = distance - goal
#     derror = curr_error - prev_error
#     # print(derror)
#     dt = time.time() - prev_time
#     # print(dt)
#     prev_time = time.time()
#     control_signal = Kp * curr_error + Kd * (derror/dt)
#     control_signal = control_sat(control_signal)
#     print('Control Signal: ', control_signal)
#     driver1_pwm1.value = control_signal
#     driver2_pwm1.value = control_signal
#     prev_error = curr_error
#     time.sleep(0.025)

# driver1_pwm1.value = 0
# driver2_pwm1.value = 0
# print('You have arrived at your destination.')
while True:
    driver2_pwm1.value = 0
    driver2_pwm2.value = 0.9
    print("Running")