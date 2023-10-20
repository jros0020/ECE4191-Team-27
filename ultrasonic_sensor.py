from gpiozero import DistanceSensor, PWMOutputDevice
from datetime import datetime
import time
import matplotlib.pyplot as plt
import numpy as np





# left_sensor = DistanceSensor(max_distance = 4, echo = 23, trigger = 18)
left_center_sensor = DistanceSensor(max_distance = 2, echo = 16, trigger = 24)
# right_center_sensor = DistanceSensor(max_distance = 4, echo = 14, trigger = 12)
# right_sensor = DistanceSensor(max_distance = 4, echo = 15, trigger = 20)

# while True:
#     print(left_center_sensor.distance * 100)
    

sensor_readings = []
sensor_readings_2 = []
init_time = time.time()
time_elapsed = 0
weight = 0.8
prev_dist = left_center_sensor.distance * 100 + 1
while time_elapsed < 10:
    # print(left_center_sensor.distance * 100)
    # new_dist = (1-weight)*round(left_center_sensor.distance * 100 + 1, 10) + weight*prev_dist
    # prev_dist = new_dist
    sensor_readings.append(left_center_sensor.distance * 100 - 3.492261644918596)
    print(left_center_sensor.distance * 100)
    # sensor_readings_2.append(right_center_sensor.distance * 100)
    time_elapsed = time.time() - init_time
    # print(new_dist)
    time.sleep(0.06)
    # driver2_pwm1.value = 0
    # driver2_pwm2.value = 0.9
    # print("Running")

offset = 20 - np.mean(sensor_readings)
offset2 = 20 - np.mean(sensor_readings_2)

print(offset)
print(offset2)

plot_time = range(0, len(sensor_readings))
plt.plot(plot_time, sensor_readings)
plt.axhline(y = 20)
plt.savefig("ece4191/ECE4191-Team-27/pics/sensor_readings_variable_1.jpg")