from gpiozero import DistanceSensor, PWMOutputDevice
import datetime
import time

sensor = DistanceSensor(max_distance = 4, echo=17, trigger=18)
Kp = 1
Kd = 2
goal = 5
prev_error = 0
prev_time = 0
distance = sensor.distance * 100

# def control(distance, prev_error, prev_time, goal = 5):
#     curr_error = distance - goal
#     derror = curr_error - prev_error
#     dt = datetime.datetime.now().time() - prev_time
#     control_signal = Kp * curr_error + Kd * (derror/dt)
#     return control_signal

while distance < 5:
    distance = sensor.distance * 100
    # print('Distance: ', distance)
    curr_error = distance - goal
    derror = curr_error - prev_error
    dt = datetime.datetime.now().time() - prev_time
    prev_time = datetime.datetime.now().time()
    control_signal = Kp * curr_error + Kd * (derror/dt)
    motor_driver = PWMOutputDevice(10, 1, 0, control_signal)
    prev_error = curr_error
    time.sleep(0.025)
    
print('You have arrived at your destination.')