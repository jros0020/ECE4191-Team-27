from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
from time import sleep, time
from speed_control import MotorControl
from distance_sensor import DistanceSensorInfo, DistanceSensorCluster 
from threading import Thread

def control_motor_speed(motor_control_object, speed): 
    motor_control_object.enable_motor()
    while True:
        motor_control_object.set_motor_speed(speed) 

# Left motor and encoder
pin_left_motor_PWM1 = 23
pin_left_motor_PWM2 = 24
pin_left_motor_EN = 12
pin_left_motor_encoder_A = 18
pin_left_motor_encoder_B = 25
left_motor_speed_control_kp = 2
left_motor_speed_control_ki = 1
left_motor_control = MotorControl(pin_left_motor_PWM1, pin_left_motor_PWM2, pin_left_motor_EN, pin_left_motor_encoder_A, pin_left_motor_encoder_B, left_motor_speed_control_kp, left_motor_speed_control_ki)

# Right motor and encoder 
pin_right_motor_PWM1 = 4
pin_right_motor_PWM2 = 17
pin_right_motor_EN = 27
pin_right_motor_encoder_A = 5
pin_right_motor_encoder_B = 6
right_motor_speed_control_kp = 2
right_motor_speed_control_ki = 1
right_motor_control = MotorControl(pin_right_motor_PWM1, pin_right_motor_PWM2, pin_right_motor_EN, pin_right_motor_encoder_A, pin_right_motor_encoder_B, right_motor_speed_control_kp, right_motor_speed_control_ki)

# Ultrasonic distance sensors
GLOBAL_TRIGGER = 20
front_dist_sensor = DistanceSensorInfo(name = 'front', ECHO = 21, TRIGGER = GLOBAL_TRIGGER)
right_dist_sensor = DistanceSensorInfo(name = 'right', ECHO = 26, TRIGGER = GLOBAL_TRIGGER)
left_dist_sensor = DistanceSensorInfo(name = 'left', ECHO = 16, TRIGGER = GLOBAL_TRIGGER)
distance_sensor_obj_dict = {'front': front_dist_sensor, 'right': right_dist_sensor, 'left': left_dist_sensor}

dist_sensor_cls = DistanceSensorCluster(distance_sensor_obj_dict, GLOBAL_TRIGGER) 


while True: 
    p1 = Thread(target = control_motor_speed, args = (right_motor_control, -0.5,))
    p2 = Thread(target = control_motor_speed, args = (left_motor_control, 0.5,))

