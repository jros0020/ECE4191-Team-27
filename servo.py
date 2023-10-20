from gpiozero import Servo, PWMOutputDevice
from time import sleep

servo = Servo(19)
# servo_PWM = PWMOutputDevice(pin = 6, frequency = 100)

# servo.value = -1
# servo.min()
# sleep(5)

# servo.value = 0
# sleep(1)
# test = PWMOutputDevice(pin = 21, initial_value = 0.0, frequency = 1000)
# while True:
#     test.value = 0.25

# def nearest_90_degree_straight_line_angle(given_angle):
#       # Calculate the remainder when dividing the given angle by 90 degrees
#       remainder = given_angle % 90
      
#       # Determine the nearest 90-degree straight-line angle
#       if remainder > 10:
#           nearest_angle = given_angle + (90 - remainder)
#       else:
#           nearest_angle = given_angle
#       print(nearest_angle)
      
#       return nearest_angle

# x = nearest_90_degree_straight_line_angle(630)
# print(x)

while True:
    servo.max()
    sleep(0.5)
    # servo.value = 0
    # sleep(5)
    servo.min()
    sleep(0.62)
# servo.value = 0
# sleep(1)

# while True:
    # servo.min()
    # sleep(1)
    # servo.mid()
    # sleep(1)
    # servo.max()
    # sleep(1)
    # servo.value = 0
    # sleep(1)
    # servo.value = -0.5
    # sleep(1)
    # servo.value = 0.1
    # sleep(1)
    