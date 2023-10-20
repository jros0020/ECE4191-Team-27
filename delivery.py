from gpiozero import servo 
from time import sleep

class Servo():
    def __init__(self, sensor_pin):
        self.servo = Servo(sensor_pin) ##change to whatever pin

    def deliver():
        servo_val = -1
        self.servo.val = servo_val ##set servo to minimum value

        #extend arm
        while servo_val < 1: ## increment slowly up to max value (1)
            servo_val += 0.1
            self.servo.val = servo_val
            sleep(1) #might need need sleep, see how we go

        sleep(15)

        #retract arm
        for i in range(20):
            servo_val -= 0.1
            self.servo.val = servo_val
            sleep(1)

        self.servo.val = None #if none servo should stop being controlled

        print('Delivery completed')
