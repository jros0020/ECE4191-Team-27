from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
from time import sleep, time
    
class PIController:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, setpoint, feedback):
        error = setpoint - feedback
        self.integral += error
        control_effort = self.kp * error + self.ki * self.integral

        if control_effort > 1:
            return 1
        elif control_effort < 0: 
            return 0
        
        return control_effort

class Encoder: 
    pulses_per_wheel_rotation = 75*12 # 75 turns of encoder is 1 rotation of wheel, 12 pulses per encoder rotation

    def __init__(self, pin_encoder_A, pin_encoder_B): 
        self.encoder = RotaryEncoder(a = pin_encoder_A, b = pin_encoder_B, max_steps = 0) 
        self.prev_time = time()
        self.prev_number_of_steps = 0

    ## NOTE: need to add delay between consecutive readings
    def get_motor_speed(self): 
        current_number_of_steps = self.encoder.steps
        current_time = time()
        steps_per_second = (current_number_of_steps-self.prev_number_of_steps)/(current_time - self.prev_time)
        motor_speed = steps_per_second/Encoder.pulses_per_wheel_rotation

        self.prev_number_of_steps = current_number_of_steps
        self.prev_time = current_time

        return motor_speed

class MotorControl: 
    def __init__(self, pin_PWM1, pin_PWM2, pin_EN, pin_encoder_A, pin_encoder_B, speed_control_kp, speed_control_ki): 
        self.motor_PWM1 = PWMOutputDevice(pin = pin_PWM1, initial_value = 0.0, frequency = 1000)
        self.motor_PWM2 = PWMOutputDevice(pin = pin_PWM2, initial_value = 0, frequency = 1000)
        self.motor_EN = DigitalOutputDevice(pin = pin_EN)

        self.encoder = Encoder(pin_encoder_A, pin_encoder_B)

        self.pi_controller = PIController(speed_control_kp, speed_control_ki)

    def enable_motor(self): 
        self.motor_EN.on()
        
    def disable_motor(self): 
        self.motor_EN.off()

    def set_motor_speed(self, ref_motor_speed):
        max_iter = 35

        for idx in range(max_iter):
            current_motor_speed = self.encoder.get_motor_speed()

            if ref_motor_speed > 0: 
                self.motor_PWM1.value = self.pi_controller.update(ref_motor_speed, current_motor_speed)
                self.motor_PWM2.value = 0
            elif ref_motor_speed < 0: 
                self.motor_PWM1.value = 0
                self.motor_PWM2.value = self.pi_controller.update(-ref_motor_speed, -current_motor_speed)
            else: 
                self.motor_PWM1.value = 0
                self.motor_PWM2.value = 0

            print(f'Current Motor speed: {current_motor_speed} rev/s, Set Motor speed: {ref_motor_speed} rev/s')
            sleep(0.07) 

    def get_current_motor_speed(self): 
        return self.encoder.get_motor_speed()


left_pin_PWM1 = 23
left_pin_PWM2 = 24
left_pin_EN = 12
left_pin_encoder_A = 18
left_pin_encoder_B = 25
left_speed_control_kp = 2
left_speed_control_ki = 1
left_motor_control = MotorControl(left_pin_PWM1, left_pin_PWM2, left_pin_EN, left_pin_encoder_A, left_pin_encoder_B, left_speed_control_kp, left_speed_control_ki)

left_motor_control.enable_motor()
left_motor_control.set_motor_speed(0.3)
left_motor_control.set_motor_speed(0.1)
left_motor_control.set_motor_speed(0.4)
left_motor_control.set_motor_speed(-0.2)
