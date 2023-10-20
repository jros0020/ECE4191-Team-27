from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
from time import sleep, time
import matplotlib.pyplot as plt


class PIController:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral = []
        self.prev_error = 0.0
        self.prev_time = 0
        self.control_effort = 0 
    
    def update(self, setpoint, feedback, time):
        error = setpoint - feedback
        
        if len(self.integral) > 500:
            self.integral.pop(0)
                
        self.integral.append(error)
        dt = time - self.prev_time
        self.control_effort = self.kp * error + self.ki * sum(self.integral)*dt

        self.prev_time = time
        
        if self.control_effort > 1:
            return 1
        elif self.control_effort < 0: 
            return 0
        
        return self.control_effort
        

class Encoder: 
    pulses_per_wheel_rotation = 75*12 # 75 turns of encoder is 1 rotation of wheel, 12 pulses per encoder rotation

    def __init__(self, pin_encoder_A, pin_encoder_B, motor_name, encoder_offset): 
        self.encoder = RotaryEncoder(a = pin_encoder_A, b = pin_encoder_B, max_steps = 0) 
        self.prev_time = time()
        self.prev_number_of_steps = 0
        # self.encoder_pub = rospy.Publisher(motor_name, Float32, queue_size=1)
        self.prev_speed = 0
        self.weight = 0
        self.name = motor_name
        self.offset = encoder_offset
        self.current_number_of_steps = 0
        self.motor_speed = 0

    def get_motor_speed(self):
        self.encoder.steps += self.offset
        self.current_number_of_steps = self.encoder.steps
        current_time = time()
        # print(self.name, " total: ", self.current_number_of_steps)
        steps_per_second = (self.current_number_of_steps-self.prev_number_of_steps)/(current_time - self.prev_time)
        self.motor_speed = steps_per_second/Encoder.pulses_per_wheel_rotation
        # print(self.name, " : ", self.current_number_of_steps)
        # print(self.name, " change: ", self.current_number_of_steps-self.prev_number_of_steps + self.offset)
        print(self.name, ": ", self.motor_speed)
        self.prev_number_of_steps = self.current_number_of_steps
        self.prev_time = current_time
        return self.motor_speed

class MotorControl: 
    def __init__(self, pin_PWM1, pin_PWM2, pin_EN, pin_encoder_A, pin_encoder_B, speed_control_kp, speed_control_ki, encoder_name, encoder_offset): 
        self.motor_PWM1 = PWMOutputDevice(pin = pin_PWM1, initial_value = 0.0, frequency = 1000)
        self.motor_PWM2 = PWMOutputDevice(pin = pin_PWM2, initial_value = 0, frequency = 1000)
        self.motor_EN = DigitalOutputDevice(pin = pin_EN)

        self.encoder = Encoder(pin_encoder_A, pin_encoder_B, encoder_name, encoder_offset)

        self.pi_controller = PIController(speed_control_kp, speed_control_ki)

    def enable_motor(self): 
        self.motor_EN.on()
        
    def disable_motor(self): 
        self.motor_EN.off()

    def set_motor_speed(self, ref_motor_speed):
        # max_iter = 1

        # for idx in range(max_iter):
        # current_motor_speed = self.encoder.get_motor_speed()
        
        # self.encoder.get_motor_speed()

        if ref_motor_speed > 0: 
            self.motor_PWM1.value = ref_motor_speed
            # self.motor_PWM1.value = self.pi_controller.update(ref_motor_speed, current_motor_speed, time())
            self.motor_PWM2.value = 0
        elif ref_motor_speed < 0: 
            self.motor_PWM1.value = 0
            self.motor_PWM2.value = -ref_motor_speed
            # self.motor_PWM2.value = self.pi_controller.update(-ref_motor_speed, -current_motor_speed, time())
        else: 
            self.motor_PWM1.value = 0
            self.motor_PWM2.value = 0

        # print(f'Current Motor speed: {current_motor_speed} rev/s, Set Motor speed: {ref_motor_speed} rev/s')
        # sleep(0.05) 

    def get_current_motor_speed(self): 
        return self.encoder.get_motor_speed()
    

pin_left_motor_PWM1 = 27
pin_left_motor_PWM2 = 17
pin_left_motor_EN = 22
pin_left_motor_encoder_A = 9
pin_left_motor_encoder_B = 10
left_motor_speed_control_kp = 2
left_motor_speed_control_ki = 2.5
left_encoder_name = "left encoder"
left_offset = 0
left_motor_control = MotorControl(pin_left_motor_PWM1, pin_left_motor_PWM2, pin_left_motor_EN, pin_left_motor_encoder_A, pin_left_motor_encoder_B, left_motor_speed_control_kp, left_motor_speed_control_ki, left_encoder_name, left_offset)

# left_motor_control.enable_motor()

pin_right_motor_PWM1 = 8
pin_right_motor_PWM2 = 21
pin_right_motor_EN = 7
pin_right_motor_encoder_A = 26
pin_right_motor_encoder_B = 25
right_motor_speed_control_kp = 2
right_motor_speed_control_ki = 2.5
right_encoder_name = "right encoder"
right_offset = 0
right_motor_control = MotorControl(pin_right_motor_PWM1, pin_right_motor_PWM2, pin_right_motor_EN, pin_right_motor_encoder_A, pin_right_motor_encoder_B, right_motor_speed_control_kp, right_motor_speed_control_ki, right_encoder_name, right_offset)


# right_motor_control.enable_motor()

# encoder = Encoder(26,19)

# PWM_test = PWMOutputDevice(pin = 19, initial_value = 0.0, frequency = 1000)

time_elapsed = 0
init_time = time()

left_motor_readings = []
right_motor_readings = []
ref_speed_readings = []
time_readings = []

ref_motor_speed_change_time = 2

initial_time = time()

ref_speed_2 = 0.6


left_motor_control.enable_motor()
right_motor_control.enable_motor()

# while True: 
#     right_motor_control.set_motor_speed(ref_speed)
#     left_motor_control.set_motor_speed(ref_speed)
    
#     sleep(0.07)
    
while time_elapsed < 20:
    if time() - initial_time < ref_motor_speed_change_time:
        ref_speed = 0 
    else: 
        print('Step response')
        ref_speed = ref_speed_2 
        
    right_motor_control.set_motor_speed(ref_speed)
    left_motor_control.set_motor_speed(ref_speed)
    
    left_motor_readings.append(left_motor_control.encoder.get_motor_speed())
    right_motor_readings.append(right_motor_control.encoder.get_motor_speed()) 
    ref_speed_readings.append(ref_speed)
    time_readings.append(time()- initial_time) 
    
    time_elapsed = time() - init_time
    
    
    sleep(0.07)
#     time_elapsed = time() - init_time
#     # left_moor_control.encoder.get_motor_speed()
#     # right_motor_control.encoder.get_motor_speed()
#     # speed = encoder.get_motor_speed()
#     # PWM_test.value = 0.4

# mean_left_motor_readings = sum(left_motor_readings[9:])/len(left_motor_readings[9:])
# mean_right_motor_readings = sum(right_motor_readings[9:])/len(right_motor_readings[9:])

# print("Average left motor reading: ", mean_left_motor_readings, "    Average right motor reading: ", mean_right_motor_readings)

# plot_time = range(0, len(left_motor_readings[9:]))

# # plt.plot(plot_time, left_motor_readings[9:])
# # plt.axhline(y = mean_left_motor_readings)
# # plt.savefig("ece4191/ECE4191-Team-27/pics/left_motor_readings_2.png")
# # plt.close()

# # plt.plot(plot_time, right_motor_readings[9:])
# # plt.axhline(y = mean_right_motor_readings)
# # plt.savefig("ece4191/ECE4191-Team-27/pics/right_motor_readings_2.png")
# # plt.close()

plt.plot(time_readings,  left_motor_readings)
plt.savefig(f"ece4191/ECE4191-Team-27/pics/left_motor_readings_2_{ref_speed_2}.png")
plt.close()


plt.plot(time_readings,  right_motor_readings)
plt.savefig(f"ece4191/ECE4191-Team-27/pics/right_motor_readings_2_{ref_speed_2}.png")
plt.close()

    
# right_motor_control.set_motor_speed(0.1)
# right_motor_control.set_motor_speed(0.4)
# right_motor_control.set_motor_speed(-0.2)