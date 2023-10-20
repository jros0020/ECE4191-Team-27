from gpiozero import InputDevice, OutputDevice
from time import sleep, time

class DistanceSensor(): 
    def __init__(self, name, pin_ECHO, pin_TRIGGER) -> None:
        self.name = name 

        self.dist_sensor_echo = InputDevice(pin_ECHO)
        self.dist_sensor_trigger = OutputDevice(pin_TRIGGER)
        self.prev_distance = 0 
        self.timeout = 1
        self.max_distance = 400

    def trigger_dist_sensor(self): 
        # Trigger a pulse
        self.dist_sensor_trigger.on()

        sleep(0.00001)

        self.dist_sensor_trigger.off() 

    def get_distance(self): 
        self.trigger_dist_sensor()

        # Measure the time for the echo pulse to return
        pulse_start = time()

        while not self.dist_sensor_echo.is_active:
            pulse_start = time()
            
        pulse_end = time()
        while self.dist_sensor_echo.is_active:
            pulse_end = time()

            if (pulse_end - pulse_start) > self.timeout: 
                print(f'{self.name} timeout')
                return self.prev_distance
            
        pulse_duration = pulse_end - pulse_start
        
        # Calculate distance using the speed of sound (343 m/s)
        distance = pulse_duration * 17150  # Divided by 2 to account for the return trip
        distance = round(distance, 2)  # Round to two decimal places
        
        distance = self.max_distance if distance > self.max_distance else distance

        self.prev_distance = distance
        return distance 
    

left_dist_sensor = DistanceSensor('left', pin_ECHO= 23, pin_TRIGGER= 18)
front_left_dist_sensor = DistanceSensor('front_left', pin_ECHO= 16, pin_TRIGGER= 24)
front_right_dist_sensor = DistanceSensor('front_right', pin_ECHO= 14, pin_TRIGGER= 12)
right_dist_sensor = DistanceSensor('right', pin_ECHO= 15, pin_TRIGGER= 20)

dist_sensors = {'left': left_dist_sensor, 'front_left': front_left_dist_sensor, 'front_right': front_right_dist_sensor, 'right': right_dist_sensor}

while True: 

    for dist_sensor in dist_sensors.values(): 
        print(f'{dist_sensor.name}: {dist_sensor.get_distance()}')
        sleep(0.1)