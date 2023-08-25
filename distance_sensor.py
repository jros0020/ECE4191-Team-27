from gpiozero import InputDevice, OutputDevice
from time import sleep, time



class DistanceSensorInfo: 
    def __init__(self, name, ECHO, TRIGGER):
        self.name = name
        self.ECHO = ECHO
        self.TRIGGER = TRIGGER

class DistanceSensorCluster: 
    def __init__(self, dist_sensor_info_dict, TRIGGER):
        self.dist_sensor_name_to_echo = {}

        for dist_sensor_name, dist_sensor_info in dist_sensor_info_dict.items():
            self.dist_sensor_name_to_echo[dist_sensor_name] = InputDevice(dist_sensor_info.ECHO)

        self.dist_sensor_cls_trigger = OutputDevice(TRIGGER)

    def trigger_cluster(self): 
        # Trigger a pulse
        self.dist_sensor_cls_trigger.on()

        sleep(0.00001)

        self.dist_sensor_cls_trigger.off()

    def calculate_single_dist(self, name): 
        self.trigger_cluster()

        dist_sensor_echo = self.dist_sensor_name_to_echo[name]

        # Measure the time for the echo pulse to return
        pulse_start = time()
        while not dist_sensor_echo.is_active:
            pulse_start = time()
            
        pulse_end = time()
        while dist_sensor_echo.is_active:
            pulse_end = time()
            
        pulse_duration = pulse_end - pulse_start
        
        # Calculate distance using the speed of sound (343 m/s)
        distance = pulse_duration * 17150  # Divided by 2 to account for the return trip
        distance = round(distance, 2)  # Round to two decimal places
        
        return distance 
    
    def calculate_cluster_dist(self): 
        dist_in_diff_directions = {}
        for name in self.dist_sensor_name_to_echo.keys():
            dist = self.calculate_single_dist(name) 
            dist_in_diff_directions[name] = self.calculate_single_dist(name)
    

front_dist_sensor = DistanceSensorInfo(name = 'front', ECHO = 21, TRIGGER = 20)
right_dist_sensor = DistanceSensorInfo(name = 'right', ECHO = 26, TRIGGER = 20)
left_dist_sensor = DistanceSensorInfo(name = 'left', ECHO = 16, TRIGGER = 20)

distance_sensor_obj_dict = {'front': front_dist_sensor, 'right': right_dist_sensor, 'left': left_dist_sensor}

dist_sensor_cls = DistanceSensorCluster(distance_sensor_obj_dict, 20)

while True: 
    # # print(f'Front: {dist_sensor_cls.calculate_single_dist("front")}')
    # # sleep(0.025)
    # print(f'left: {dist_sensor_cls.calculate_single_dist("left")}')
    # sleep(0.045)
    dist_sensor_cls.calculate_cluster_dist()
    sleep(1) 