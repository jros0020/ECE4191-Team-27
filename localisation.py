import speed_control as controller
import numpy as np

def localisation(self, direction, dist1, dist2, dist3, dist4, dist5):
#dist 1 front left, dist 2 front right, dist 3 right, dist 4 back, dist 5 left
#turn to where it thinks the correct direction is
    while(abs(self.th  - direction) > 0.05):
        if(self.th - direction < 0):
            print('Turn Right Initial')
            #drive signal to motor to turn right
            duty_cycle_l,duty_cycle_r = controller.drive(0,0.1,self.wl,self.wr)

            # Simulate robot motion - send duty cycle command to robot
            self.pose_update(duty_cycle_l,duty_cycle_r)



        elif(self.th - direction > 0):
            # drive signal to motor to turn left
            print('Turn Left Initial')
            duty_cycle_l,duty_cycle_r = controller.drive(0,-0.1,self.wl,self.wr)

            # Simulate robot motion - send duty cycle command to robot
            self.pose_update(duty_cycle_l,duty_cycle_r)

        

    #correct pose
    while(abs(dist1-dist2) > 0.01):
        if(dist1 - dist2 < 0):
            # drive signal to motor to turn right
            print('Turn Right Localising')
            #drive signal to motor to turn right
            duty_cycle_l,duty_cycle_r = controller.drive(0,0.1,self.wl,self.wr)

            # Simulate robot motion - send duty cycle command to robot
            self.pose_update(duty_cycle_l,duty_cycle_r)

        elif(dist2 - dist1 > 0.01):
            # drive signal to motor to turn left
            print('Turn Left Localising')
            duty_cycle_l,duty_cycle_r = controller.drive(0,-0.1,self.wl,self.wr)

            # Simulate robot motion - send duty cycle command to robot
            self.pose_update(duty_cycle_l,duty_cycle_r)

    #after the while function, robot should be roughly straight on, so set new positions
    #origin is bottom left of arena(same as code in documentation)
    #facing right
    if(direction == 0):
        self.x = ((2-dist1)+dist4)/2
        self.y = ((2-dist5)+dist3)/2
        self.th = direction
    # facing up
    if(direction == np.pi/4):
        self.x = ((2-dist3)+dist5)/2
        self.y = ((2-dist1)+dist4)/2
        self.th = direction
    #facing left
    if(direction == np.pi/2):
        self.x = ((2-dist4)+dist1)/2
        self.y = ((2-dist3)+dist5)/2
        self.th = direction
    #facing down
    if(direction == np.pi/2):
        self.x = ((2-dist4)+dist1)/2
        self.y = ((2-dist3)+dist5)/2
        self.th = direction

    return 