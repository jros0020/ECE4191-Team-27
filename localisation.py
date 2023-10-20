def front_localisation(dist1, dist2):
    while(abs(dist1-dist2)>0.05):
        #dist1 left dist 2 right
        if(dist1>dist2):
            print('Turning Right')
            left_motor_speed = 0.5
            right_motor_speed = -0.5
        elif(dist2>dist1):
            print('Turning Left')
            left_motor_speed = -0.5
            right_motor_speed = 0.5
    
    left_motor_speed = 0
    right_motor_speed = 0
        
    return 