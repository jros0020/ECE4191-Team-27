import numpy as np

class Planner:
    
    def __init__(self,obstacles):
        self.obstacles = obstacles
    
    def euc_distance(self,x1,y1,x2,y2):
        dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return dist
    
    # Find the direction that reduces distance to goal
    # Check Distance represents how many metres ahead we evaluate
    def find_direction(self,check_distance,goal_x,goal_y,x,y):
        distances = np.array([])

        #Was getting errors using the np.pi/4 angles, so added an 
        #array of the correct trig values (probably a better way to do this...)
        ang = np.array([0,np.pi/2, 2*np.pi/2, 3*np.pi/2]) #list of cardinal directions
        directions = np.array([[1, 0], [0,1], [-1,0], [0,-1]])#cos,sin of each of those directions

        #Check cardinal directions to see which path is clear of 
        #obstacles and has the shortests distance to goal
        for x_check,y_check in directions:
            x_new = x + check_distance*x_check
            y_new = y + check_distance*y_check

            if (self.check_collision(x_new,y_new)):
                distances = np.append(distances,np.inf)
            else:
                distances = np.append(distances,self.euc_distance(x_new,y_new,goal_x,goal_y))
            
        
        #Return the desired direction (relative to the arena, not the robot)
        best_idx = np.argmin(distances)

        return ang[best_idx]
    
    def check_collision(self,x,y):
        
        #Get the distance to each obstacle
        for obstacle in self.obstacles:
            obstacle_x, obstacle_y = obstacle
            obstacle_distance = self.euc_distance(x,y,obstacle_x,obstacle_y)

            #If within 0.1m or going outside the arena (0,0 at bottom left), don't use that path.
            if(obstacle_distance < 0.1 or x < 0 or y < 0):
                print(f'Collision if {x},{y}')
                return True

        return False
    
obstacles = np.array([[0.5,0.2],[0.4,0.3]])
x,y = [0.4,0.2]
goal_x,goal_y = [0.4,0.4]
planner = Planner(obstacles)
direction = planner.find_direction(0.05,goal_x,goal_y,x,y)

print(np.degrees(direction))