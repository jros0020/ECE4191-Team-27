def obstacle_detect(x,y,th,dist1,dist2,dist3,dist4,dist5,obstacles):
    #sensor order: 1/2 front, 3 right, 4 back, 5 left
    ##check if wall/obstacle

    if(dist1<0.1):
        x1 = x + dist1*np.cos(th)
        y1 = y + dist1*np.sin(th)
        obstacles.append([x1,y1])
    if(dist2<0.1):
        x2 = x + dist2*np.cos(th)
        y2 = y + dist2*np.sin(th)
        obstacles.append([x2,y2])
    if(dist3<0.1):
        x3 = x + dist3*np.cos(th-np.pi/2)
        y3 = y + dist3*np.sin(th-np.pi/2)
        obstacles.append([x3,y3])
    if(dist4<0.1):
        x4 = x + dist4*np.cos(th+np.pi)
        y4 = y + dist4*np.sin(th+np.pi)
        obstacles.append([x4,y4])
    if(dist5<0.1):
        x5 = x + dist5*np.cos(th+np.pi/2)
        y5 = y + dist5*np.sin(th+np.pi/2)
        obstacles.append([x5,y5])

    return obstacles
    
    
        
    
    
