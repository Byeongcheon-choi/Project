#!python3

import RRT
import math
import time

#  a test of it just driving forward
def trial1():
    startPoint = (300,550,(3*math.pi)/2)
    targetPoint = (100,550,(3*math.pi)/2)
    obstacleList = [((200,450),(300,250)),((350,700),(400,600))]
    (vertices,edges,path) = RRT.RRTPlanner(startPoint,targetPoint,obstacleList)
    RRT.plotCSpace(obstacleList,edges,vertices,path)
# For 1000 iterations, took us 11 ms to generate the points, 223 ms for us to get the closest, valid point
#  0 ms for us to get the trajectory, and 14501 ms to see if trajectory is collision free

def trial2():
    startPoint = (400,100,(3*math.pi)/2)
    targetPoint = (100,550,(3*math.pi)/2)
    obstacleList = [((200,450),(300,250)),((350,700),(400,600))]
    (vertices,edges,path) = RRT.RRTPlanner(startPoint,targetPoint,obstacleList)
    RRT.plotCSpace(obstacleList,edges,vertices,path, plotTitle='RRT Algorithm After 10000 Iterations')

def trial3():
    startPoint = (400,100,(3*math.pi)/2)
    targetPoint = (100,550,(3*math.pi)/2)
    time0 = int(round(time.time() * 1000))
    obstacleList = []
    (vertices,edges,path) = RRT.RRTPlanner(startPoint,targetPoint,obstacleList)
    time1 = int(round(time.time() * 1000))
    print("0 Obstacles: " + str(time1 - time0))
    obstacleList = [((150,150),(250,250))]
    (vertices,edges,path) = RRT.RRTPlanner(startPoint,targetPoint,obstacleList)
    time2 = int(round(time.time() * 1000))
    print("1 Obstacles: " + str(time2 - time1))
    obstacleList = [((150,150),(250,250)),((250,250),(350,350))]
    (vertices,edges,path) = RRT.RRTPlanner(startPoint,targetPoint,obstacleList)
    time3 = int(round(time.time() * 1000))
    print("2 Obstacles: " + str(time3 - time2))
    obstacleList = [((150,150),(250,250)),((250,250),(350,350)),((350,350),(450,450))]
    (vertices,edges,path) = RRT.RRTPlanner(startPoint,targetPoint,obstacleList)
    time4 = int(round(time.time() * 1000))
    print("3 Obstacles: " + str(time4 - time3))
    # all times are for 1000 iterations
    # 0 Obstacles, 500 ms; 1 Obstacle, 11179 ms; 2 Obstacles, 31881 ms; 3 Obstacles, 52555 ms


#To be used in generating reverse RRTplanner
def trial4():
    startPoint = (300,550,(3*math.pi)/2)
    targetPoint = (100,550,(3*math.pi)/2)
    obstacleList = [((200,450),(300,250)),((350,700),(400,600))]
    (vertices,edges,path) = RRT.reverseRRTPlanner(startPoint,targetPoint,obstacleList)
    RRT.plotCSpace(obstacleList,edges,vertices,path)

def trial5():
    startPoint = (400,100,(3*math.pi)/2)
    targetPoint = (100,550,(3*math.pi)/2)
    obstacleList = [((50,50),(150,150)),((150,150),(250,250)),((250,250),(350,350)),((350,350),(450,450))]
    (vertices,edges,path) = RRT.RRTPlanner(startPoint,targetPoint,obstacleList)
    RRT.plotCSpace(obstacleList,edges,vertices,path, plotTitle="RRT Algorithm Where Path Couldn't Be Found")


trial4()
# print(RRT.genAchievableTraj((300,550,(3*math.pi)/2), (100,550,(3*math.pi)/2)))
# print(RRT.genAchievableTraj((286.59234, 546.60575, 4.8427), (100, 550, 4.71238898038469)))
# print(RRT.genAchievableTraj((300,550,(3*math.pi)/2), (300,550,0)))


