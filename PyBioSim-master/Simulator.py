'''
Created on Jan 16, 2015

@author: Arindam
'''

import numpy as np
import matplotlib.pyplot as plt
from numpy import *
from World import *
from Agent import Agent
from Obstacle import *
from pylab import *
from Ball import Ball
from LinearAlegebraUtils import distBetween
from AlexBrain import AlexBrain
from Team import Team



#Called once for initialization
'''
Usage guidelines:
1. Define globals required for the simulation in the __init__ constructor, here we define a bunch of waypoints for the ball
2. Initialize the globals in the setup() method. 
'''

class Simulator(object):
    def __init__(self, world, simTime, fps, imageDirName):
        self.world = world
        self.simTime = simTime
        self.fps = fps
        self.imageDirName = imageDirName
        self.currWP = 0
        self.ballWPs = [array([50.0, -100.0, 0.0]), array([0.0, 100.0, -70.0]), array([50.0, 20.0, 100.0]),array([-30.0, 50.0, -100.0]), array([80.0, -50.0, 50.0]), array([80.0, -50.0, -50.0]), array([-65.0, 20.0, 50.0]), array([-50.0, 20.0, -60.0])]
        self.file = open('positions_ground_truth.txt','w+')

    def setup(self):    
        #setup directory to save the images
        self.imageDirName = 'BioSim_Output_Images'
        try:
            os.mkdir(self.imageDirName)
        except:
            print self.imageDirName + " subdirectory already exists. OK."

         #define teams which the agents can be a part of
        teamA = Team("A", '#416341')  #Green Team
        teamB = Team("B", '#FF3A83')  #PurpleTeam
        teamC = Team("C", '#72AACA')  #Blue Team
        teamD = Team("D", '#996633')  #Yellow Team

        #Team A Green Team
        ag1Pos = array([100, 0, 0])
        ag1Rot = array([0, 0, 0])
        ag1Brain = AlexBrain(np.array([1,0,0]),np.array([0,0,0]))
        agent1 = Agent(teamA, ag1Pos, ag1Rot, ag1Brain, 15, 15)
                
        ag2Pos = array([100, 50, 0])
        ag2Rot = array([0, 0, 0])
        ag2Brain = AlexBrain(np.array([1,0,0]),np.array([0,0,0]))
        agent2 = Agent(teamA, ag2Pos, ag2Rot, ag2Brain, 15, 15)
        

        #Team B Purple Team
        ag3Pos = array([50, -100, 0])
        ag3Rot = array([0, 0, 0])
        ag3Brain = AlexBrain(np.array([0,1,0]),np.array([0,0,0]))
        agent3 = Agent(teamB, ag3Pos, ag3Rot, ag3Brain, 15, 15)
         
        ag4Pos = array([0, -100, 0])
        ag4Rot = array([0, 0, 0])
        ag4Brain = AlexBrain(np.array([0,1,0]),np.array([0,0,0]))
        agent4 = Agent(teamB, ag4Pos, ag4Rot, ag4Brain, 15, 15)

        #Team C Blue Team
        ag5Pos = array([-50, -100, 0])
        ag5Rot = array([0, 0, 0])
        ag5Brain = AlexBrain(np.array([1,1,0]),np.array([0,0,0]))
        agent5 = Agent(teamC, ag5Pos, ag5Rot, ag5Brain, 15, 15)
        
        ag6Pos = array([0, 0, 0])
        ag6Rot = array([0, 0, 0])
        ag6Brain = AlexBrain(np.array([1,1,0]),np.array([0,0,0]))
        agent6 = Agent(teamC, ag6Pos, ag6Rot, ag6Brain, 15, 15)

        #Team D Yellow Team
        ag7Pos = array([-225, 0, 0])
        ag7Rot = array([0, 0, 0])
        ag7Brain = AlexBrain(np.array([1,0,0]),np.array([0,0,0]))
        agent7 = Agent(teamD, ag7Pos, ag7Rot, ag7Brain, 15, 15)
        
        ag8Pos = array([-225, 50, 0])
        ag8Rot = array([0, 0, 0])
        ag8Brain = AlexBrain(np.array([1,0,0]),np.array([0,0,0]))
        agent8 = Agent(teamD, ag8Pos, ag8Rot, ag8Brain, 15, 15)


        #Add the agent to the world
        self.world.agents.append(agent1)
        self.world.agents.append(agent2)
        self.world.agents.append(agent3)
        self.world.agents.append(agent4)
        self.world.agents.append(agent5)
        self.world.agents.append(agent6)
        self.world.agents.append(agent7)
        self.world.agents.append(agent8)
#         
        '''
        #define a bunch of obstacles
        ob1Pos = array([-50,-50,-50])
        ob1 = Obstacle(ob1Pos, 30)
         
        ob2Pos = array([80,-50,-50])
        ob2 = Obstacle(ob2Pos, 20)
         
        #add obstacles to the world
        self.world.obstacles.append(ob1);
        self.world.obstacles.append(ob2)
        '''
        #define a ball
        ball = Ball(array([0, 1100, 0]))
        
        
        #add the ball to the world
        self.world.balls.append(ball)
        
        
#called at a fixed 30fps always
    def fixedLoop(self):
        self.file.write("FRAME")
        self.file.write("\n")
        for agent in self.world.agents:
            for i in range(0,2):
                self.file.write(str(agent.position[i]))
                self.file.write(",")
            self.file.write("\n")
            agent.moveAgent(self.world)
         
        for ball in self.world.balls:  
            if len(self.ballWPs) > 0:  
                ball.moveBall(self.ballWPs[0], 1)
                if distBetween(ball.position, self.ballWPs[0]) < 0.5:
                    if len(self.ballWPs) > 0:
                        self.ballWPs.remove(self.ballWPs[0])

    
#Called at specifed fps
    def loop(self, ax):       
        self.world.draw(ax)
       
                
    def run(self):
        #Run setup once
        self.setup()
        
        #Setup loop
        timeStep = 1/double(30)
        frameProb = double(self.fps) / 30
        currTime = double(0)
        drawIndex = 0
        physicsIndex = 0
        while(currTime < self.simTime):
            self.fixedLoop()
            currProb = double(drawIndex)/double(physicsIndex+1)
            if currProb < frameProb:
                self.drawFrame(drawIndex)  
                drawIndex+=1
            physicsIndex+=1
            currTime+=double(timeStep)
     
        print "Physics ran for "+str(physicsIndex)+" steps"
        print "Drawing ran for "+str(drawIndex)+" steps"
            
    def drawFrame(self, loopIndex):
        fig = plt.figure(figsize=(16,12))
        ax = fig.add_subplot(111, projection='3d') 
        ax.axis('off')
        ax.view_init(elev = 90, azim = 90)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")    
        fname = self.imageDirName + '/' + str(int(100000000+loopIndex)) + '.jpg' # name the file 
        self.loop(ax)
        plt.gca().set_ylim(ax.get_ylim()[::-1])
        #plt.show()
        savefig(fname, format='jpg', dpi = 20)
        print 'Written Frame No.'+ str(loopIndex)+' to '+ fname
        plt.close()


#Simulation runs here
#set the size of the world
world = World(100, 100)
#specify which world to simulate, total simulation time, and frammerate for video
sim = Simulator(world, 6, 30, "images")
#run the simulation
sim.run()

'''
To create a video using the image sequence, execute the following command in command line.
>ffmpeg -f image2 -i "1%08d.jpg" -r 30 outPut.mp4
Make sure to set your current working directory to /images and have ffmpeg in your path.
'''

    