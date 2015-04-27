from Agent import Agent
from Ball import Ball
from Obstacle import Obstacle
from LinearAlegebraUtils import getYPRFromVector
import numpy as np
from random import randint

class AlexBrain(object):
    '''
    classdocs
    '''

    def __init__(self,  deltaPos, deltaRot):      
        self.deltaPos = deltaPos #np.array([1,0,0])
        self.deltaRot = deltaRot #np.array([0,0,0])
    
    def takeStep(self, myTeam=[], enemyTeam=[], balls=[], obstacles=[]):
        return self.deltaPos, self.deltaRot
