from pygame import draw
import random

class kilobot:
    position;
    direction;
    def __init__ (self, winSize, coordInit = False):
        x = random.uniform(0,winSize(0))
        y = random.uniform(0,winSize(1))
        self.position = (x,y)

        if coordInit = True:
            #TODO initialize the pos of the fours coordinate robots
