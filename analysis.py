import numpy as np
from model import Missile_Brain
import time
import torch
from math import sin,cos,pi,sqrt,asin,acos,exp,tan
from missile import Missile,rand3
from tank import Gun
FPS=500
def Test(start,target):
    Dxz=(target[0]-start[0],target[2]-start[2])
    DL=(Dxz[0]**2+Dxz[1]**2)**0.5
    phi=acos(Dxz[0]/DL)
    if(Dxz[1]!=0):
        phi=-phi*Dxz[1]/abs(Dxz[1])
    missile=Missile(xyz=start,theta=pi/4,P=300,phi=phi,gamma=0,v=(10*Dxz[0]/DL,10,10*Dxz[1]/DL))
    tank=Gun(start=target,v=(15/1.414,0,15/1.414))
    tot_T=0.
    for i in range(50):
        # print("xyz:{} v:{} theta:{} P:{}".format(missile.xyz,missile.v,missile.angle2['theta'],missile.P))
        # if(i==20):
        #     missile.P=0
        for t in range(FPS):
            tot_T+=1/FPS
            tank.Move(dt=1/FPS)
            missile.Guide(target=tank,dt=1/FPS)
            if(missile.Hit(tank.xyz)):
                return True
            if(missile.xyz[1]<0):
                return False
    return False

from tqdm import tqdm
import matplotlib.pyplot as plt
import os
def Alpha(DL):
    k1,b1=0.02,2.25
    k2,b2=0.014,5.6
    L1,L2,L3=564,715,1020
    if(DL<L1):
        alpha=pi/min(k1*DL+b1,15.65)
    elif DL<L2:
        alpha=pi/min(k2*DL+b2,15.65)
    elif DL<L3:
        alpha=pi/15.65
    else:
        alpha=pi/16
    return alpha    

if __name__=='__main__':
    x=[]
    y=[]
    fuck=0
    tot=200
    for i in tqdm(range(tot)):
        start=rand3(0,2000,5,20)
        target=rand3(0,2000,5,20)
        Dxz=(target[0]-start[0],target[2]-start[2])
        DL=(Dxz[0]**2+Dxz[1]**2)**0.5
        if(not Test(start,target)):
            fuck+=1
            print("fuck={} {}".format(fuck,DL))
            print((start,target))
        continue
        low=2
        high=16
        split=40
        alpha=pi/high
        for i in range(split+1):
            tmp=high-(high-low)*i/split
            alpha=pi/tmp
            if(Test(start,target,alpha)):
                break
        x.append(DL)
        y.append(pi/alpha)
    exit()
    plt.scatter(x,y)
    plt.show()
    z=[(x[i],y[i]) for i in range(tot)]
    with open("test.txt","w") as f:
        f.write("{}".format(z))