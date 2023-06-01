import numpy as np
import time
import _thread
from math import sin,cos,pi,sqrt,asin,acos,exp,tan
from tank import Bullet,Gun
from missile import Missile

import random
def rand3(l,r,low,high):
    return (random.random()*(r-l)+l,random.random()*(high-low)+low,random.random()*(r-l)+l)


missiles=[]
tanks=[]
bullets=[]
mx,my,mz=[],[],[]

def Launch_Missile(start=(0,0,0),target=Gun()):
    txyz=target.xyz
    Dxz=(txyz[0]-start[0],txyz[2]-start[2])
    DL=(Dxz[0]**2+Dxz[1]**2)**0.5
    phi=acos(Dxz[0]/DL)
    if(Dxz[1]!=0):
        phi=-phi*Dxz[1]/abs(Dxz[1])
    mtot=len(missiles)
    missile=Missile(xyz=start,theta=pi/4,P=300,phi=phi,gamma=0,v=(10*Dxz[0]/DL,10,10*Dxz[1]/DL),id=mtot,tid=target.id)
    missiles.append(missile)
    mx.append([])
    my.append([])
    mz.append([])
    

def Set_Tank(start=(0,0,0),v=(5,0,10)):
    gtot=len(tanks)
    tank=Gun(start=start,id=gtot,v=v)
    tanks.append(tank)


import matplotlib.pyplot as plt


if __name__=="__main__":
    missile_start=rand3(-2000,2000,0,50)
    tank_start=rand3(-2000,2000,0,50)
    print("{},{}".format(missile_start,tank_start))
    Set_Tank(start=tank_start)
    tank=tanks[-1]
    Launch_Missile(start=missile_start,target=tank)
    FPS=500
    tot_T=0
    bs=0
    for i in range(50):
        # print(i)
        flag1,flag2=True,True
        for t in range(FPS):
            tot_T+=1/FPS
            tmp=[]
            flag1,flag2=False,False
            for bullet in bullets:
                if(bullet.able==False):
                    continue
                bullet.Move(dt=1/FPS)
                tmp.append(bullet)
                bullets=tmp.copy()

            for id,missile in enumerate(missiles):
                if(missile.able==False):
                    continue
                flag1=True
                tank=tanks[missile.tid]
                DL=missile.Len(missile.xyz-tank.xyz)
                missile.Guide(target=tank,dt=1/FPS)
                mx[id].append(missile.xyz[0])
                my[id].append(missile.xyz[1])
                mz[id].append(missile.xyz[2])
                if(missile.Hit(tank.xyz)):
                    tank.able=False
                if(missile.xyz[1]<0):
                    missile.able=False
                for b in bullets:
                    if(b.able==False):
                        continue
                    if(missile.Intercepted(b)):
                        print(DL)
                        break

            for id,tank in enumerate(tanks):
                if(tank.able==False):
                    continue
                tank.Move(1/FPS)
                flag2=True
                if(flag1==False):
                    continue
                mnL=1145141919810
                mn_id=0
                for mid,missile in enumerate(missiles):
                    if(missile.able==False):
                        continue
                    mxyz=missile.xyz
                    txyz=tank.xyz
                    Dxyz=mxyz-txyz
                    DL=tank.Len(Dxyz)
                    if(DL<mnL):
                        mnL=DL
                        mn_id=mid
                # mxyz=missiles[mn_id].xyz
                # T=mnL/tank.bullet_vol
                # txyz=mxyz+missiles[mn_id].v*T
                # tank.Rotate(target=mxyz)
                bullet=tank.Intercept(target=missiles[mn_id],dt=1/FPS)
                # continue
                if(bullet!=None):
                    bs+=1
                    bullets.append(bullet)
            
            if(not (flag1 and flag2)):
                break
        
        if(not (flag1 and flag2)):
            break

    print("Time:{} bullets={} Tank survive:{}".format(tot_T,bs,tanks[0].able))

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_zlabel('Y')
    ax.set_ylabel('Z')
    ax.set_xlabel('X')
    mtot=len(missiles)
    for i in range(mtot):
        mx[i]=np.array(mx[i],dtype=float)
        my[i]=np.array(my[i],dtype=float)
        mz[i]=np.array(mz[i],dtype=float)
        ax.plot(mx[i],mz[i],my[i])
    for b in bullets:
        x,y,z=b.xyz
        ax.scatter(x,z,y,c='gold',marker='.')

    ax.scatter(tank_start[0], tank_start[2], tank_start[1], c='r', marker='^')
    ax.scatter(tanks[0].xyz[0], tanks[0].xyz[2], tanks[0].xyz[1], c='purple', marker='^')
    ax.scatter(missile_start[0],missile_start[2],missile_start[1],c='g',marker='o')
    plt.show()