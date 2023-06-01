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
    

def Set_Tank(start=(0,0,0)):
    gtot=len(tanks)
    tank=Gun(start=start,id=gtot)
    tanks.append(tank)


import matplotlib.pyplot as plt

def Run(dt):
    global missiles
    global tanks
    global bullets
    flag1,flag2=True,True
    tmp=[]
    flag1,flag2=False,False
    for bullet in bullets:
        if(bullet.able==False):
            continue
        bullet.Move(dt=dt)
        tmp.append(bullet)
        bullets=tmp.copy()

    for id,missile in enumerate(missiles):
        if(missile.able==False):
            continue
        flag1=True
        tank=tanks[missile.tid]
        missile.Guide(target=tank,dt=dt)
        if(missile.Hit(tank.xyz)):
            tank.able=False
            missile.able=False
        if(missile.xyz[1]<0):
            missile.able=False
        for b in bullets:
            if(b.able==False):
                continue
            if(missile.Intercepted(b)):
                break

    for id,tank in enumerate(tanks):
        if(tank.able==False):
            continue
        tank.Move(dt)
        flag2=True
        if(flag1==False):
            continue
        mnL=1145141919810
        mn_id=-1
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
        if(mn_id<0):
            continue
        bullet=tank.Intercept(target=missiles[mn_id],dt=dt)
        if(bullet!=None):
            bullets.append(bullet)

import socket

def Recv(conn):
    rec=''
    while True:
        tmp=conn.recv(1)
        if(tmp==b''):
            return b''
        tmp=tmp.decode('utf-8')
        if(tmp=='*'):
            break
        # print(tmp)
        rec+=tmp
    return rec

disconnected=False

def Interact(conn=socket.socket()):
    while True:
        global disconnected
        try:
            msg=Recv(conn=conn)
            if(msg==b''):
                print('connection disconnected')
                disconnected=True
                break
        except:
            print('connection disconnected')
            disconnected=True
            break
        # print(msg)
        msg=msg.split(' ')
        head=msg[0]
        if(head=="Modify_Tank"):
            tid=int(msg[1])
            x,y,z=float(msg[2]),float(msg[3]),float(msg[4])
            vx,vy,vz=float(msg[5]),float(msg[6]),float(msg[7])
            tanks[tid].ModifyTo((x,y,z))
            # print(tanks[tid].Len((vx,vy,vz)))
            tanks[tid].SetV((vx,vy,vz))

        if(head=="Set_Tank"):
            x,y,z=float(msg[1]),float(msg[2]),float(msg[2])
            # print((x,y,z))
            Set_Tank((x,y,z))
            # print(len(tanks))
        
        if(head=="Launch_Missile"):
            x,y,z=float(msg[1]),float(msg[2]),float(msg[3])
            tid=int(msg[4])
            Launch_Missile((x,y,z),tanks[tid])

        if(head=="Get_Missiles"):
            ans=["Missiles_info",0]
            # ans.append(str(len(missiles)))
            for m in missiles:
                if(m.exploded):
                    continue
                if(m.able==False):
                    m.exploded=True
                x,y,z=m.xyz
                tmp=[str(m.id)]
                tmp.append("{}|{}|{}".format(format(x,".2f"),format(y,".2f"),format(z,".2f")))
                theta,phi,gamma=m.angle1['theta'],m.angle1['phi'],m.angle1['gamma']
                tmp.append("{}|{}|{}".format(format(theta,".3f"),format(phi,".3f"),format(gamma,'.3f')))
                tmp.append("{}".format(1 if m.able else 0))
                ans.append('|'.join(tmp))
                ans[1]+=1
            ans[1]=str(ans[1])
            re=' '.join(ans)
            re=re.encode('utf-8')
            conn.send(re)
        
        if(head=="Get_Bullets"):
            ans=["Bullets_info"]
            ans.append(str(len(bullets)))
            for b in bullets:
                x,y,z=b.xyz
                tmp=[]
                tmp.append("{}|{}|{}".format(format(x,".2f"),format(y,".2f"),format(z,".2f")))
                theta,phi=m.angle1['theta'],m.angle1['phi']
                tmp.append("{}|{}".format(format(theta,".3f"),format(phi,".3f")))
                ans.append('|'.join(tmp))
            re=' '.join(ans)
            re=re.encode('utf-8')
            conn.send(re)

        if(head=="Get_Tanks"):
            ans=["Tanks_info",0]
            for t in tanks:
                if(t.exploded):
                    continue
                t.exploded=not t.able
                tmp=[str(t.id)]
                x,y,z=t.look
                tmp.append("{}|{}|{}".format(format(x,".3f"),format(y,".3f"),format(z,".3f")))
                tmp.append('{}|{}'.format((1 if t.onfire else 0),(1 if t.able else 0)))
                ans.append('|'.join(tmp))
                ans[1]+=1
            ans[1]=str(ans[1])
            re=' '.join(ans)
            re=re.encode('utf-8')
            conn.send(re)

import os

def Start_Server(port=1145):
    s=socket.socket()
    host=socket.gethostname()
    s.bind(("127.0.0.1",port))
    print(host)
    s.listen(1)
    print("listening port:{0}".format(port))
    try:
        os.startfile("missile.exe")
    except:
        pass
    conn,addr=s.accept()
    print("connection from {}".format(addr))
    _thread.start_new_thread(Interact,(conn,))
    pass

debug=False
times=[]
thetas=[]
dts=[]

if __name__=="__main__":
    Start_Server()
    FPS=600
    clock_last=time.perf_counter()
    count=0
    while not disconnected:
        while time.perf_counter()<(count+1)/FPS:
            time.sleep(0.1/FPS)
            continue
        count+=1
        clock_now=time.perf_counter()
        dt=clock_now-clock_last
        # if(int(clock_now)-int(clock_last)>=1):
        #     print(clock_now)
        clock_last=clock_now
        Run(dt=1/FPS)
        if(debug and len(missiles)>0):
            dts.append(dt)
            times.append(clock_now)
            thetas.append(missiles[0].angle1['theta'])
            if(missiles[0].able==False):
                break
    if debug:
        plt.plot(times,thetas)
        plt.show()
        plt.plot(times,dts)
        plt.show()