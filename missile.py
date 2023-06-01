import numpy as np
import time
from math import sin,cos,pi,sqrt,asin,acos,exp,tan
from tank import Bullet,Gun
g=9.8
FPS=500
debug=1000
debug1=False
class Missile:
    #地面系 x y z
    #弹体坐标系 x1 y1 z1 俯仰角theta 偏航角phi 倾斜角gamma
    #弹道坐标系 x2 y2 z2 弹道倾角theta 弹道偏角phi
    #速度坐标系 x3 y3 z3 攻角alpha 侧滑角beta 速度滚转角gamma
    #质量m 推进力P m通常在20kg以下 P通常在1.5mg~3mg之间
    def __init__(self,xyz=(0.,0.,0.),theta=0.,phi=0.,m=15,P=300.,length=1.1,diameter=0.127,gamma=0.,v=(0.,0.,0.),id=0,tid=0):
        self.xyz=np.array(xyz,dtype=float)
        self.start=self.xyz.copy()
        self.angle1={'theta':theta,'phi':phi,'gamma':gamma}
        self.v=np.array(v,dtype=float)
        self.V=self.Len(v)
        self.angle2={'theta':0.,'phi':-pi/2}
        if(v[1]!=0):
            self.angle2['theta']=asin(v[1]/self.V)
        if(v[0]!=0):
            self.angle2['phi']=acos(v[0]/self.Len((v[0],0,v[2])))
        if(v[2]!=0):
            self.angle2['phi']=-self.angle2['phi']*v[2]/abs(v[2])

        self.P=P
        self.m=m
        self.L=length
        self.D=diameter
        self.S=0.25*pi*diameter*diameter
        self.Jx=m*diameter*diameter/8
        self.Jy=self.Jz=m*(diameter*diameter/16+length*length/12)
        self.Omega=np.array([0.,0.,0.])#角速度ωx ωy ωz
        self.mxyz=np.array([0.,0.,0.])#可控力矩系数mx my mz
        self.Beta=np.array([0.,0.,0.])#角加速度βx βy βz
        self.able=True
        self.exploded=False
        self.id=id
        self.tid=tid
        self.tot_Time=0
        self.Update()

    def Update(self):
        if(self.tot_Time>15):
            self.P=0
        V=self.V
        a1=self.angle1
        a2=self.angle2
        v=(V*cos(a2['theta'])*cos(a2['phi']),V*sin(a2['theta']),-V*cos(a2['theta'])*sin(a2['phi']))
        self.v=np.array(v)
        self.angle3={'alpha':0.,'beta':pi/2,'gamma':0.}
        self.angle3['beta']=asin(cos(a2['theta'])*((cos(a1['gamma'])*sin(a1['phi']-a2['phi'])+sin(a1['theta'])*sin(a1['gamma'])*cos(a1['phi']-a2['phi'])))-sin(a2['theta'])*cos(a1['theta'])*sin(a1['gamma']))
        beta=self.angle3['beta']
        self.angle3['alpha']=asin((cos(a2['theta'])*(sin(a1['theta'])*cos(a1['gamma'])*cos(a1['phi']-a2['phi'])-sin(a1['gamma'])*sin(a1['phi']-a2['phi']))-sin(a2['theta'])*cos(a1['theta'])*cos(a1['gamma']))/cos(beta))
        alpha=self.angle3['alpha']
        self.angle3['gamma']=asin(cos(alpha)*sin(beta)*sin(a1['theta'])-sin(alpha)*sin(beta)*cos(a1['gamma'])*cos(a1['theta'])+cos(beta)*sin(a1['gamma'])*cos(a1['theta']))/cos(a2['theta'])

        self.xyz1=np.matmul(self.L01(),self.xyz.T).T
        self.xyz2=np.matmul(self.L02(),self.xyz.T).T
        self.xyz3=np.matmul(self.L23(),self.xyz2.T).T
        self.test=np.matmul(self.L31(),self.xyz3.T).T
    
    def Len(self,v):
        return sqrt(v[0]**2+v[1]**2+v[2]**2)

    def L01(self):#地面系->弹体坐标系 转换矩阵
        theta=self.angle1['theta']
        phi=self.angle1['phi']
        gamma=self.angle1['gamma']
        L1=np.array([[1,0,0],
                     [0,cos(gamma),sin(gamma)],
                     [0,-sin(gamma),cos(gamma)]])
        L2=np.array([[cos(theta),sin(theta),0],
                     [-sin(theta),cos(theta),0],
                     [0,0,1]])
        L3=np.array([[cos(phi),0,-sin(phi)],
                     [0,1,0],
                     [sin(phi),0,cos(phi)]])
        return np.matmul(L1,np.matmul(L2,L3))
    
    def L02(self):#地面系->弹道坐标系 转换矩阵
        theta=self.angle2['theta']
        phi=self.angle2['phi']
        return np.array([[cos(theta)*cos(phi),sin(theta),-cos(theta)*sin(phi)],
                         [-sin(theta)*cos(phi),cos(theta),sin(theta)*sin(phi)],
                         [sin(phi),0,cos(phi)]])
    
    def L23(self):#弹道坐标系->速度坐标系 转换矩阵
        gamma=self.angle3['gamma']
        return np.array([[1,0,0],
                         [0,cos(gamma),sin(gamma)],
                         [0,-sin(gamma),cos(gamma)]])
    
    def L31(self):#速度坐标系->弹体坐标系 转换矩阵
        alpha=self.angle3['alpha']
        beta=self.angle3['beta']
        return np.array([[cos(alpha)*cos(beta),sin(alpha),-cos(alpha)*sin(beta)],
                         [-sin(alpha)*cos(beta),cos(alpha),sin(alpha)*sin(beta)],
                         [sin(beta),0,cos(beta)]])
    
    def Move(self,dt=1e-5,h0=0):#海拔h(km)
        alpha=self.angle3['alpha']
        beta=self.angle3['beta']
        row_sl=1.225#海平面大气密度
        h=h0+self.xyz[1]/1000
        if(h<11.0191):
            W=1-h/44.3308
            T=288.15
            rou=row_sl*(W**4.2559)
            a=20.0468*sqrt(T)#音速a
        else:
            W=exp((14.9647-h)/6.3416)
            T=216.65
            rou=0.15895*row_sl*W
            a=20.0468*sqrt(T)
        # print(rou)
        # print(a)
        V=self.V
        S=self.S
        K=0.5*rou*(V**2)*S#1/2*ρ*V^2

        cr_l=0.9
        cr_h=1.05
        cx,cy,cz=0.01,0.1,0.1
        Ma=V/a
        if Ma<cr_l:
            cx=cx/sqrt(1-Ma*Ma)
            cy=2*pi/sqrt(1-Ma*Ma)+2*cos(self.angle3['alpha'])
            cz=2*pi/sqrt(1-Ma*Ma)+2*cos(self.angle3['beta'])
        elif Ma>cr_h:
            cx=cx/sqrt(Ma*Ma-1)
            cy=2*pi/sqrt(Ma*Ma-1)+2*cos(self.angle3['alpha'])
            cz=2*pi/sqrt(Ma*Ma-1)+2*cos(self.angle3['beta'])
        else:
            cx=cx/sqrt(cr_h*cr_h-1)
            cy=2*pi
            cz=2*pi

        self.Beta=self.mxyz*K*self.L
        X=cx*K
        Y=cy*K*alpha
        Z=-cz*K*beta
        P=self.P
        m=self.m
        theta2=self.angle2['theta']
        phi2=self.angle2['phi']
        gamma2=self.angle3['gamma']
        dV=dt*(P*cos(alpha)*cos(beta)-X-m*g*sin(theta2))/m
        dtheta2=dt*(P*(sin(alpha)*cos(gamma2)+cos(alpha)*sin(beta)*sin(gamma2))+Y*cos(gamma2)-Z*sin(gamma2)-m*g*cos(theta2))/(m*V)
        dphi2=-dt*(P*(sin(alpha)*sin(gamma2)-cos(alpha)*sin(beta)*cos(gamma2))+Y*sin(gamma2)+Z*cos(gamma2))/(m*V*cos(theta2))

        wx,wy,wz=self.Omega
        dxyz=dt*np.array([V*cos(theta2)*cos(phi2),V*sin(theta2),-V*cos(theta2)*sin(phi2)])

        theta1=self.angle1['theta']
        gamma1=self.angle1['gamma']
        # print(gamma1)
        phi1=self.angle1['phi']

        dtheta1=dt*(wy*sin(gamma1)+wz*cos(gamma1))
        dphi1=dt*(wy*cos(gamma1)-wz*sin(gamma1))/cos(theta1)
        dgamma1=dt*(wx-tan(theta1)*(wy*cos(gamma1)-wz*sin(gamma1)))


        self.V=V+dV
        self.angle2['theta']=theta2+dtheta2
        self.angle2['phi']=phi2+dphi2
        self.xyz+=dxyz
        self.Omega+=self.Beta*dt
        self.angle1['theta']=theta1+dtheta1
        self.angle1['gamma']=gamma1+dgamma1
        self.angle1['phi']=phi1+dphi1
        self.tot_Time+=dt
        self.Update()

    def DeltaAngle(self,xyz,target):
        target=np.array(target)
        xyz=np.array(xyz)
        Dxyz=target-xyz
        Dxyz_=np.matmul(self.L01(),Dxyz.T).T
        # print((Dxyz,Dxyz_,self.angle1))
        # time.sleep(0.5)
        L=self.Len(Dxyz_)
        LXZ=self.Len((Dxyz_[0],0,Dxyz_[2]))
        Dtheta=0
        Dphi=0
        if(L>0):
            Dtheta=asin(Dxyz_[1]/L)
            if(LXZ>0):
                Dphi=acos(Dxyz_[0]/LXZ)
            if(Dxyz_[2]!=0):
                Dphi=-Dphi*Dxyz_[2]/abs(Dxyz_[2])
        return Dtheta,Dphi

    def rou_a(self,h0=0):
        row_sl=1.225#海平面大气密度
        h=h0+self.xyz[1]/1000
        if(h<11.0191):
            W=1-h/44.3308
            T=288.15
            rou=row_sl*(W**4.2559)
            a=20.0468*sqrt(T)#音速a
        else:
            W=exp((14.9647-h)/6.3416)
            T=216.65
            rou=0.15895*row_sl*W
            a=20.0468*sqrt(T)
        return rou,a

    def Rotate(self,target,h0=0,dt=1e-3,final_target=True):
        rou,a=self.rou_a(h0=h0)
        V=self.V
        S=self.S
        K=0.5*rou*(V**2)*S*self.L#1/2*ρ*V^2*L
        Ma=V/a
        Dtheta,Dphi=self.DeltaAngle(self.xyz,target)
        Dtheta_,Dphi_=self.DeltaAngle(self.xyz+self.v*dt,target)
        dtheta,dphi=(Dtheta_-Dtheta)/dt,(Dphi_-Dphi)/dt
        eps=1e-6
        wy,wz=self.Omega[1:]
        wz-=dtheta
        wy-=dphi
        mz,my=0,0
        if(abs(Dtheta)>eps):
            if(wz*Dtheta<0 or wz*wz/(2*K)>=abs(Dtheta)-eps):
                mz=-abs(wz)/wz
            else:
                mz=abs(Dtheta)/Dtheta
        
        if(abs(Dphi)>eps):
            if(wy*Dphi<0 or wy*wy/(2*K)>=abs(Dphi)-eps):
                my=-abs(wy)/wy
            else:
                my=abs(Dphi)/Dphi
        self.mxyz[1]=my
        self.mxyz[2]=mz
        # print("{} {}".format((Dtheta,Dphi,my,mz),target))
        txyz=np.array(target)
        dxyz=txyz-self.xyz
        dis=self.Len(dxyz)
        # return
        if(final_target and dis<200):#矢量发动机模拟 最后阶段冲刺
            # time.sleep(100000)
            v_=self.V*dxyz/dis
            P=180
            a=P/self.m
            Dv=self.Len(v_-self.v)
            dv=(v_-self.v)/Dv
            v=self.v+dv*min(a*dt,Dv)
            if(v[1]!=0):
                self.angle2['theta']=asin(v[1]/self.V)
            if(v[0]!=0):
                self.angle2['phi']=acos(v[0]/self.Len((v[0],0,v[2])))
            if(v[2]!=0):
                self.angle2['phi']=-self.angle2['phi']*v[2]/abs(v[2])
        
        
    def target_mid(self,target,alpha=None):
        if(alpha!=None):
            theta=alpha
        else:
            txyz=np.array(target,dtype=float)
            DL=self.Len((self.start[0]-txyz[0],0,self.start[2]-txyz[2]))
            # print(DL)
            theta=self.Alpha(DL)
            # theta=pi/16
        x,y,z=self.xyz
        tx,ty,tz=target
        dx,dy,dz=tx-x,ty-y,tz-z
        dxz=(dx**2+dz**2)**(1/2)
        y1=ty+100
        y2=ty+dxz*tan(theta)
        if(y2<=y1):
            # exit()
            global debug1
            if(debug1):
                print(self.V)
                debug1=False
            return (tx,ty,tz),True
        if(y>=y1):
            return (tx,y1,tz),False
        x_=x+100*dx/dxz
        z_=z+100*dz/dxz
        return (x_,y1,z_),False
        
    def Hit(self,target,dt=1e-2):#判断是否击中目标
        x,y,z=self.xyz-self.v*dt
        vx,vy,vz=self.v
        V=self.V
        tx,ty,tz=target
        dx,dy,dz=x-tx,y-ty,z-tz
        tm=-(vx*dx+vy*dy+vz*dz)/(V*V)
        tm=min(max(0,tm),dt)
        Lmin=sqrt(V*V*tm*tm+2*(vx*dx+vy*dy+vz*dz)*tm+(dx*dx+dy*dy+dz*dz))
        global debug
        debug=min(Lmin,debug)
        if(Lmin<3):
            return True
        return False
    
    def Intercepted(self,bullet=Bullet(),dt=1e-2):#判断是否被子弹拦截 以导弹为静止参考系 判断子弹在间隔时间内走过的线段是否经过圆柱
        r=self.D/2
        bv=bullet.v
        bxyz2=bullet.xyz
        bxyz1=bxyz2-bv*dt
        xyz2=self.xyz
        xyz1=xyz2-self.v*dt
        Dxyz1,Dxyz2=bxyz1-xyz1,bxyz2-xyz2
        D1,D2=np.matmul(self.L01(),Dxyz1.T).T,np.matmul(self.L01(),Dxyz2.T).T
        x1,y1,z1=D1
        x2,y2,z2=D2
        Dx,Dy,Dz=x2-x1,y2-y1,z2-z1
        Delta=(z1*Dz+y1*Dy)**2-(z1**2+y1**2-r**2)*(Dz**2+Dy**2)
        if(Delta<0):
            return False
        t1=(-(z1*Dz+y1*Dy)-Delta**0.5)/(Dz**2+Dy**2)
        t2=(-(z1*Dz+y1*Dy)+Delta**0.5)/(Dz**2+Dy**2)
        if((t1<0 and t2<0) or (t1>1 and t2>1)):
            return False
        t1=max(t1,0)
        t2=min(t2,1)
        x_t1=Dx*t1+x1
        x_t2=Dx*t2+x1
        x_min=min(x_t1,x_t2)
        x_max=max(x_t1,x_t2)
        if(x_min>self.L/2 or x_max<-self.L/2):
            return False
        self.able=False
        return True
    
    def Alpha(self,DL):
        k1,b1=0.02,2.25
        k2,b2=0.014,5.5
        L1,L2,L3=564,715,1020
        #pi/16
        if(DL<L1):
            alpha=pi/min(k1*DL+b1,15.65)
        elif DL<L2:
            alpha=pi/min(k2*DL+b2,15.65)
        elif DL<L3:
            alpha=pi/15.65
        else:
            alpha=pi/16
        return alpha  

    def Guide(self,target=Gun(),dt=1e-2):
        DL=self.Len(self.xyz-target.xyz)
        T=min(DL/self.V,10)
        target_mid,final=self.target_mid(target=target.xyz+target.v*T)
        self.Rotate(target_mid,dt=dt,final_target=final)
        self.Move(dt=dt)

import matplotlib.pyplot as plt

import random
def rand3(l,r,low,high):
    return (random.random()*(r-l)+l,random.random()*(high-low)+low,random.random()*(r-l)+l)

if __name__=='__main__':
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    x=[]
    y=[]
    z=[]
    thetas=[]
    times=[]

    start=rand3(0,2500,5,50)
    target=rand3(0,2500,5,50)
    # start,target=[(1019.1034346281332, 11.571481297260908, 1717.4659850135467), (1169.6228752441257, 7.9780071606846885, 764.2027772355302)]
    tank=Gun(start=target,v=(15/1.414,0,15/1.414))
    print("{},{}".format(start,target))
    Dxz=(target[0]-start[0],target[2]-start[2])
    DL=(Dxz[0]**2+Dxz[1]**2)**0.5
    print(DL)
    phi=acos(Dxz[0]/DL)
    if(Dxz[1]!=0):
        phi=-phi*Dxz[1]/abs(Dxz[1])
    missile=Missile(xyz=start,theta=pi/4,P=300,phi=phi,gamma=0,v=(10*Dxz[0]/DL,10,10*Dxz[1]/DL))
    
    tot_T=0.
    for i in range(40):
        # print("xyz:{} v:{} theta:{} P:{}".format(missile.xyz,missile.v,missile.angle2['theta'],missile.P))
        for t in range(FPS):
            tot_T+=1/FPS
            times.append(tot_T)
            tank.Move(dt=1/FPS)
            missile.Guide(target=tank,dt=1/FPS)
            thetas.append(missile.angle1['theta'])
            x.append(missile.xyz[0])
            y.append(missile.xyz[1])
            z.append(missile.xyz[2])
            if(missile.Hit(tank.xyz) or missile.xyz[1]<0):
                break
        if(missile.Hit(tank.xyz) or missile.xyz[1]<0):
            break
    print("time:{} V:{} min:{}".format(tot_T,missile.V,debug))
    x=np.array(x)
    y=np.array(y)
    z=np.array(z)
    ax.plot(x,z,y)
    ax.set_zlabel('Y')
    ax.set_ylabel('Z')
    ax.set_xlabel('X')
    ax.scatter(target[0], target[2], target[1], c='r', marker='^')
    ax.scatter(tank.xyz[0], tank.xyz[2], tank.xyz[1], c='purple', marker='^')
    ax.scatter(start[0],start[2],start[1],c='g',marker='o')
    plt.show()
    # plt.plot(times,thetas)
    # plt.show()
    