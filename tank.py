import numpy as np
from math import sin,cos,pi,sqrt,asin,acos,exp,tan
class CFG:
    max_dis=1000
    shoot_dis=500
    bullet_vol=1800
    fire_freq=1000
class Bullet:
    def __init__(self,start=(500.,0,1000.),v=(707.,707.,0.),id=0):
        self.xyz=np.array(start,dtype=float)
        self.v=np.array(v,dtype=float)
        self.dis=0.
        v=self.v/self.Len(self.v)
        self.theta=asin(v[1])
        self.phi=acos(v[0]/self.Len((v[0],0,v[2])))
        if(v[2]!=0):
            self.phi=-self.phi*v[2]/abs(v[2])
        self.able=True
        self.id=id

    def Move(self,dt):
        self.xyz+=self.v*dt
        self.dis+=self.Len(self.v*dt)
        if(self.dis>CFG.max_dis):
            self.able=False
        
    def Len(self,v):
        return sqrt(v[0]**2+v[1]**2+v[2]**2)
    
class Gun:
    def __init__(self,start=(500.,0,1000.),id=0,v=(0,0,0)):
        self.xyz=np.array(start,dtype=float)
        self.tot_T=0
        self.v=np.array(v,dtype=float)
        self.look=np.array((1,0,0),dtype=float)
        self.CD=60/CFG.fire_freq
        self.cd=0.
        self.bullet_vol=CFG.bullet_vol
        self.id=id
        self.able=True
        self.omega=2*pi
        self.onfire=False
        self.exploded=False
        self.Update()

    def Update(self,dt=0):
        self.tot_T+=dt
        look=self.look
        self.theta=asin(look[1])
        self.phi=acos(look[0]/self.Len((look[0],0,look[2])))
        if(look[2]!=0):
            self.phi=self.phi*look[2]/abs(look[2])

        self.cd=max(0.,self.cd-dt)
    
    def SetV(self,v=(0.,0.,0.)):
        self.v=np.array(v,dtype=float)

    def Modify(self,dxyz=(0.,0.,0.)):
        dxyz=np.array(dxyz,dtype=float)
        self.xyz+=dxyz
    
    def Move(self,dt=0):
        self.xyz+=self.v*dt

    def ModifyTo(self,xyz):
        xyz=np.array(xyz,dtype=float)
        self.xyz=xyz

    def Len(self,v):
        return sqrt(v[0]**2+v[1]**2+v[2]**2)

    def Rotate(self,target,dt):
        target=np.array(target,dtype=float)
        Dxyz=target-self.xyz
        L=self.Len(Dxyz)
        target_look=Dxyz/L
        Delta_look=target_look-self.look
        DL=self.Len(Delta_look)
        theta=acos((2-DL*DL)/2)
        if(theta<=self.omega*dt):
            self.look=target_look
        else:
            dtheta=self.omega*dt
            alpha=(pi-theta)/2
            beta=pi-alpha-dtheta
            dL=sin(dtheta)/sin(beta)
            self.look+=Delta_look*dL
        self.Update()
    
    def Fire(self,target):
        target=np.array(target,dtype=float)
        Dxyz=target-self.xyz
        L=self.Len(Dxyz)
        if(L>CFG.shoot_dis):
            self.onfire=False
            return None
        self.onfire=True
        if(self.cd>0.):
            return None
        self.cd+=self.CD
        bullet=Bullet(self.xyz+self.look*3,self.look*self.bullet_vol+self.v)
        return bullet
    
    def Intercept(self,target,dt=0):
        self.Update(dt=dt)
        # print(self.v)
        Dv=target.v-self.v
        Dxyz=target.xyz-self.xyz
        L2=(Dxyz*Dxyz).sum()
        A=(Dv*Dxyz).sum()
        A2=A*A
        Dv2=(Dv*Dv).sum()
        Vb2=self.bullet_vol**2
        Delta=A2+(Vb2-Dv2)*L2
        t=(A+sqrt(Delta))/(Vb2-Dv2)
        R=self.bullet_vol*t
        if(R>CFG.max_dis):
            self.onfire=False
            return None
        txyz=self.xyz+Dxyz+Dv*dt
        self.Rotate(target=txyz,dt=dt)
        return self.Fire(target=txyz)