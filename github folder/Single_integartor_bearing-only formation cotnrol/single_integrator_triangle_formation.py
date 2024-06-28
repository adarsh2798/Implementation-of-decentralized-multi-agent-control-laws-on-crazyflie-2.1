

########################################################
##### This code works perfect. Here i use commander class's world velocity function. Earlier i used rotation matrix to
####### convert control law's world velocity to body. But i think that velocity was not perfect.
###### directly sending world velocity to crazy works perfect as in this code.
####### NOTE that in test func(), after 0.5m posiiton, giving a long
###### sleep of say 3s causes crazy to land , coz crazy lands if it doset receive any control signal for 
###### more than some time.
####################################################

import time
import csv
import datetime
import numpy as np
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
import logging
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from scipy.spatial.transform import Rotation

import matplotlib.pyplot as plt


URI1='radio://0/80/2M/E7E7E7E701'
URI2='radio://0/50/2M/E7E7E7E704'
URI3='radio://0/80/2M/E7E7E7E703'
uris={URI1,URI2,URI3}

g=np.array([[0],[0],[0]])
pos_1=np.array([[0],[0],[0]])
pos_2=np.array([[0],[0],[0]])
pos_3=np.array([[0],[0],[0]])
pos_1_x_history=[]
pos_1_y_history=[]
pos_1_z_history=[]
pos_2_x_history=[]
pos_2_y_history=[]
pos_2_z_history=[]
pos_3_x_history=[]
pos_3_y_history=[]
pos_3_z_history=[]
R_1=np.zeros((3,3))
R_2=np.zeros((3,3))
R_3=np.zeros((3,3))
roll=0
pitch=0
yaw=0


c1=0
c2=0
c3=0

def get_R(roll,pitch,yaw):
     # Create a Rotation object
     roll_rad=np.radians(roll)
     pitch_rad=np.radians(pitch)
     yaw_rad=np.radians(yaw)
     R_z=np.array([[np.cos(yaw_rad),np.sin(yaw_rad),0],[-np.sin(yaw_rad),np.cos(yaw_rad),0],[0,0,1]])
     R_y=np.array([[np.cos(pitch_rad),0,-np.sin(pitch_rad)],[0,1,0],[np.sin(pitch_rad),0,np.cos(pitch_rad)]])
     R_x=np.array([[1,0,0],[0,np.cos(roll_rad),np.sin(roll_rad)],[0,-np.sin(roll_rad),np.cos(roll_rad)]])
     r = Rotation.from_euler('zyx', [yaw_rad, pitch_rad, roll_rad], degrees=False)

     # Get the 3D rotation matrix
     R = r.as_matrix()
     #R=np.matmul(R_x,np.matmul(R_y,R_z))
     
     return R


def log_callback(timestamp,data,logconf):
    
   
       global R_1,R_2,pos_1,pos_2,pos_3,pos_1_x_history,pos_1_y_history,pos_1_z_history,pos_2_x_history,pos_2_y_history,pos_2_z_history
       global pos_3_x_history,pos_3_y_history,pos_3_z_history
       global roll,pitch,yaw
       pos_x=data['stateEstimate.x']
       pos_y=data['stateEstimate.y']
       pos_z=data['stateEstimate.z']
       
       roll=data['stateEstimate.roll']
       pitch=data['stateEstimate.pitch']
       yaw=data['stateEstimate.yaw']
       
       cf_id=logconf.uri[-2:]
       
       if(int(cf_id[-1])==1):
            R_1=get_R(roll,pitch,yaw)
            pos_1=np.array([[pos_x],[pos_y],[pos_z]])
            pos_1_x_history.append(pos_x)
            pos_1_y_history.append(pos_y)
            pos_1_z_history.append(pos_z)
            
       elif(int(cf_id[-1])==4) :
            R_2=get_R(roll,pitch,yaw)
            pos_2=np.array([[pos_x],[pos_y],[pos_z]])
            pos_2_x_history.append(pos_x)
            pos_2_y_history.append(pos_z)
            pos_2_z_history.append(pos_z)
       else:
            R_3=get_R(roll,pitch,yaw)
            pos_3=np.array([[pos_x],[pos_y],[pos_z]])
            pos_3_x_history.append(pos_x)
            pos_3_y_history.append(pos_z)
            pos_3_z_history.append(pos_z)


def get_data(scf):
    logconf = LogConfig(name='Position', period_in_ms=10)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('stateEstimate.roll','float')
    logconf.add_variable('stateEstimate.pitch','float')
    logconf.add_variable('stateEstimate.yaw','float')
    logconf.uri=scf.cf.link_uri
    scf.cf.log.add_config(logconf)
    
    logconf.data_received_cb.add_callback(log_callback)
    
    logconf.start()
    


def plot():
    print("HERE")
    global pos_1_x_history,pos_1_y_history,pos_2_x_history,pos_2_y_history,pos_3_x_history,pos_3_y_history
    global c1,c2,c3
    

    plt.plot(pos_1_x_history[c1:], pos_1_z_history[c1:],'blue')
    plt.plot(pos_2_x_history[c1:], pos_2_z_history[c1:],'blue')
    plt.plot(pos_3_x_history[c1:], pos_3_z_history[c1:],'blue')


    
    # Mark the starting point
    plt.scatter(pos_1_x_history[c1], pos_1_z_history[c1], color='green', marker='o', label='Start')

    # Mark the ending point
    plt.scatter(pos_1_x_history[-1], pos_1_z_history[-1], color='red', marker='o', label='End')

    # Mark the starting point
    plt.scatter(pos_2_x_history[c1], pos_2_z_history[c1], color='green', marker='o', label='Start')

    # Mark the ending point
    plt.scatter(pos_2_x_history[-1], pos_2_z_history[-1], color='red', marker='o', label='End')

    # Mark the starting point
    plt.scatter(pos_3_x_history[c1], pos_3_z_history[c1], color='green', marker='o', label='Start')

    # Mark the ending point
    plt.scatter(pos_3_x_history[-1], pos_3_z_history[-1], color='red', marker='o', label='End')

    plt.plot([pos_1_x_history[c1],pos_2_x_history[c1]],[pos_1_z_history[c1],pos_2_z_history[c1]],'green')
    plt.plot([pos_2_x_history[c1],pos_3_x_history[c1]],[pos_2_z_history[c1],pos_3_z_history[c1]],'green')
    #plt.plot([pos_3_x_history[c1],pos_1_x_history[c1]],[pos_3_z_history[c1],pos_1_z_history[c1]],'green')

    plt.plot([pos_1_x_history[-1],pos_2_x_history[-1]],[pos_1_z_history[-1],pos_2_z_history[-1]],'red')
    plt.plot([pos_2_x_history[-1],pos_3_x_history[-1]],[pos_2_z_history[-1],pos_3_z_history[-1]],'red')
    plt.plot([pos_3_x_history[-1],pos_1_x_history[-1]],[pos_3_z_history[-1],pos_1_z_history[-1]],'red')
    
    plt.savefig('plot.png')
    # Show the plot
    plt.show()


def vel_clipper(x,thresh):
    if abs(x)>thresh:
        x=np.sign(x)*thresh
    return x 

def skew_symm_op(x):

    x1=x[0,0]
    x2=x[1,0]
    x3=x[2,0]
    S=np.array([[0,-x3,x2],[x3,0,-x1],[-x2,x1,0]])
    return S
def test(scf):
      print("yello")
      global R_1,R_2,R_3,pos_1,pos_2,pos_3,roll,pitch,yaw,pos_1_x_history,pos_2_x_history,pos_3_x_history
      global c1,c2,c3
      v=np.zeros((3,1))
      cf=scf.cf
      cf_id=cf.link_uri[-2:]
      cf_id=int(cf_id[-1])

      
      if cf_id==1:
          ss=time.time()
          while((time.time()-ss)<3):
            cf.commander.send_position_setpoint(pos_1[0,0],pos_1[1,0],1,0)
            #cf.commander.send_velocity_world_setpoint(0,0,0.2,0)
          
          c1=len(pos_1_x_history)
      if cf_id==4:
          ss=time.time()
          while((time.time()-ss)<3):
            cf.commander.send_position_setpoint(pos_2[0,0],pos_2[1,0],1,0)
            #cf.commander.send_velocity_world_setpoint(0,0,0.2,0)
          c2=len(pos_2_x_history)
      if cf_id==3:
          ss=time.time()
          while((time.time()-ss)<3):
            cf.commander.send_position_setpoint(pos_3[0,0],pos_3[1,0],1,0)
            #cf.commander.send_velocity_world_setpoint(0,0,0.2,0)
          c3=len(pos_3_x_history)
      
      #time.sleep(1)### dont make it too long
      s=time.time()

      # required bearings
      g12=(pos_2-pos_1)/np.linalg.norm(pos_2-pos_1)
      g_req12=np.array([[1/2],[0],[np.sqrt(3)/2]])

      g23=(pos_3-pos_2)/np.linalg.norm(pos_3-pos_2)
      g_req23=np.array([[1/2],[0],[-np.sqrt(3)/2]])

      g13=(pos_3-pos_1)/np.linalg.norm(pos_3-pos_1)
      g_req13=np.array([[1],[0],[0]])
  
      q1=np.linalg.norm(g12-g_req12)
      q2=np.linalg.norm(g23-g_req23)
      q3=np.linalg.norm(g13-g_req13)

      q=np.array([q1,q2,q3])
      
      while((time.time()-s)<8):
        
        # control law calculation
        if cf_id==1:
            g12=(pos_2-pos_1)/np.linalg.norm(pos_2-pos_1)
            P_12=np.eye(3)-np.matmul(g12,g12.T)
            g_req12=np.array([[1/2],[0],[np.sqrt(3)/2]])
            v12=-np.matmul(P_12,g_req12)
            
            g13=(pos_3-pos_1)/np.linalg.norm(pos_3-pos_1)
            P_13=np.eye(3)-np.matmul(g13,g13.T)
            g_req13=np.array([[1],[0],[0]])
            v13=-np.matmul(P_13,g_req13)

            v=v12+v13

        if cf_id==4:
            g21=-(pos_2-pos_1)/np.linalg.norm(pos_2-pos_1)
            P_21=np.eye(3)-np.matmul(g21,g21.T)
            g_req21=-np.array([[1/2],[0],[np.sqrt(3)/2]])
            v21=-np.matmul(P_21,g_req21)
            
            g23=(pos_3-pos_2)/np.linalg.norm(pos_3-pos_2)
            P_23=np.eye(3)-np.matmul(g23,g23.T)
            g_req23=np.array([[1/2],[0],[-np.sqrt(3)/2]])
            v23=-np.matmul(P_23,g_req23)

            v=v21+v23
        if cf_id==3:
            g32=(pos_2-pos_3)/np.linalg.norm(pos_2-pos_3)
            P_32=np.eye(3)-np.matmul(g32,g32.T)
            g_req32=np.array([[-1/2],[0],[np.sqrt(3)/np.sqrt(2)]])
            v32=-np.matmul(P_32,g_req32)
            
            g31=-(pos_3-pos_1)/np.linalg.norm(pos_3-pos_1)
            P_31=np.eye(3)-np.matmul(g31,g31.T)
            g_req31=np.array([[-1],[0],[0]])
            v31=-np.matmul(P_31,g_req31)

            v=v32+v31
            
        print(np.linalg.norm(q))
        cf.commander.send_velocity_world_setpoint(vel_clipper(v[0,0],0.1),vel_clipper(v[1,0],0.1),vel_clipper(v[2,0],0.1),0)
    
      s1=time.time()


      c=(pos_1+pos_2+pos_3)/3
      S=skew_symm_op(np.array([[1],[0],[0]]))
      w=0.5
      ## irrelevant to formation control. Here, a velocify os given to each crazyfli esuch that whole formation
      ## rotates. The velocity is not in any way related to any decentralized formation control.
      while(time.time()-s1<-10):
            if cf_id==1:
                v=w*np.matmul(S,pos_1-c)
            if cf_id==2:
                v=w*np.matmul(S,pos_2-c)
            if cf_id==3:
                v=w*np.matmul(S,pos_3-c)
            cf.commander.send_velocity_world_setpoint(vel_clipper(v[0,0],0.1),vel_clipper(v[1,0],0.1),vel_clipper(v[2,0],0.1),0)

          

      
      scf.MotionCommander.land()
      print("DONE!!")
      
      

          
          
    

if __name__=="__main__":
    cflib.crtp.init_drivers()
    factory=CachedCfFactory(rw_cache="./cache")
    with Swarm(uris,factory=factory) as swarm:
     
        swarm.parallel(get_data)
        time.sleep(2)
        swarm.parallel(test)
    plot()
   




