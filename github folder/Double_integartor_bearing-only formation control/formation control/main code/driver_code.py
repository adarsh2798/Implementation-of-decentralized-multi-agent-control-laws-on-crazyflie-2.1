


#in this copde to go from world to body use R_i.T


import time
import numpy as np
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
import logging
import matplotlib.pyplot as plt
import math



from MAS_network import MAS
from utils import find_agent_by_cf_id,dist,saturate,get_R,convert_accel2attitude,ema_filt
from plotter import collect_agent_position,plot,find_bearing_error,collect_d
from control_law import get_acceleration

URI1='radio://0/80/2M/E7E7E7E701'
URI2='radio://0/80/2M/E7E7E7E702'
URI3='radio://1/50/2M/E7E7E7E703'
URI4='radio://1/50/2M/E7E7E7E704'
URI5='radio://2/20/2M/E7E7E7E705'
URI6='radio://2/20/2M/E7E7E7E706'



URI1='radio://0/80/2M/E7E7E7E701'
URI2='radio://2/80/2M/E7E7E7E702'
URI3='radio://1/80/2M/E7E7E7E703'
URI4='radio://1/50/2M/E7E7E7E704'
URI5='radio://1/20/2M/E7E7E7E705'
URI6='radio://0/20/2M/E7E7E7E706'

r=0.5 #change this r in control_law.py file also
d_req=0.8 # choose d_req between [0.7,1.4]
angle=np.arcsin(r/d_req)
R=get_R(angle*180/np.pi)


adjacency_dict={6: [1], 3: [1, 6], 4: [1, 3]}
#adjacency_dict={2: [4], 3: [4, 2], 5: [4, 3]}
adjacency_dict={6: [1]}

target_formation={6:[np.array([[-1],[0]]),np.matmul(R,np.array([[-1],[0]]))], 
                  3:[np.array([[-1/np.sqrt(2)],[-1/np.sqrt(2)]]),np.array([[0],[-1]])],
                  4:[np.array([[0],[-1]]),np.array([[1],[0]])]}

target_formation={6:[np.array([[-1],[0]]),np.matmul(R,np.array([[-1],[0]]))]}

plane='xy'
leader_cf_id=1
first_follower_cf_id=6
leader_velocity=[0.0,0.0]

leaders_z=0.5
followers_z=0.8
formation_z=leaders_z
MAS_created=False


uris={URI6,URI3,URI1,URI4}
uris={URI1,URI6}







######## failsafe to avoid crazyflie flying out of range ########

def cf_out_of_bounds(p_x, p_y, p_z):
    x_min = -0.88
    x_max = 1.47

    y_min = -1.1
    y_max = 0.95

    z_min = 0
    z_max = 1.5

    if p_x < x_min or p_x > x_max:
        return True
    if p_y < y_min or p_y > y_max:
        return True
    if p_z < z_min or p_z > z_max:
        return True
    return False
################################################################

def log_callback(timestamp,data,logconf):
    
       global leaders,followers,leaders_id_loc,followers_id_loc,MAS_created
       global leaders_list,followers_list
       
       global roll,pitch ####temp additoon
       
       pos_x=data['stateEstimate.x']
       pos_y=data['stateEstimate.y']
       pos_z=data['stateEstimate.z']

       vel_x=data['stateEstimate.vx']
       vel_y=data['stateEstimate.vy']
       vel_z=data['stateEstimate.vz']


       pos=None
       vel=None
       if plane=='xy':
          pos=np.array([[pos_x],[pos_y]])
          vel=np.array([[vel_x],[vel_y]])
       if plane=='xz':
             pos=np.array([[pos_x],[pos_z]])
             vel=np.array([[vel_x],[vel_z]])
           
       cf_id=logconf.uri[-1:]
       cf_id=int(cf_id[-1])
      
       
       
        
       if MAS_created:
            
            a=find_agent_by_cf_id(agents,cf_id)
            a.position=pos
            if a.velocity is not None:
               a.prev_velocity=a.velocity
               
            a.velocity=vel
            
            pos=[pos[0,0],pos[1,0]]
            collect_agent_position(cf_id,pos)


def get_data(scf):
    
    type_data='FP16'
    logconf = LogConfig(name='Position', period_in_ms=10)
    logconf.add_variable('stateEstimate.x', type_data)
    logconf.add_variable('stateEstimate.y', type_data)
    logconf.add_variable('stateEstimate.z', type_data)
    logconf.add_variable('stateEstimate.vx',type_data)
    logconf.add_variable('stateEstimate.vy',type_data)
    logconf.add_variable('stateEstimate.vz',type_data)

    ########temp addition#########
    logconf.add_variable('stateEstimate.roll',type_data)
    logconf.add_variable('stateEstimate.pitch',type_data)
    ##########################

    logconf.uri=scf.cf.link_uri
    scf.cf.log.add_config(logconf)
    
    logconf.data_received_cb.add_callback(log_callback)
    
    logconf.start()
    
    




def control_action(scf):
    

    global roll,roll_list,pitch,pitch_list,roll_ref_list,pitch_ref_list ###temp addition
    
    print("ffdd")
    cf=scf.cf
    cf_id=cf.link_uri[-2:]
    cf_id=int(cf_id[-1])
    
    if cf_id ==leader_cf_id:
        
        l=find_agent_by_cf_id(leader_agents,cf_id)
        
        start_1_l=time.time()
        
        while(time.time()-start_1_l<-4):
            cf.commander.send_position_setpoint(l.position[0,0],l.position[1,0],leaders_z,0)
        start_2_l=time.time()

        while(time.time()-start_2_l<-15):
          if plane=='xy':
           cf.commander.send_velocity_world_setpoint(leader_velocity[0],leader_velocity[1],0,0)
          if plane=='xz':
           cf.commander.send_velocity_world_setpoint(leader_velocity[0],0,leader_velocity[1],0)
        
    else:
        
        f=find_agent_by_cf_id(follower_agents,cf_id)
        start_1_f=time.time()
        
        while(time.time()-start_1_f<3):
          
              cf.commander.send_position_setpoint(f.position[0,0],f.position[1,0],followers_z,0)
            
        start_2_f=time.time()
        
        prev=0
        vel=np.array([[f.velocity[0,0]],[f.velocity[1,0]]])
        accel=np.zeros((2,1))
        while(time.time()-start_2_f<15):
          
          
          curr=time.time()
          dt=curr-prev
          if cf_id==first_follower_cf_id:
   
           accel=get_acceleration(f,"ff")
         
           collect_d(dist([f.position[0,0],f.position[1,0]],[leader_agents[0].position[0,0],leader_agents[0].position[1,0]]))
          else:
    
           accel=get_acceleration(f,"f")
         
          
          vel+=accel*dt
          
         
          if plane=='xy':
           
         
             if cf_out_of_bounds(f.position[0,0],f.position[1,0],f.position[2,0]):
          
               cf.commander.send_velocity_world_setpoint(0,0,0,0)
             else:
   
              #cf.commander.send_zdistance_setpoint(att_roll,att_pitch,0,followers_z)
              cf.commander.send_velocity_world_setpoint(saturate(vel[0,0],0.1),saturate(vel[1,0],0.1),0,0)
          if plane=='xz':
           #if cf_id==4:
            cf.commander.send_velocity_world_setpoint(saturate(vel[0,0],0.1),0,saturate(vel[1,0],0.1),0)

          prev=curr
           





        








      

          
          
    

if __name__=="__main__":
    cflib.crtp.init_drivers()
    factory=CachedCfFactory(rw_cache="./cache")
    with Swarm(uris,factory=factory) as swarm:
       #swarm.reset_estimators()
       swarm.parallel(get_data)
       time.sleep(2)
       
       mas=None
       
       mas=MAS(adjacency_dict,leader_cf_id,target_formation)
       mas.create_network()
       MAS_created=True
       follower_agents=mas.get_followers()
       leader_agents=mas.get_leaders()
       agents=mas.get_agents()
       leader_agents_sorted = sorted(leader_agents, key=lambda x: x.cf_id)
       follower_agents_sorted = sorted(follower_agents, key=lambda x: x.cf_id)
       ff=mas.get_followers()

       for f in ff:
         nn=[fnn.cf_id for fnn in f.neighbors]
         print(f.cf_id,nn)
     
        
       swarm.parallel(control_action)
   
    find_bearing_error(target_formation,adjacency_dict,r,first_follower_cf_id)
    plot(leader_agents,follower_agents,first_follower_cf_id,adjacency_dict,d_req)

    ani=get_animate_object()
    animate(ani)
        
           
       
     
  




