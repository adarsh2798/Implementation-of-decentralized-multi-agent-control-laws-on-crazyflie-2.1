


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



from MAS_network import MAS,agent
from utils import find_agent_by_cf_id,dist,get_leader_hull_centroid,saturate
from containment_control_algorithm import find_GLT_roots
from plotter import collect_agent_position,plot,save_agent_positions_to_csv

URI1='radio://0/80/2M/E7E7E7E701'
URI2='radio://0/80/2M/E7E7E7E702'
URI3='radio://1/50/2M/E7E7E7E703'
URI4='radio://1/50/2M/E7E7E7E704'
URI5='radio://2/20/2M/E7E7E7E705'
URI6='radio://2/20/2M/E7E7E7E706'


m=2
n=4
followers=[5,6]
leaders=[1,2,3,4]
followers_list=[5,6]
leaders_list=[1,2,3,4]
followers_id_loc=[]
leaders_id_loc=[]
initial_followers_z=0.8
leaders_z=0.5
formation_z=leaders_z
MAS_created=False


uris={URI1,URI2,URI3,URI4,URI5,URI6}
#uris={URI1,URI5}

def get_R(yaw):
     yaw=yaw*np.pi/180
     R= np.array([[np.cos(yaw), -np.sin(yaw)], 
                         [np.sin(yaw),  np.cos(yaw)]])
     return R



def log_callback(timestamp,data,logconf):
    
       global leaders,followers,leaders_id_loc,followers_id_loc,MAS_created
       global leaders_list,followers_list
       
       
       pos_x=data['stateEstimate.x']
       pos_y=data['stateEstimate.y']
       pos_z=data['stateEstimate.z']
    
       yaw=data['stateEstimate.yaw']
       pos=[pos_x,pos_y]
       cf_id=logconf.uri[-1:]
       cf_id=int(cf_id[-1])
       
       if cf_id in leaders:
           leaders.remove(cf_id)
           leaders_id_loc.append((cf_id,[pos_x,pos_y]))
       if cf_id in followers:
           followers.remove(cf_id)
           followers_id_loc.append((cf_id,[pos_x,pos_y]))
        
       if MAS_created:
            if cf_id in leaders_list:

                l=find_agent_by_cf_id(leader_agents,cf_id)

                pos=[pos_x,pos_y]
                l.position=pos
                
            if cf_id in followers_list:

                f=find_agent_by_cf_id(follower_agents,cf_id)

                pos=[pos_x,pos_y]
                f.position=pos
            collect_agent_position(cf_id,pos)


def get_data(scf):
    logconf = LogConfig(name='Position', period_in_ms=10)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('stateEstimate.yaw','float')
    logconf.uri=scf.cf.link_uri
    scf.cf.log.add_config(logconf)
    
    logconf.data_received_cb.add_callback(log_callback)
    
    logconf.start()
    
    


def control_action(scf):
    
    global leaders_list,followers_list,initial_followers_z,leaders_z,formation_z,n
    cf=scf.cf
    cf_id=cf.link_uri[-2:]
    cf_id=int(cf_id[-1])
    
    if cf_id in leaders_list:
        l=find_agent_by_cf_id(leader_agents,cf_id)
        start_1=time.time()
        
        while(time.time()-start_1<4):
            cf.commander.send_position_setpoint(l.position[0],l.position[1],leaders_z,0)
           
        rad=0.005
        c=[0.365,0]
       
        start_2=time.time()
     
        if cf_id==1:
            offset=-2.315
        if cf_id==4:
            offset=-0.82
        if cf_id==3:
            offset=0.82
        if cf_id==2:
            offset=2.315
        if offset<0:
            offset+=2*math.pi       
     
        radius=0.7
        while(time.time()-start_2<10):
            
            l.angle+=rad
            p_x=c[0] + (math.cos(l.angle + offset)) * radius
            p_y=c[1] + (math.sin(l.angle + offset)) * radius
        
            if dist(l.position,c)<0.058:
                cf.commander.send_velocity_world_setpoint(0,0,0,0)
                
            else:
             
                cf.commander.send_position_setpoint(p_x,p_y,leaders_z,0)
           
              



    if cf_id in followers_list :
      
     f=find_agent_by_cf_id(follower_agents,cf_id)
     start_1=time.time()
           
     while(time.time()-start_1<3):
           
            cf.commander.send_position_setpoint(f.position[0],f.position[1],initial_followers_z,0)
            x=3
            
     start3=time.time()
     while(time.time()-start3<10):
        neighbors=f.neighbors

        neighbor_pos=[]

        for n in neighbors:
            neighbor_pos.append([n.position[0],n.position[1]])

        candidate_loc=find_GLT_roots(neighbor_pos)
        currrent_loc=f.position
        go_to_loc=None
        min_dist=math.inf
        for c_l in candidate_loc:
            d=dist(c_l,currrent_loc)

            if d<min_dist:
                min_dist=d
                go_to_loc=c_l
        
       
            
       
        cf.commander.send_position_setpoint(go_to_loc[0],go_to_loc[1],initial_followers_z,0)
        








      

          
          
    

if __name__=="__main__":
    cflib.crtp.init_drivers()
    factory=CachedCfFactory(rw_cache="./cache")
    with Swarm(uris,factory=factory) as swarm:
     
       swarm.parallel(get_data)
       time.sleep(2)
       mas=None
       if len(leaders_id_loc)==n and len(followers_id_loc)==m:
           mas=MAS(m,n,leaders_id_loc,followers_id_loc)
       mas.create_MAS()
       MAS_created=True
       follower_agents=mas.get_followers()
       leader_agents=mas.get_leaders()
       leader_agents_sorted = sorted(leader_agents, key=lambda x: x.cf_id)
       follower_agents_sorted = sorted(follower_agents, key=lambda x: x.cf_id)

       #for ff in follower_agents:
       #  ff_n=ff.neighbors
       #  l=[ff_n_a.cf_id for ff_n_a in ff_n]
        
       #  print(ff.cf_id,"--->",l)
        
       swarm.parallel(control_action)
    plot(leader_agents,follower_agents)
    save_agent_positions_to_csv('agent_positions_data.csv')

    #ani=get_animate_object()
    #animate(ani)
        
           
       
     
  




