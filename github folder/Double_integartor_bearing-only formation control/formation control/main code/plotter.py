import matplotlib.pyplot as plt
from utils import find_agent_by_cf_id,dist,get_R
import csv
import numpy as np
import time
import math
agent_positions_x = {}
agent_positions_y = {}
d=[]
bearing_err=[]
start=0
def collect_d(d_val):
   d.append(d_val)
def collect_agent_position(agent_id,position):
    if agent_id not in agent_positions_x:
        start=time.time()
        agent_positions_x[agent_id] = []
        agent_positions_y[agent_id] = []
    agent_positions_x[agent_id].append(position[0])
    agent_positions_y[agent_id].append(position[1])

def find_bearing_error(target_formation,adjacency_dict,r,first_follower_cf_id):
   min_length=math.inf
   for a_p in agent_positions_x:
      min_length=min(len(agent_positions_x[a_p]),min_length)

   for i in range(min_length):
      err=0
      for t_f in target_formation:
         pos_agent=np.array([[agent_positions_x[t_f][i]],[agent_positions_y[t_f][i]]])
         j=0
         for n in adjacency_dict[t_f]:
            pos_neigh=np.array([[agent_positions_x[n][i]],[agent_positions_y[n][i]]])
            curr_bear=(pos_neigh-pos_agent)/np.linalg.norm(pos_neigh-pos_agent)
            req_bear=target_formation[t_f][j]
            err+=np.linalg.norm(curr_bear-req_bear)
            j+=1
         if t_f==first_follower_cf_id:
            angle=np.arcsin(r/dist([pos_agent[0,0],pos_agent[1,0]],[pos_neigh[0,0],pos_neigh[1,0]]))
            R=get_R(angle*180/np.pi)
            curr_bear_pseudo=np.matmul(R,curr_bear)
            req_bear_pseudo=target_formation[t_f][j]
            err+=np.linalg.norm(curr_bear_pseudo-req_bear_pseudo)
      bearing_err.append(err)

      
   


def plot(leader_agents,follower_agents,first_follower_cf_id,adjacency_dict,d_req):
    elap=time.time()-start
    global agent_positions_x,agent_positions_y

    leader_color='red'
    follower_color='blue'
    first_follower_color='green'
    leader_list=[]
    follower_list=[]
    for l in leader_agents:
        leader_list.append(l.cf_id)
    for f in follower_agents:
        follower_list.append(f.cf_id)
    agents_list=leader_list+follower_list

    plt.figure() ## first figure of agents_plot
    for a in agents_list:
          if a not in agent_positions_x:
              continue
          pos_x=agent_positions_x[a]
          pos_y=agent_positions_y[a]

          if(a in leader_list):
              plt.plot(pos_x,pos_y,leader_color)
          if(a in follower_list):
              if a==first_follower_cf_id:
                  plt.plot(pos_x,pos_y,first_follower_color)
              else:
                  
                plt.plot(pos_x,pos_y,follower_color)
    
    for f in follower_list:
        
        for n in adjacency_dict[f]:
           
           
           plt.plot([agent_positions_x[f][0],agent_positions_x[n][0]],
                 [agent_positions_y[f][0],agent_positions_y[n][0]],'--k')
    
    for f in follower_list:
        
        for n in adjacency_dict[f]:
           
           
           plt.plot([agent_positions_x[f][-1],agent_positions_x[n][-1]],
                 [agent_positions_y[f][-1],agent_positions_y[n][-1]],'k')
    for a in agents_list:
     if a in leader_list:
      #plt.scatter([agent_positions_x[a][0]],[agent_positions_y[a][0]],s=20,color=leader_color)
      plt.scatter([agent_positions_x[a][0]], [agent_positions_y[a][0]], marker='o', facecolors='none', edgecolors=leader_color,s=40)
      plt.scatter([agent_positions_x[a][-1]], [agent_positions_y[a][-1]], marker='o', edgecolors=leader_color,s=40)
 
     if a in follower_list:
      if a==first_follower_cf_id:
       #plt.scatter([agent_positions_x[a][0]],[agent_positions_y[a][0]],s=20,color=first_follower_color)
       plt.scatter([agent_positions_x[a][0]], [agent_positions_y[a][0]], marker='o', facecolors='none', edgecolors=first_follower_color,s=40)
       plt.scatter([agent_positions_x[a][-1]], [agent_positions_y[a][-1]], marker='o', edgecolors=first_follower_color,s=40)
      else:
       #plt.scatter([agent_positions_x[a][0]],[agent_positions_y[a][0]],s=20,color=follower_color)
       plt.scatter([agent_positions_x[a][0]], [agent_positions_y[a][0]], marker='o', facecolors='none', edgecolors=follower_color,s=40)
       plt.scatter([agent_positions_x[a][-1]], [agent_positions_y[a][-1]], marker='o', edgecolors=follower_color,s=40)
         

        
    plt.figure() #2nd figure for dist plot 
    step=elap/(len(d)-1)
    tt=np.arange(0,elap+step,step)
    d_req_arr=[d_req]*len(d)

    plt.plot(tt,d)
    plt.plot(tt,d_req_arr,"orange")
   
    plt.figure()
    step=elap/(len(bearing_err)-1)
    tt=np.arange(0,elap+step,step)
    plt.plot(tt,bearing_err)

    plt.show()




def save_agent_positions_to_csv(filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header row
        writer.writerow(['Agent_ID', 'X_Positions', 'Y_Positions'])
        # Write data rows
        for agent_id, x_positions in agent_positions_x.items():
            y_positions = agent_positions_y[agent_id]
            writer.writerow([agent_id, x_positions, y_positions])




