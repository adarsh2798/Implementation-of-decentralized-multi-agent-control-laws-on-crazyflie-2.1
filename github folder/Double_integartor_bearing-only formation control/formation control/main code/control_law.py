import numpy as np
from utils import dist,get_R,ema_filt


r=0.5 ### radius of leader circumcircle
fc=1
def get_acceleration(agent,type_foll):
    
    k_p=0.1
    k_v=100

    if type_foll=="ff":
      i=0
      term_1=np.zeros((2,1))
      term_2=np.zeros((2,1))
      for n in agent.neighbors:
        
        curr_bearing=(n.position-agent.position)/np.linalg.norm(n.position-agent.position)
        P=np.eye(2)-np.matmul(curr_bearing,curr_bearing.T)

        req_bearing=agent.desired_bearings[i]

        term_1+=np.matmul(P,req_bearing)
        
        smooth_rel_vel=ema_filt(fc,n.velocity-agent.velocity,n.prev_velocity-agent.prev_velocity)
        smooth_rel_vel=n.velocity-agent.velocity
        dot_curr_bearing=np.matmul(P,smooth_rel_vel)/np.linalg.norm(n.position-agent.position)

        term_2+=dot_curr_bearing

        i+=1
      
      ##### pseudo-neighbor of 1st follower################
      leader=agent.neighbors[0]

      d=dist([agent.position[0,0],agent.position[1,0]],[leader.position[0,0],leader.position[1,0]])

      angle=np.arcsin(r/d)

      R=get_R(angle*180/np.pi)

      curr_bearing=(leader.position-agent.position)/np.linalg.norm(leader.position-agent.position)
      curr_bearing_pseudo=np.matmul(R,curr_bearing)

      req_bearing_pseudo=agent.desired_bearings[i]

      P=np.eye(2)-np.matmul(curr_bearing_pseudo,curr_bearing_pseudo.T)
      term_1+=np.matmul(P,req_bearing_pseudo)
      
      smooth_rel_vel=ema_filt(fc,leader.velocity-agent.velocity,leader.prev_velocity-agent.prev_velocity)
      smooth_rel_vel=n.velocity-agent.velocity
      dot_curr_bearing_pseudo=np.matmul(P,smooth_rel_vel)/np.sqrt(d**2-r**2)
      

      term_2+=dot_curr_bearing_pseudo
    











      accel=(-k_p*term_1)+(k_v*term_2)

    if type_foll=="f":
      i=0
      term_1=np.zeros((2,1))
      term_2=np.zeros((2,1))
      for n in agent.neighbors:
        
        curr_bearing=(n.position-agent.position)/np.linalg.norm(n.position-agent.position)
        P=np.eye(2)-np.matmul(curr_bearing,curr_bearing.T)

        req_bearing=agent.desired_bearings[i]

        term_1+=np.matmul(P,req_bearing)
         
        smooth_rel_vel=ema_filt(fc,n.velocity-agent.velocity,n.prev_velocity-agent.prev_velocity)
        dot_curr_bearing=np.matmul(P,smooth_rel_vel)/np.linalg.norm(n.position-agent.position)

        term_2+=dot_curr_bearing

        i+=1

      accel=(-k_p*term_1)+(k_v*term_2)
       
    return accel 


"""

from MAS_network import MAS
class agent(object):


    def __init__(self,cf_id):

        self.cf_id=cf_id
        self.position=None
        self.velocity=None
       
       

  


class follower(agent):
     def __init__(self, cf_id):
        super().__init__(cf_id)
        self.neighbors = None  # Additional attribute for followers
        self.desired_bearings=None
     
     def assign_neighbors(self,neighbors):

        self.neighbors=neighbors
     def assign_desired_bearings(self,desired_bearings):
         self.desired_bearings=desired_bearings

class leader(agent):
     def __init__(self, cf_id):
        super().__init__(cf_id)



adjacency_dict={2: [1], 3: [1, 2], 4: [1, 3]}
r=0.2 #change this r in control_law.py file also
d_req=0.7 # choose d_req between [0.7,1.4]
angle=np.arcsin(r/d_req)
R=get_R(angle*180/np.pi)

target_formation={2:[np.array([[-1],[0]]),np.matmul(R,np.array([[-1],[0]]))], 
                  3:[np.array([[-1/np.sqrt(2)],[-1/np.sqrt(2)]]),np.array([[0],[-1]])],
                  4:[np.array([[0],[-1]]),np.array([[1],[0]])]}

mas=MAS(adjacency_dict,1,target_formation)
mas.create_network()
agents=mas.get_agents()

pos1=np.array([[1],[2]])
pos2=np.array([[2],[2]])
pos3=np.array([[3],[-2]])
pos4=np.array([[1],[3]])
pos=[pos1,pos2,pos3,pos4]
vel1=np.array([[0.1],[0.2]])
vel2=np.array([[-0.1],[0.2]])
vel3=np.array([[0.1],[0.3]])
vel4=np.array([[0.05],[0.4]])
vel=[vel1,vel2,vel3,vel4]
i=0
for a in agents:
   a.position=pos[i]
   a.velocity=vel[i]
   i+=1

print(get_acceleration(agents[3],"f")) 


"""