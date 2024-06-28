
from utils import find_agent_by_cf_id




class agent(object):


    def __init__(self,cf_id):

        self.cf_id=cf_id
        self.position=None
        self.velocity=None
        self.prev_velocity=None
       
       

  


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
        


class MAS():
    
    
 

    def __init__(self,adjacency_dict,leader_cf_id,target_formation):
        self.network=adjacency_dict
        self.target_formation=target_formation
        self.leader_cf_id=leader_cf_id
         
        self.leaders=[leader(leader_cf_id)]
        self.followers=[follower(key) for key in adjacency_dict]
        self.agents=self.leaders+self.followers


    def create_network(self):

        for f in self.followers:

            
            neighbors=[find_agent_by_cf_id(self.agents,n) for n in self.network[f.cf_id]]
            desired_bearings=[ d_b for d_b in self.target_formation[f.cf_id] ]
            f.assign_neighbors(neighbors)
            f.assign_desired_bearings(desired_bearings)

        

    def get_followers(self):
        return self.followers
    def get_leaders(self):
        return self.leaders
    def get_agents(self):
        return self.leaders+self.followers
            





        
            

        


   

       
    

    
















        
  
