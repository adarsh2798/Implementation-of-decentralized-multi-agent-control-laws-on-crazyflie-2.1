
from utils import get_sequence,closest_foll_to_triangle,get_closest_triangle_to_foll




class agent(object):


    def __init__(self,cf_id,position,yaw=0):

        self.cf_id=cf_id
        self.position=position
        self.yaw=yaw
        self.neighbors=None
        self.angle=0

    def assign_neighbors(self,neighbors):

        self.neighbors=neighbors



class MAS():
    
    
    # _id_loc is a list conatining elements of form (cf_id,[x,y])

    def __init__(self,m,n,leaders_id_loc,foll_id_loc):

        self.m=m  #number of followers ,NOTE: for level-1, ensure (n-2)<m<=2*(n-2)
        self.n=n  #number of leaders

        self.leaders=[agent(l[0],l[1]) for l in leaders_id_loc]
        self.followers=[agent(f[0],f[1]) for f in foll_id_loc]
    

    # this function determines the sequence in which leader cf's are arranged. This seq is needed
    # to compute non-overalpping triangles
    def get_initial_leaders_seq(self):

        leaders_loc=[x.position for x in self.leaders]
        
        ans=get_sequence(leaders_loc)

        leaders_seq=[self.leaders[a] for a in ans]
        
        
        return leaders_seq

    def find_non_overlapping_triangles(self,leaders_seq):

        self.tri=[]

        i=0
        while (i<self.n-2):
            self.tri.append([leaders_seq[0],leaders_seq[i+1],leaders_seq[i+2]])
            i+=1
        
        
    

    def assign_followers_per_triangle(self):

        # as per the paper 2 followers are assigned to each traingle intially
        self.num_foll_per_tri=[2 for i in range(self.n-2)]
        self.foll_per_tri=[[] for k in range(self.n-2)]
        k=2*(self.n-2)
        i=0

        # The below while loop assigns actual number of followers per triangle based on 'm'
        # If m=k then its trivial. If m<k then we keep removing one follower from each triangle
        # till k=m, so in this case dependng on 'm' some triangle may have 1 follower 
        
        while(k!=self.m):

            self.num_foll_per_tri[i]-=1
            i+=1
            i=i%(self.n-2)
            k-=1
        
        followers_temp=self.followers.copy() # simple equal to in python means both will reference
                                             # same memory address hence changes in one will be 
                                             # reflected in other
        
        tri_temp=self.tri.copy()

        ### assigning triangles to followers
        """
        t=0
        for tri in self.tri:

            tri_v1=tri[0].position
            tri_v2=tri[1].position
            tri_v3=tri[2].position

            tri_pos=[tri_v1,tri_v2,tri_v3]

            for i in range(self.num_foll_per_tri[t]):


                closest_follower=closest_foll_to_triangle(tri_pos,followers_temp)

                self.foll_per_tri[t].append(closest_follower)

                followers_temp.remove(closest_follower)
            t+=1
        """
        ## assigning followers to triangles
        curr_num_foll_per_tri=[0 for i in range(self.n-2)]
        for f in followers_temp:

              closest_triangle=get_closest_triangle_to_foll(tri_temp,f)
              index=self.tri.index(closest_triangle)
              self.foll_per_tri[index].append(f)
              curr_num_foll_per_tri[index]+=1

              if(curr_num_foll_per_tri[index]==self.num_foll_per_tri[index]):
                  tri_temp.remove(closest_triangle)
   
    def link_followers_leaders(self):

        for i in range (self.n-2):

            for j in range (self.num_foll_per_tri[i]):

                neighbors=[self.tri[i][0],self.tri[i][1],self.tri[i][2]]

                self.foll_per_tri[i][j].assign_neighbors(neighbors)

    
    def create_MAS(self):
       self.leaders=self.get_initial_leaders_seq()
       self.find_non_overlapping_triangles(self.leaders)
       self.assign_followers_per_triangle()
       self.link_followers_leaders()

    def get_followers(self):
        return self.followers
    def get_leaders(self):
        return self.leaders



"""
if __name__=="__main__":

    mas=MAS(4,4,[(3,[1,-1]),(1,[-1,1]),(2,[1,1]),(4,[-1,-1])],[(5,[-2,0.5]),(6,[0.8,-2]),(7,[3,0]),(8,[0.5,3])])
    
    mas=MAS(3,4,[(3,[1,-1]),(1,[-1,1]),(2,[1,1]),(4,[-1,-1])],[(5,[-2,0.5]),(6,[0.8,-2]),(7,[3,0])])

    mas=MAS(2,4,[(3,[1,-1]),(1,[-1,1]),(2,[1,1]),(4,[-1,-1])],[(5,[-2,0.5]),(6,[0.8,-2])])

    #mas=MAS(1,4,[(3,[1,-1]),(1,[-1,1]),(2,[1,1]),(4,[-1,-1])],[(5,[-2,0.5])])
    
    l1=(3,[4,3])
    l2=(5,[0,3])
    l3=(2,[3,1])
    l4=(1,[0,0])
    l5=(4,[2,4])

    f1=(6,[-2.6,8.36])
    f2=(7,[8,8])
    f3=(8,[-3,-6])
    f4=(9,[0.8,-4.84])
    f5=(10,[9.9,1.7])
    f6=(11,[8.8,1.29])
    f7=(12,[7,2.7])
    f8=(13,[7,-5.36])
    f9=(14,[-3.5,-3.38])
    f10=(15,[-3.6,2.8])

    mas=MAS(2,5,[l1,l2,l3,l4,l5],[f1,f2])
    mas.create_MAS()

    f=mas.get_followers()

    for ff in f:
        ff_n=ff.neighbors
        l=[ff_n_a.cf_id for ff_n_a in ff_n]
        
        print(ff.cf_id,"--->",l)

"""












        
  
