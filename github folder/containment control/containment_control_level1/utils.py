import math
import numpy as np


#points is a list of form  polygon_vertices = [[0, 0],[3, 1],[4, 3],[2, 4],[0, 3]]
# This function returns the points in sequence(assuming the points form a convex polygon)
# We use a line sweep to measure angle of each point wrt centroid and sort in terms of angle
def get_sequence(points):
    
    seq_points=[]
    c=np.mean(points,axis=0)
    check=[]
    for i in range(len(points)):
       line=points[i]-c
       angle=math.atan2(line[1],line[0])*180/math.pi
       if angle<0:
          angle+=360
       check.append((i,angle))

    check = sorted(check, key=lambda x: x[1])

    for ch in check:
      seq_points.append(ch[0])
    return seq_points



def dist(p1,p2):

   t1=(p1[0]-p2[0])**2
   t2=(p1[1]-p2[1])**2

   return ((t1+t2)**0.5)

def closest_foll_to_triangle(tri_pos,followers):

   c=[(tri_pos[0][0]+tri_pos[1][0]+tri_pos[2][0])/3,(tri_pos[0][1]+tri_pos[1][1]+tri_pos[2][1])/3]
   
   closest_f=-1
   min_dist=math.inf
   for f in followers:

      p=f.position

      d=dist(c,p)

      if(d<min_dist):
         closest_f=f
         min_dist=d
   return closest_f


def get_closest_triangle_to_foll(triangles,follower):
   
   closest_triangle=-1
   min_dist=math.inf
   
   for t in triangles:

      c=[(t[0].position[0]+t[1].position[0]+t[2].position[0])/3,(t[0].position[1]+t[1].position[1]+t[2].position[1])/3]

      d=dist(c,follower.position)
      
      if(d<min_dist):
         closest_triangle=t
         min_dist=d
   return closest_triangle



def find_agent_by_cf_id(agents,cf_id):

   for a in agents:
      if a.cf_id==cf_id:
         return a

def get_leader_hull_centroid(leaders_list,leader_agents):
   c_x=0
   c_y=0
   n=len(leaders_list)
   for l in leaders_list:
      l_agent=find_agent_by_cf_id(leader_agents,l)
      c_x+=l_agent.position[0]
      c_y+=l_agent.position[1]
   
   c_x=c_x/n
   c_y=c_y/n
   c=[c_x,c_y]
   return c



def saturate(x,thresh):
    if abs(x)>thresh:
        x=np.sign(x)*thresh
    return x 



