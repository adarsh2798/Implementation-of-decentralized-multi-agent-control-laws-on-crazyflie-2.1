import math
import numpy as np




def saturate(x,thresh):
    if abs(x)>thresh:
        x=np.sign(x)*thresh
    return x 


def convert_accel2attitude(a):
    g=9.8
    acc_thresh=0.1
    accelx=saturate(a[0,0],acc_thresh)
    accely=saturate(a[1,0],acc_thresh)
    #### to control x-accel we control pitch angle ##########
    pitch_command=-np.arctan2(saturate(a[0,0],acc_thresh),g)*180/np.pi

    pitch_command=np.arcsin(accelx/g)*180/np.pi ##method2
    

    #### to control y-accel we control roll angle ##########
    roll_command=-np.arctan2(saturate(a[1,0],acc_thresh),g)*180/np.pi

    roll_command=-np.arcsin(accely/(g*np.cos(np.radians(pitch_command))))*180/np.pi #method2

    #return (roll_command,pitch_command)
    return (roll_command,-pitch_command)  ##methdo2


def dist(p1,p2):

   t1=(p1[0]-p2[0])**2
   t2=(p1[1]-p2[1])**2

   return ((t1+t2)**0.5)





def find_agent_by_cf_id(agents,cf_id):

   for a in agents:
      if a.cf_id==cf_id:
         return a
def get_R(angle):
     angle=angle*np.pi/180
     R= np.array([[np.cos(angle), -np.sin(angle)], 
                         [np.sin(angle),  np.cos(angle)]])
     return R



def ema_filt(fc,curr,prev):

    return ((fc*curr)+((1-fc)*prev))










