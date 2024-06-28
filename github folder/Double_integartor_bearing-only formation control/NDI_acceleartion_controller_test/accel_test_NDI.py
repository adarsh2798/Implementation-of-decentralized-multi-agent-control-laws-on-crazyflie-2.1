from cflib.crazyflie.log import LogConfig
import matplotlib.pyplot as plt
import math
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie

import cflib.crtp

from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import time
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as R
from scipy.signal import savgol_filter

#from transforms3d.euler import euler2quat


URI1='radio://0/80/2M/E7E7E7E701'
URI2='radio://0/80/2M/E7E7E7E702'
URI3='radio://1/50/2M/E7E7E7E703'
URI4='radio://0/50/2M/E7E7E7E704'
URI5='radio://0/20/2M/E7E7E7E705'
URI6='radio://0/20/2M/E7E7E7E706'
URI=URI6


print_prev_time=0

fc=10
fs=70
vbat=0
th_log=[]
th_calc_pwm=[]
th_calc_volt=[]
ax=[]
ay=[]
az=[]
axf=[]
ayf=[]
azf=[]
accel_sensor=np.zeros((3,1))
accel_est=np.zeros((3,1))
ax_ref=[]
ay_ref=[]
m1_pwm=0
m2_pwm=0
m3_pwm=0
m4_pwm=0
p_x=0
p_y=0
p_z=0

pxl=[]
pyl=[]

w_x=0
w_y=0

af_sensor=np.zeros((3,1))
af_est=np.zeros((3,1))
tf=np.zeros((3,1))
prev_pitch=0
pitch=0
roll=0
prev_roll=0
cf_mass=0.034 #0.027
g=9.81
T_bz=0




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



##### PID class################################################################
class PIDController:
    def __init__(self, desired,kp, ki, kd,kff,dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff=kff
        self.error=0
        self.deriv=0
        self.integral = 0
        self.previous_error = 0
        self.desired=desired
        self.iLimit=5000
        self.outputLimit=50000
        self.dt=dt

    def set_iLimit(self,iLimit):
      self.iLimit=iLimit
    def reset(self):
        self.error=0
        self.deriv=0
        self.integral = 0
        self.previous_error = 0
    def setError(self,error):
      self.error=error
    def setDesired(self,desired):
      self.desired=desired

    def update(self, measured,updateError):
        output=0
        if updateError:
          self.error=self.desired-measured
        outP=self.kp*self.error
        output+=outP
        deriv=(self.error-self.previous_error)/self.dt
        self.deriv=deriv
        if math.isnan(self.deriv):
          self.deriv=0
        outD=self.kd*self.deriv
        output+=outD

        self.integral += self.error * self.dt
        self.integral=min(self.iLimit,max(-self.iLimit,self.integral))
        outI=self.ki*self.integral
        output+=outI

        outFF=self.kff*self.desired
        output+=outFF

        output=min(self.outputLimit,max(-self.outputLimit,output))
        self.previous_error = self.error

        return output


def get_R(roll,pitch,yaw):
     roll=np.radians(roll)
     pitch=np.radians(pitch)
     yaw=np.radians(yaw)
     r = R.from_euler('xyz', [roll, pitch, yaw])
     rotation_matrix = r.as_matrix()

     return rotation_matrix




def get_thrust_from_pwm(pwm):

    ###these a,b,c were obtained from the polyfit func using bitcraze data of (N)thrust vs volt applied
    #thrust in (g) was converted to N by *(9.8/1000). Then that column was used as y and x was volt.
    #np.polyfit(x,y)
    a=0.03671014
    b=0.05804912
    c=0.00706211

    ###these a,b,c were copied from internet where some user similarly found individual motor 
    #pwm(0-65535) to thrust (N)
    a=2.130295e-11
    b=1.032633e-6
    c=5.484560e-4
    
    ## these a,b,c are same as previous ones, only these I found. The thrust(g) was converted to 
    ## N by *(9.8/1000), then was divided by 4 for individual motor
    ## the  pwm column (0-100) was scaled to (0-65535) by *(65535/100)
    
    a=1.51508942e-11
    b=1.35166819e-06
    c=-2.16243299e-04

    ans=a*(pwm**2)+(b*pwm)+c
    return ans

def get_thrust_from_volt(v):

    ### NOTE::: thrust from volt seems more appropriate coz just by pwm (0-65535) we only get duty cycle
    ### so pwm of 65535/2 means 50% duty so motor driver mught apply 50 percent of 
    ### supply voltage, but here supply voltage is not constant!! it keeps droppijng on 
    ## battery so we log vbat every instant and find actual volt applied absed on thus vbat and pwm
    ### then use this mapping function

    ###these a,b,c were obtained from the polyfit func using bitcraze data of (N)thrust vs volt applied
    #thrust in (g) was converted to N by *(9.8/1000). Then that column was used as y and x was volt.
    #np.polyfit(x,y)
    a=0.03671014
    b=0.05804912
    c=0.00706211

    ### same as above but these are for individual motors so thrust was divided by 4
    a=0.00917754
    b=0.01451228
    c=0.00176553

    ans=a*(v**2)+(b*v)+c
    return ans


########## 2nd order centered finiyte diff ######################
class finite_diff_2:
    def __init__(self,buffer_size,dt):
        self.buffer_size=buffer_size
        self.buffer=deque(maxlen=self.buffer_size)
        self.dt=1

    def calc(self,data):
        self.buffer.append(data)

        if len(self.buffer)==self.buffer_size:
            smoothed_buffer = savgol_filter(list(self.buffer), window_length=3, polyorder=2)
            smoothed_buffer = deque(smoothed_buffer, maxlen=self.buffer_size)
            d_prev, d_curr, d_next = smoothed_buffer
    
             
    
            # Central difference for acceleration
            deriv_2 = (d_next - 2 * d_curr + d_prev) / (self.dt ** 2)
            return deriv_2
        return 0


     
#### this implements a real-tile live butterworth 2nd order dgital filter
#### a0*y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
class filter:
    def __init__(self,fc,fs):
        self.xq=deque([0] * 3, maxlen=3)
        self.yq=deque([0] * 2, maxlen=2)
        self.fc=fc
        self.fs=fs
    def filter_it(self,x):
        gamma=1/(np.tan(np.radians(180*self.fc/self.fs)))

        denom=(gamma**2)+((2**0.5)*gamma)+1

        a1=((-2*(gamma**2))+2)/denom
        a2=((gamma**2)-((2**0.5)*gamma)+1)/denom

        b0=1/denom
        b1=2*b0
        b2=b0

        a=np.array([1,a1,a2])
        b=np.array([b0,b1,b2])

        self.xq.appendleft(x)
        y = np.dot(b, self.xq) - np.dot(a[1:], self.yq)
        self.yq.appendleft(y)
        return y




ax_filter=filter(fc,fs)  
ay_filter=filter(fc,fs)  

px2ax=finite_diff_2(3,0.01)
py2ay=finite_diff_2(3,0.01)
pz2az=finite_diff_2(3,0.01)

def log_callback_1(timestamp,data,logconf_1):
  
    global vbat,th_log,th_calc_pwm,th_calc_volt,m1_pwm,m2_pwm,m3_pwm,m4_pwm,p_x,p_y,ax,ay,az,af_est,af_sensor,tf,roll,pitch,T_bz,p_z
    global prev_pitch,prev_roll,accel_sensor,accel_est,pxl,pyl
    

    

    vbat=data['pm.vbat']
    p_x=data['stateEstimate.x']
    p_y=data['stateEstimate.y']
    p_z=data['stateEstimate.z']
    pxl.append(p_x)
    pyl.append(p_y)
    m1_pwm=data['motor.m1']
    m2_pwm=data['motor.m2']
    m3_pwm=data['motor.m3']
    m4_pwm=data['motor.m4']
    
    max_pwm=65535
    m1_th=get_thrust_from_volt((m1_pwm/max_pwm)*vbat)
    m2_th=get_thrust_from_volt((m2_pwm/max_pwm)*vbat)
    m3_th=get_thrust_from_volt((m3_pwm/max_pwm)*vbat)
    m4_th=get_thrust_from_volt((m4_pwm/max_pwm)*vbat)

    th_calc_volt.append(m1_th+m2_th+m3_th+m4_th)
     
    

    accel_sensor=np.array([[data['stateEstimate.ax']],[data['stateEstimate.ay']],[data['stateEstimate.az']]])*9.80665+np.array([[0],[0],[g]])
    af_sensor=np.array([[ax_filter.filter_it(accel_sensor[0,0])],[ay_filter.filter_it(accel_sensor[1,0])],[accel_sensor[2,0]]])
    #af=np.array([[accel[0,0]],[accel[1,0]],[accel[2,0]]])

    accel_est=np.array([[px2ax.calc(p_x)],[px2ax.calc(p_y)],[px2ax.calc(p_z)]])+np.array([[0],[0],[g]])
    af_est=np.array([[ax_filter.filter_it(accel_est[0,0])],[ay_filter.filter_it(accel_est[1,0])],[accel_est[2,0]]])
    #af=np.array([[accel[0,0]],[accel[1,0]],[accel[2,0]]])

    pitch=data['stateEstimate.pitch']
    roll=data['stateEstimate.roll']
    yaw=0

    R_mat=get_R(roll,pitch,yaw)
    thrust_bf=np.array([[0],[0],[1*(m1_th+m2_th+m3_th+m4_th)]]) #thrust in body z-axis
    T_bz=m1_th+m2_th+m3_th+m4_th
    
    thrust_if=np.matmul(R_mat,thrust_bf) #thrust in inertial frame
    th_log.append(thrust_if[2,0])
    #tf=np.array([[filter(fc,fs,thrust_if[0,0])],[filter(fc,fs,thrust_if[1,0])],[filter(fc,fs,thrust_if[2,0])]])/cf_mass
    tf=np.array([[thrust_if[0,0]],[thrust_if[1,0]],[thrust_if[2,0]]])/cf_mass



def log_callback_2(timestamp,data,logconf_2):
    global w_x,w_y,roll,pitch,prev_roll,prev_pitch

    w_x=data['stateEstimateZ.rateRoll']*0.0572958
    w_y=data['stateEstimateZ.ratePitch']*0.0572958
    R=get_R(roll,pitch,0)
    wb=np.array([[w_x],[w_y],[0]])
    wi=np.matmul(R,wb)
    w_x=wi[0,0]
    w_y=wi[1,0]
    w_x=(roll-prev_roll)/1
    w_y=(pitch-prev_pitch)/1
    prev_roll=roll
    prev_pitch=pitch



                                                             


   




import matplotlib.patches as patches
center=None
radius=0    
xl=[]
yl=[]

def plot():
    
     global th_log,th_calc_pwm,th_calc_volt,ax,ay,az,axf,ayf,azf,ax_ref,ay_ref,pxl,pyl,xl,yl
     
  

     
     plt.plot(az,color='red',label='az')
     plt.plot(azf,color='green',label='azf')
     plt.figure()

     plt.plot(ax,color='red',label='ax')
     plt.plot(ax_ref,color='green',label='ax_ref')
     plt.figure()
     plt.plot(ay,color='red',label='ay')
     plt.plot(ay_ref,color='green',label='ay_ref')

     plt.figure()
     plt.plot(pxl,label="x")
     plt.plot(pyl,label="y")
     #plt.figure()
     #fig, ax = plt.subplots()
     # Create a circle patch
     #circle = patches.Circle(center, radius, edgecolor='r', facecolor='none')

     # Add the circle patch to the axis
     #ax.add_patch(circle)
     #ax.plot(xl,yl,color='blue')
     #ax.set_aspect('equal', adjustable='box')



    


     plt.legend()
     
     plt.show()

#################### accel controllers ########################################
## dynamic setpoint for az to ensure ut  cf is at desired "z".
pid_az=PIDController(0,0.1,0.0,0.0,0,dt=1)
pid_ax=PIDController(0,1,0.0,0.0,0,dt=1)
pid_ay=PIDController(0,1,0.0,0.0,0,dt=1)
def get_a_setpoint(ob,p_curr,p_des):
  
 
  ob.setDesired(p_des)
  op=ob.update(p_curr,True)
  

  return -op

def get_G(roll,pitch,yaw,T):
    roll=np.radians(roll)
    pitch=np.radians(pitch)

    G=np.array([[(np.cos(roll)*np.sin(yaw)-np.sin(roll)*np.cos(yaw)*np.sin(pitch))*T
                 ,np.cos(roll)*np.cos(yaw)*np.cos(pitch)*T
                 ,np.sin(roll)*np.sin(yaw)+np.cos(roll)*np.cos(yaw)*np.sin(pitch)],
                 
                 [(-np.sin(roll)*np.sin(yaw)*np.sin(pitch)-np.cos(yaw)*np.cos(roll))*T
                  ,np.cos(roll)*np.sin(yaw)*np.cos(pitch)*T
                  ,np.cos(roll)*np.sin(yaw)*np.sin(pitch)-np.cos(yaw)*np.sin(roll)],
                  
                  [-np.cos(pitch)*np.sin(roll)*T
                   ,-np.sin(pitch)*np.cos(roll)*T
                   ,np.cos(roll)*np.cos(pitch)]])
    #print(G)
    return G
prev_acc_err=0
cum_err=0
### normal 1st order NDI with added PID
def accel_controller1(ac,af,T_bz,roll,pitch):
    global prev_acc_err,cum_err
    ilim=500
    r=roll
    p=pitch
    curr_acc_err=ac-af
    cum_err+=curr_acc_err
    cum_err=np.minimum(ilim,np.maximum(-ilim,cum_err))
    d_error=curr_acc_err-prev_acc_err
    d_error/=1
  
    ##########indi main ref implementation#######
    kp=1+70*0
    kd=2
    ki=0.001
    G=get_G(r,p,0,T_bz)
    uf=np.array([[r],[p],[T_bz]])
    uc=uf+cf_mass*np.matmul(np.linalg.inv(G),(kp*(ac-af)+kd*d_error+ki*cum_err))
    if math.isnan(uc[2,0]):
        uc[2,0]=0.5586

    if uc[2,0]>0.5586:
      uc[2,0]=0.5586
    if uc[2,0]<0:
      uc[2,0]=0
    prev_acc_err=curr_acc_err
    return (uc[2,0],uc[0,0],uc[1,0])

### 2nd order NDI (PD error dynamics) enforcement with extra constraint
### on min ang accel commnads (see copy)
enter_flag=0
def accel_controller2(ac,af,T_bz,roll,pitch,rollRate,pitchRate):
  global enter_flag
  rp_thresh=300
  rpR_thresh=300
  kp=57500
  kd=50*3
  r=roll
  p=pitch
  rR=rollRate
  pR=pitchRate

  G=get_G(r,p,0,T_bz)
  
  A=np.concatenate((G/cf_mass,G*kd/cf_mass),axis=1)
  b=kp*(ac-af)
  R_angaccel=0.5*np.array([[1,0,0],[0,1,0],[0,0,1]])
  R_angvel=np.array([[1,0,0],[0,1,0],[0,0,0.001]])*1e-6
  zeros_mat=np.zeros((3,3))
  temp_mat1=np.concatenate((R_angaccel,zeros_mat),axis=1)
  temp_mat2=np.concatenate((zeros_mat,R_angvel),axis=1)
  R_tilde=np.concatenate((temp_mat1,temp_mat2),axis=0)

  X=np.matmul(np.linalg.inv(R_tilde),np.matmul(A.T,np.matmul(np.linalg.inv(np.matmul(A,A.T)),b)))
  Xf=np.array([[rR],[pR],[0],[roll],[pitch],[T_bz]])
  if enter_flag==0:
        print("af=",af)
        print("ac=",ac)
        print("G=",G)
        print("Xf",Xf)
        print("X=",X)
        enter_flag=1

  Xc=X+Xf
  #Xc=X
  if Xc[5,0]>0.5586 or  math.isnan(Xc[5,0]):
      Xc[5,0]=0.5586
  if Xc[5,0]<0:
      Xc[5,0]=0
  #az.append(Xc[5,0])
  Xc[3,0]=min(rp_thresh,max(-rp_thresh,Xc[3,0]))
  Xc[4,0]=min(rp_thresh,max(-rp_thresh,Xc[4,0]))
  Xc[1,0]=min(rpR_thresh,max(-rpR_thresh,Xc[1,0]))
  Xc[0,0]=min(rpR_thresh,max(-rpR_thresh,Xc[0,0]))
  return (Xc[5,0],Xc[3,0],Xc[4,0],Xc[0,0],Xc[1,0])



#1st order NDI different approach refer the paper 
#"Accurate Tracking of Aggressive Quadrotor Trajectories using Incremental Nonlinear Dynamic Inversion and Differential Flatness"
def accel_controller(ac,af,tf):

    global roll,pitch,az,print_prev_time
    r=roll
    p=pitch

    ##########indi main ref implementation#######
    max_Tbz=0.5586
    min_Tbz=0
    k=1
    G=get_G(r,p,0,T_bz)
    uf=np.array([[r],[p],[T_bz]])
    uc=uc=uf+cf_mass*np.matmul(np.linalg.inv(G),k*(ac-af))

    uc[2,0]=min(max_Tbz,max(min_Tbz,uc[2,0]))
    tc=tf+ac-af
    tc=np.array([tc[0,0],tc[1,0],tc[2,0]*1])

    req_thrust_mag=cf_mass*np.linalg.norm(tc)
    azf.append(req_thrust_mag)
 
    def euler_to_quaternion(roll, pitch, yaw):
       # Convert Euler angles to quaternion
       #quaternion = euler2quat(roll, pitch, yaw, 'sxyz')  # 'sxyz' denotes sequential rotations about X, Y, and Z axes

       rq=quaternion_from_vector_and_angle(np.array([1,0,0]),roll)
       pq=quaternion_from_vector_and_angle(np.array([0,1,0]),pitch)
       yq=quaternion_from_vector_and_angle(np.array([0,0,1]),yaw)
       quaternion= quaternion_multiply(yq,quaternion_multiply(pq,rq))

       return quaternion
    def rotate_vector(vector, quaternion):
       # Convert the vector to a quaternion representation (pure quaternion)
       vector_quaternion = np.concatenate(([0], vector))

       # Calculate the rotated vector
       rotated_vector_quaternion = quaternion_multiply(quaternion, quaternion_multiply(vector_quaternion, quaternion_conjugate(quaternion)))
       #rotated_vector_quaternion = quaternion_multiply(quaternion_conjugate(quaternion), quaternion_multiply(vector_quaternion, quaternion))

       # Convert the resulting quaternion back to a 3D vector representation
       rotated_vector = rotated_vector_quaternion[1:]

       return rotated_vector

    def quaternion_multiply(q1, q2):
       w1, x1, y1, z1 = q1
       w2, x2, y2, z2 = q2

       w = w1*w2 - x1*x2 - y1*y2 - z1*z2
       x = w1*x2 + x1*w2 + y1*z2 - z1*y2
       y = w1*y2 + y1*w2 + z1*x2 - x1*z2
       z = w1*z2 + z1*w2 + x1*y2 - y1*x2

       return np.array([w, x, y, z])
    def quaternion_from_vector_and_angle(vector, angle):
       # Normalize the vector
       vector_normalized = vector / np.linalg.norm(vector)

       # Compute half the angle
       half_angle = angle / 2.0

       # Compute quaternion components
       scalar_part = np.cos(half_angle)
       vector_part = vector_normalized * np.sin(half_angle)

       # Construct the quaternion
       quaternion = np.array([scalar_part, *vector_part])

       return quaternion
    def quaternion_to_euler(quaternion):
       # Create a Rotation object from the quaternion
       r = R.from_quat(quaternion)

       # Convert the Rotation object to Euler angles
       euler_angles = r.as_euler('xyz', degrees=True)  # You can specify the order of rotations ('xyz', 'zyx', etc.)

       return euler_angles
    def normalize_quaternion(quaternion):
       norm = np.linalg.norm(quaternion)
       if norm == 0:
           return quaternion
       return quaternion / norm
    def quaternion_conjugate(q):
       w, x, y, z = q
       return np.array([w, -x, -y, -z])
    
    rot_quat=euler_to_quaternion(np.radians(r), np.radians(p), 0)
    tc_norm=tc/np.linalg.norm(tc)
    tc_bf = rotate_vector(tc_norm, rot_quat)
    z_bf=np.array([0,0, 1])
    theta=np.arccos(np.dot(z_bf,tc_bf))
    axis=np.cross(z_bf,tc_bf)
    q=quaternion_from_vector_and_angle(axis,theta)
    incremental_euler_angles = -quaternion_to_euler(q)

    new_roll=r+incremental_euler_angles[2]
    new_pitch=p+incremental_euler_angles[1]

    if req_thrust_mag>(g*0.057):
        req_thrust_mag=g*0.057
 
    return (req_thrust_mag,new_roll,new_pitch)


def limit(val,min_th,max_th):
    val=min(max_th,max(min_th,val))
    return val  

 

def control_action(scf):
    cf=scf.cf

    
    global p_x,p_y,p_z,print_prev_time,roll,pitch,T_bz,w_x,w_y,accel_est,accel_sensor,xl,yl,center,radius
    
    cf_id=cf.link_uri[-2:]
    cf_id=int(cf_id[-1])  
    s=time.time()
    while(time.time()-s<3):
          x=3
          cf.commander.send_position_setpoint(p_x,p_y,0.8,0) 
    
    def dist(x1,y1,x2,y2):
          return (((x1-x2)**2)+((y2-y1)**2))**0.5 
    x_c=0.3
    y_c=0
    center=(x_c,y_c)
    r=dist(p_x,p_y,x_c,y_c)
    radius=r
    #cf.param.set_value('flightmode.stabModeRoll', '0')
    #cf.param.set_value('flightmode.stabModePitch','0')
    #cf.param.set_value('stabilizer.controller', '2')
    ac=np.array([[-0.001],[0.0],[0.0]])
    s=time.time()
    prev_time=0
    a_step=[0.5,-1]
    index=0
    p=np.array([[p_x],[p_y]])
    #ac=np.array([[0],[0.0],[0]])
    #cf.commander.send_setpoint(0,0,0,0)## to use cf.commander.send_setpoint we need to send a thrust of 0 before giving nonzero val
    
    while(time.time()-s<20):
          


          #######circular motion#######################
          xl.append(p_x)
          yl.append(p_y)
          t_now=time.time()-s
                
          r_vec=np.array([[p_x-x_c],[p_y-y_c]])
          r_unit_vec=r_vec/np.linalg.norm(r_vec)
          w=1
                

          a=-(w**2)*r_vec

          a=np.array([[a_step[index]],[0]])

          if t_now-prev_time>2:
              index+=1
              index=index%(len(a_step))
              prev_time=t_now
              

          accelx=-(w**2)*r*np.cos(2*w*t_now)
          accely=-(w**2)*r*np.sin(2*w*t_now)
       
          #ac[0,0]=accelx*1
          #ac[1,0]=accely*1

          ac[0,0]=get_a_setpoint(pid_ax,p_x,0)
          ac[1,0]=get_a_setpoint(pid_ay,p_y,0)
          ac[0,0]=0.5*np.tanh(((4/3)*np.pi*t_now)-2*np.pi)*0
          ac[1,0]=-0.5*np.tanh(((4/3)*np.pi*t_now)-2*np.pi)*0
          ax_ref.append(ac[0,0])
          ax.append(limit(af_sensor[0,0],-2,2))
          ay.append(limit(af_sensor[1,0],-2,2))

          ay_ref.append(ac[1,0])

          print("ax=",ac[0,0],"ay=",ac[1,0])
          #ac=np.array([[accelx],[accely],[0]])
          
          ac[2,0]= get_a_setpoint(pid_az,p_z,0.8)
          #t,r,p=accel_controller(ac,af,tf)

          #T_bz_des,roll_des,pitch_des=accel_controller1(ac,af,T_bz,roll,pitch)
          T_bz_des,roll_des,pitch_des,roll_rate_des,pitch_rate_des=accel_controller2(ac,af_est,T_bz,roll,pitch,w_x,w_y)
          #az.append(pitch_rate_des)
          azf.append(roll_rate_des)
          #print(ac[2,0],T_bz_des)
          #print(roll_rate_des,pitch_rate_des)
          t_map=int((((T_bz_des-0)*(60000-10001)/(0.5586))+10001))
          azf.append(t_map)
          #print(r,p,t_map)
          if cf_out_of_bounds(p_x,p_y,p_z):
              cf.commander.send_velocity_world_setpoint(0,0,0,0)
              #break
              
          else:
            cf.commander.send_setpoint(roll_des*0,pitch_des*0,0,t_map)
            #cf.commander.send_zdistance_setpoint(-roll_rate_des,-pitch_rate_des, 0, 0.8)
         
    
    ## NOTE::::!!!!::: you must have seen even after control loop ends cf flies randomlyy for some time
    #### ig this is coz when control loop ends the logconf takes some time to stop so as long as logging happening
    #### the sync crazyflie instance still is open and link is still opne to it and so cf is still moving and maybe 
    #### the last command it received
    ### before control loop ends is moving it so to enusre it is stopped include the below as last ever line in 
    ### control loop function
    #cf.commander.send_velocity_world_setpoint(0,0,0,0)
    cf.commander.send_position_setpoint(p_x,p_y,0,0) 
    plot()






cflib.crtp.init_drivers()

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

       
    logconf_1 = LogConfig(name='block-1', period_in_ms=10)
    logconf_2 = LogConfig(name='block-2', period_in_ms=10)
 
    logconf_1.add_variable('stateEstimate.x', 'FP16')
    logconf_1.add_variable('stateEstimate.y', 'FP16')
    logconf_1.add_variable('stateEstimate.z', 'FP16')
    logconf_1.add_variable('pm.vbat', 'uint8_t') ##size 1byte
    logconf_1.add_variable('motor.m1', 'FP16')
    logconf_1.add_variable('motor.m2', 'FP16')
    logconf_1.add_variable('motor.m3', 'FP16')
    logconf_1.add_variable('motor.m4', 'FP16')
    logconf_1.add_variable('stateEstimate.ax', 'FP16')
    logconf_1.add_variable('stateEstimate.ay', 'FP16')
    logconf_1.add_variable('stateEstimate.az', 'FP16')
    
    logconf_1.add_variable('stateEstimate.roll', 'FP16')
    logconf_1.add_variable('stateEstimate.pitch', 'FP16')
    logconf_2.add_variable('stateEstimateZ.rateRoll','FP16')
    logconf_2.add_variable('stateEstimateZ.ratePitch','FP16')
    
    logconf_1.uri=scf.cf.link_uri
    scf.cf.log.add_config(logconf_1)
    scf.cf.log.add_config(logconf_2)
    #scf.cf.log.add_config([logconf_1,logconf_2])

    
    logconf_1.data_received_cb.add_callback(log_callback_1)
    logconf_2.data_received_cb.add_callback(log_callback_2)
    
    logconf_1.start()
    logconf_2.start()
    control_action(scf)
    logconf_1.stop()
    logconf_2.stop()
    