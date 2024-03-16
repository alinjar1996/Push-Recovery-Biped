import pybullet as p
import time
import numpy as np
from biped_6DOF_torso_appended import Biped
#from traj import GenTraj
import csv
import comjacob as cj
import optimize_xy as opt
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from Ax12 import Ax12
import biped_6DOF_u2d2comm_code as u2d2commcode

class BipedSim:
    def __init__(self):
        self.t_stop = 100
        
        #self.t_step = 1/240
        self.t_step1 = 1/240
        self.t_step2 = 17/240
        self.file_written = False
        #self.Kp = np.diag(8*[10])
        #self.Kd = np.diag(8*[2*np.sqrt(10)])
        self.Kp = 10
        self.Kd = 2*np.sqrt(10)
        #self.T = 2
        self.linkmass_data=[]
        self.linkmass = []
        self.jointvelerr = []
        self.momentumerror = []
        self.desiredmomentumfromh = []
        self.actualmomentumfromh = []
        self.Reaction_force = []
        self.timee = []
        self.ZMP = []
        self.CP=[]
        self.q = []
        self.qdot = [ ]
        self.applied_torques = []
#        self.timearray_in = np.linspace(0,self.t_set,self.t_set*240)
#       self.timearray = np.linspace(self.t_set,self.t_stop,(self.t_stop-self.t_set)*240)
       
      #  self.w = 2*np.pi/self.T

        #self.urdf_pos = np.zeros((1,17))
        #self.urdf_orien = np.zeros((1,100))
        #self.local_com_pos = 0
        #self.com_pos = np.zeros((1,100))
        #self.com_base_pos = 0
        #self.com_base = np.zeros((1,100))
        
        #self.modi_a_link_com = np.zeros((10))
        #self.modi_b_link_com = np.zeros((10))
        #self.modi_a_link_origin = np.zeros((10))
        #self.modi_b_link_origin = np.zeros((10))
        self.modi_a_link_mass = np.zeros((10))
        self.modi_b_link_mass = np.zeros((10))
    """ 
    def timeee(self,t):
        total_time = self.t_stop:

        return  total_time
    """
    def map_val(self, x_in, x_min, x_max, y_min, y_max):
       """Linearly maps x to y; returns corresponding y value"""
       m = ((y_max - y_min) / (x_max - x_min))
       y_out = m * (x_in - x_min) + y_min
       return y_out


    def deg_to_pos(self,obj, deg):
       pos = self.map_val(deg, -150, 150, obj.MIN_POS_VAL, obj.MAX_POS_VAL)
       return pos    

    def update_servo_pos(self,target1, target2, target3, target4, target5, target6):

      #  if (leg == 'l'):
      #  print("Sensed Motor positions: ", self.get_angle(self.hipL), self.get_angle(self.kneeL), self.get_angle(self.ankleL))
       # print("target1", target1)
      #  print("self.hipL", self.hipL)
      # motor objects
        all_motors = Ax12(254)
        ankleL = Ax12(1)
        kneeL = Ax12(2)
        hipL = Ax12(3)
        hipR = Ax12(4)
        kneeR = Ax12(5)
        ankleR = Ax12(6)

        self.set_angle(hipL, target1)
        self.set_angle(kneeL, -target2)
        self.set_angle(ankleL, target3)
    
       # elif (leg == 'r'):
      #  print("Sensed Motor positions: ", self.get_angle(self.hipR), self.get_angle(self.kneeR), self.get_angle(self.ankleR))
       
       # self.set_angle(self.hipR, -target1)
       # self.set_angle(self.kneeR, target2)
       # self.set_angle(self.ankleR, -target3)

        self.set_angle(hipR, -target4)
        self.set_angle(kneeR, target5)
        self.set_angle(ankleR, -target6)

    def check_limit(motor_object, deg):
        if deg > 90:
           return 90
        elif deg < -90:
           return -90
        else:
           return deg

    def set_angle(self, id, input_deg):
      #Sets motor to specified input angle.
        dxl_goal_position = int(self.deg_to_pos(id, input_deg))
        self.set_position(dxl_goal_position)


    def get_angle(self):
       """Returns present angle."""
       dxl_present_position = self.get_position()
       dxl_angle = int(self.map_val(dxl_present_position, self.MIN_POS_VAL, self.MAX_POS_VAL, -150, 150))
       
       return dxl_angle

    """
    def rest_position():
       update_servo_pos(0, 0, 0, 'l')
       update_servo_pos(0, 0, 0, 'r')
       time.sleep(5)
    """

    def turn_on():
       Ax12.open_port()
       Ax12.set_baudrate()
      # print('Connection Successful')
       #self.all_motors.set_moving_speed(default_speed)

    """
    def turn_off():
       self.all_motors.disable_torque()
       Ax12.close_port()
    """
    def RR(self,quat): #rotfromquat
        q0=quat[1] #x
        q1=quat[2] #y
        q2=quat[3] #z
        q3=quat[0] #scalar
        #q0,q1,q2,q3 = quat

        #RR = np.matrix([[1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                       # [2*(q1*q2+q0*q3),  1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1)],
                       # [2*(q1*q3-q0*q2),  2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)]])

        RR = np.array([[1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                        [2*(q1*q2+q0*q3),  1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1)],
                        [2*(q1*q3-q0*q2),  2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)]])


        

   
        # T = np.concatenate((np.vstack((R,[0,0,0])), np.array([[x,y,z,1]]).T), axis=1)
        return RR
    def RR_modi(self,quat):
        q = quat
        Q=p.getMatrixFromQuaternion(q) 
        #Qm =[]
        #Qm.append(Q[0:3])
        #Qm.append(Q[3:6])
        #Qm.append(Q[6:9])
        #Qm=np.matrix(Qm)
        A=np.zeros((3,3))
        A[0][0]=Q[0]
        A[0][1]=Q[1]
        A[0][2]=Q[2]

        A[1][0]=Q[3]
        A[1][1]=Q[4]
        A[1][2]=Q[5]

        A[2][0]=Q[6]
        A[2][1]=Q[7]
        A[2][2]=Q[8]

        B=A 
        return B
    """
    def getCOMPos(self, link_mass, link_com_pos, com_base_pos):
       # return (self.m_link['l1a']*self.com_thigh_l + self.m_link['l2a']*self.com_pos[5] + self.m_link['l3a']*self.com_feet_l + \
          #  self.m_link['l1b']*self.com_thigh_r + self.m_link['l2b']*self.com_pos[13] + self.m_link['l3b']*self.com_feet_l + \
               # self.m_link['base']*self.com_base_pos) / ( self.m_link['l1a'] + self.m_link['l2a'] + self.m_link['l3a'] + self.m_link['l1b'] + self.m_link['l2b'] + self.m_link['l3b'] + self.m_link['base'])
        #self.m_array = link_mass

        com_base_pos=np.array(com_base_pos)

        self.COM_pos=(link_mass[0]*link_com_pos[0]+ link_mass[1]*link_com_pos[1]+ link_mass[2]*link_com_pos[2]+ link_mass[3]*link_com_pos[3]+\
                link_mass[4]*link_com_pos[4]+ link_mass[5]*link_com_pos[5]+ link_mass[6]*link_com_pos[6]+ link_mass[7]*link_com_pos[7]+\
                link_mass[8]*link_com_pos[8]+ link_mass[9]*link_com_pos[9]+ link_mass[10]*link_com_pos[10]+ link_mass[11]*link_com_pos[11]+\
                link_mass[12]*link_com_pos[12]+ link_mass[13]*link_com_pos[13]+ link_mass[14]*link_com_pos[14]+ link_mass[15]*link_com_pos[15] +\
                link_mass[16]*com_base_pos)*(1/np.sum(link_mass))

        return(self.COM_pos)
    """
    def skewM(self,e):
        """

        converting vector into a skew symmetric matrix 
        for vector multiplication

        """
        skewM = np.array([[    0,  -e[2],   e[1]],
                          [ e[2],      0,  -e[0]],
                          [-e[1],   e[0],     0]])
        return skewM        
    def e(self):
        return self.tpos - self.q
    
    def edot(self):
        return self.tdpos - robot.qdot

    

    def getNormalForce_modi(self):

        #self.urdf_path1 = "/home/udayan/Desktop/Alinjar/Biped Pybullet/bipedal_12_01-20220809T113418Z-001/bipedal_12_01/box_small.urdf"

        #self.plane_2  = p.loadURDF(self.urdf_path1,useFixedBase= False, basePosition=[-0.1,-0.25,0.025])

        p.setRealTimeSimulation(0)

        p.stepSimulation()
        
        """
        f1a = p.getContactPoints(robot.robot_id,robot.planeID,47,-1)

        f2a = p.getContactPoints(robot.robot_id,robot.planeID,24,-1)

        f1b = p.getContactPoints(robot.robot_id,robot.planeID,48,-1)

        f2b = p.getContactPoints(robot.robot_id,robot.planeID,25,-1)
        """
        f1a = p.getContactPoints(robot.robot_id,robot.planeID,5,-1)

        f2a = p.getContactPoints(robot.robot_id,robot.planeID,2,-1)

        f1b = p.getContactPoints(robot.robot_id,robot.planeID,6,-1)

        f2b = p.getContactPoints(robot.robot_id,robot.planeID,3,-1)

        N1a = N2a = N1b = N2b = 0
        fric1ay = fric1ax = 0
        fric2ay = fric2ax = 0
        fric1by = fric1bx = 0
        fric2by = fric2bx = 0
        fric1y_dir=0
        fric2y_dir=0
        fric1x_dir=0 
        fric2x_dir=0 
        
        

       # minlenf1 = min(len(f1a)-1,len(f1b)-1)

       # maxlenf1 = max(len(f1a),len(f1b))

        C1a = []

        Norm1a = []

        at1a = []

        friction1ax = []

        friction1ay = []

        CP =[] #contact_points

        N1C1x = []

        N1C1y = []
        
        for i in range(len(f1a)):


            N1a += f1a[i][9]
            fric1ay += f1a[i][10]
            fric1y_dir+=np.array([f1a[i][11]])
            fric1ax += f1a[i][12]
            fric1x_dir+=np.array([f1a[i][13]])

            Norm1a.append(f1a[i][9])
            friction1ax.append(f1a[i][12])
            friction1ay.append(f1a[i][10])

            CP.append(f1a[i][6][0:2]) #contact point on ground

            #Force1a = np.array()
            C1a.append(f1a[i][5]) #contact point on robot

            N1C1x.append(f1a[i][9]*f1a[i][6][0])
            N1C1y.append(f1a[i][9]*f1a[i][6][1])

         #   N1C1x += f1a[i][9]*f1a[i][6][0]
          #  N1C1y += f1a[i][9]*f1a[i][6][1]

           # N1c1 = np.dot()

            #at1a.append(np.dot(np.array([0,0,Norm1a]),np.array([C1a])))

       

        ankle_joint_pos = robot.urdf_pos[5]

        for j in range(len(C1a)):

            F = np.array([friction1ax[j],friction1ay[j],Norm1a[j]])

            C = np.array([C1a[j]])-ankle_joint_pos

            F = np.reshape(F,(3,1))

            C = np.reshape(C,(1,3))

            at1a.append(np.dot(C,F))
             
        


        C1b = []

        Norm1b = []

        at1b = []

        friction1bx = []
        
        friction1by = []
        
        

        for i in range(len(f1b)):


            N1b += f1b[i][9]
            fric1by += f1b[i][10]
            fric1y_dir+=np.array([f1b[i][11]])
            fric1bx += f1b[i][12]
            fric1x_dir+= np.array([f1b[i][13]])

            Norm1b.append(f1b[i][9])

            friction1bx.append(f1b[i][12])
            friction1by.append(f1b[i][10])

            CP.append(f1b[i][6][0:2])

            C1b.append(f1b[i][5])

            N1C1x.append(f1b[i][9]*f1b[i][6][0])
            N1C1y.append(f1b[i][9]*f1b[i][6][1])

        


        ankle_joint_pos = robot.urdf_pos[6]
        
        for j in range(len(C1b)):

            F = np.array([friction1bx[j],friction1by[j],Norm1b[j]])

            C = np.array([C1b[j]])- ankle_joint_pos

            F = np.reshape(F,(3,1))

            C = np.reshape(C,(1,3))

            at1b.append(np.dot(C,F))
             
       
        C2a = []

        Norm2a = []    

        at2a = []

        friction2ax = []
        
        friction2ay = []

        N2C2x = []

        N2C2y = []
           
        for i in range(len(f2a)):


            N2a += f2a[i][9]
            fric2ay += f2a[i][10]
            fric2y_dir+=np.array([f2a[i][11]])
            fric2ax += f2a[i][12]
            fric2x_dir+=np.array([f2a[i][13]])

            Norm2a.append(f2a[i][9])
            friction2ax.append(f2a[i][12])
            friction2ay.append(f2a[i][10])

            N2C2x.append(f2a[i][9]*f2a[i][6][0])
            N2C2y.append(f2a[i][9]*f2a[i][6][1])

            CP.append(f2a[i][6][0:2])

            C2a.append(f2a[i][5])

        ankle_joint_pos = robot.urdf_pos[2]    
        
        for j in range(len(C2a)):

            F = np.array([friction2ax[j],friction2ay[j],Norm2a[j]])

            C = np.array([C2a[j]])- ankle_joint_pos

            F = np.reshape(F,(3,1))

            C = np.reshape(C,(1,3))

            at2a.append(np.dot(C,F))





        C2b = []

        Norm2b = []        

        at2b  = []

        friction2bx = []
        
        friction2by = []

        for i in range(len(f2b)):


            N2b += f2b[i][9]
            fric2by += f2b[i][10]
            fric2y_dir+=np.array([f2b[i][11]])
            fric2bx += f2b[i][12]
            fric2x_dir+=np.array([f2b[i][13]])

            Norm2b.append(f2b[i][9])
            friction2bx.append(f2b[i][12])
            friction2by.append(f2b[i][10])

            N2C2x.append(f2b[i][9]*f2b[i][6][0])
            N2C2y.append(f2b[i][9]*f2b[i][6][1])

            CP.append(f2b[i][6][0:2])

            C2b.append(f2b[i][5])
        

        ankle_joint_pos = robot.urdf_pos[3] 

        for j in range(len(C2b)):

            F = np.array([friction2bx[j],friction2by[j],Norm2b[j]])

            C = np.array([C2b[j]]) - ankle_joint_pos

            F = np.reshape(F,(3,1))

            C = np.reshape(C,(1,3))

            at2b.append(np.dot(C,F))
   
        
        N1 = N1a + N1b
        N2 = N2a + N2b
        fric1x = fric1ax + fric1bx
        fric2x = fric2ax + fric2bx
        fric1y = fric1ay + fric1by
        fric2y = fric2ay + fric2by  

        ankle_torque_1 = np.sum(at1a)+ np.sum(at1b) 
        ankle_torque_2 = np.sum(at2a)+ np.sum(at2b)

        #print('ankle_torque', ankle_torque)
        #ankletorque = Norm1a*C1a+Norm2a*C2a+Norm1b*C1b+Norm2b*C2b
        
        xzmp=(np.sum(N1C1x)+np.sum(N2C2x))/(N1+N2)
        yzmp=(np.sum(N1C1y)+np.sum(N2C2y))/(N1+N2)

        zmp=[xzmp, yzmp]

        return N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp   

    def chooseeig(self,v,thd):

        xnorm=np.zeros((len(robot.rev),1))
       
        for i in range(len(v)):
          x= v[:,i]-thd
          xnorm[i]=np.linalg.norm(x)
        
        xnormmin = min(xnorm)

        for k in range(len(v)):

          if  xnorm[k] ==  xnormmin:
            
            choseneig =  v[:,k]

           # print('k',k)             

        return choseneig

          
    def plott(self,points,hull, zmp, t):
              
        import matplotlib.pyplot as plt
        plt.rc('font', size=20)
        plt.plot(points[:,0], points[:,1], 'o')

        for simplex in hull.simplices:
          plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
        
        
        plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
        plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'ro')
        plt.plot(zmp[0],zmp[1], 'gs')

        plt.title("t=" + str(t-0.5))
        
        plt.show() 

    
    def getJointState(self):
        joints = p.getJointStates(robot.robot_id,robot.rev)
        self.q = [ i[0] for i in joints]
        self.qdot = [ i[1] for i in joints]
        self.applied_torques = [i[3] for i in joints]
        
      #  print("qdot:",self.qdot)
        
        self.jdetails=[]
        #for i in joints:
        for ii in range(6):
          self.jdetails.append(p.getJointInfo(robot.robot_id,ii)[13])
      
      #  print("jdetails:",self.jdetails)
        #print("jd",self.jdetails)
        alljoints = p.getJointStates(robot.robot_id, robot.allj)
        qall = [ i[0] for i in alljoints]
        qdotall = [ i[1] for i in alljoints]


      #  jointinfo = p.getJointInfo(robot.robot_id,9)

        #print('joininfo', jointinfo)
        return self.q, qdotall, self.qdot


    def runSim(self):
        #robot.setInitialState()
        #robot.turnOffActuators()
        #robot.turnOffDamping()
        #p.enableJointForceTorqueSensor(robot.robot_id,7,enableSensor=1)

        # while True:
        nj = 6; #number of revolute joints
        self.dthdes = np.zeros((nj))
        self.thdes = np.zeros((nj))
        self.ddthdes = np.zeros((nj))
        self.jtorques = np.zeros((nj))
        self.thdvel =  np.zeros((nj))
        self.thdvel = np.array(self.thdvel)
        self.dthdes_new = np.zeros((nj))
        self.t_impact = 0.05*2
        self.F_app = np.array([0,0,-0])
        MVrhs = np.zeros((3))
        m_vcm = np.zeros((3))
        xin = np.matrix([[0],[0]])
        self.fric_info =  np.array([0,0,0])
        self.t_rest = 0
        self.t_set = 0.5 + self.t_rest

        #for t in  np.linspace(0,self.t_impact,int((self.t_impact-0)/self.t_step)):

        m_box = 0

       # b_x = 0.08; b_y = 0.02; b_z=0.05

       # Ixx = m_box*b_z*b_z
       # Iyy =m_box*b_x*b_x+m_box*b_z*b_z
       # Izz = m_box*b_x*b_x

      #  print("Ixx", Ixx)
      #  print("Iyy", Iyy)
      #   print("Izz", Izz)

      #  p.changeDynamics(robot.robot_id ,  -1, mass=0)
        p.changeDynamics(robot.robot_id ,  0, mass=m_box) 
       # p.changeDynamics(robot.robot_id ,  0, local_inertial_diagonal = list((Ixx,Iyy,Izz))) 



       # for t in  np.linspace(self.t_impact,self.t_impact+self.t_set,int((self.t_set-self.t_impact)/self.t_step)):
        for t in  np.linspace(0,0+self.t_set,int((self.t_set-0)/self.t_step1)):

            all_motors = Ax12(254)
            ankleL = Ax12(1)
            kneeL = Ax12(2)
            hipL = Ax12(3)
            hipR = Ax12(4)
            kneeR = Ax12(5)
            ankleR = Ax12(6)

            self.timee = np.array([t,0])
            self.timee = np.reshape(self.timee,(2,))

            p.stepSimulation()

            #robot.drawPb1(p.getBasePositionAndOrientation(robot.robot_id)[0])

            #f_cm = p.getLinkState(robot.robot_id,7)[0]
            #f_o = p.getLinkState(robot.robot_id,7)[4]
            #robot.drawPb1(f_o)

            #foot_pose = p.getLinkState(robot.robot_id, 7)[4] 
                #base_p=p.getBasePositionAndOrientation(robot.robot_id)[0]
              
            #f_p = np.array(foot_pose)

            #f_p1 = tuple(f_p+np.array([0,0.0,0.0*t]))
            
                   #b_p = np.array(base_p)

            #ja = p.calculateInverseKinematics(robot.robot_id,7, targetPosition =f_p1)

            
        #ja = p.calculateInverseKinematics(robot.robot_id,-1, targetPosition = tuple(b_p+np.array([0,0,0.5])))
              
                
               
            #p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions=ja)

            #f_pa = np.array(p.getLinkState(robot.robot_id, 7)[4])

            cm = self.checkH()

            #robot.drawPb2(cm)

            #global thdes


            

            
            self.thdes[3] = - 0.31
            self.thdes[4] =  0.31
            #self.thdes[5] = 0
            #thdes[6] = 0.1

            p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes ,targetVelocities= self.dthdes)

           # p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[2], controlMode = p.POSITION_CONTROL,targetPosition=-0.1)
           # p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[1], controlMode = p.POSITION_CONTROL,targetPosition=0.1)

            #k_a = self.invkin()

            #self.invkin()

            jointstates = p.getJointStates(robot.robot_id,robot.rev)

            self.thpos_deg =  [(state[0]*1*180/np.pi) for state in jointstates]

            print("self.thpos_deg", self.thpos_deg)


           # self.thpos_deg = self.thpos_deg.tolist()
           # print("self.thpos_deg", self.thpos_deg[0])
       
#            t1 = self.thpos_deg[0], t2 = self.thpos_deg[1], t3 = self.thpos_deg[2]
#            t4 = self.thpos_deg[3], t5 = self.thpos_deg[4], t6 = self.thpos_deg[5]
        
           # self.update_servo_pos(t1,t2,t3,t4,t5,t6)

            u2d2commcode.update_servo_pos(self.thpos_deg[0], self.thpos_deg[1], self.thpos_deg[2], -self.thpos_deg[3], self.thpos_deg[4], self.thpos_deg[5])

            
            self.write_data()

 

            #time.sleep(1/240)
        

        #self.urdf_path1 = "/home/udayan/Desktop/Alinjar/Biped Pybullet/bipedal_12_01-20220809T113418Z-001/bipedal_12_01/box_small.urdf"

        #self.plane_2  = p.loadURDF(self.urdf_path1,useFixedBase= False, basePosition=[-0.1,-0.25,0.025])

        #p.changeDynamics(self.plane_2 ,  -1, lateralFriction=0.9)
        #p.changeDynamics(self.plane_2 ,  -1, lateralFriction=0.6)
        p.changeDynamics(robot.planeID ,  -1, lateralFriction=0.6)

       # p.changeDynamics(robot.robot_id ,  -1, mass=0.0)  #changing mass of base of biped
       # p.changeDynamics(robot.robot_id ,  0, mass=0.0)
           
     #   time.sleep(1)

        for t in np.linspace(self.t_set,self.t_set+self.t_impact,int((self.t_impact)/self.t_step1)):
         
            #self.timeee(self,t)
            all_motors = Ax12(254)
            ankleL = Ax12(1)
            kneeL = Ax12(2)
            hipL = Ax12(3)
            hipR = Ax12(4)
            kneeR = Ax12(5)
            ankleR = Ax12(6)

            self.timee = np.array([t,0])
            self.timee = np.reshape(self.timee,(2,))
            #print("time", t)  
           # p.setRealTimeSimulation(0)

            robot.drawPbText(str(t))
            #robot.drawPb1(self.c_cm)
            robot.drawPbPoint(self.c_cm)
            p.stepSimulation()
            #robot.getJointState()
            #robot.drawPb1(p.getBasePositionAndOrientation(robot.robot_id)[0])
            self.getJointState()
 

            self.thdvel = np.array([self.thdvel])


            
            global thdvel_prev
            thdvel_prev = self.thdvel

            

            cm = self.checkH() #gets the joint velocity self.thdvel
 
            
            #robot.drawPb2(cm)

 
            self.thddacc = (self.thdvel -  thdvel_prev)*(1/self.t_step1)
            
          


            if t > self.t_set+self.t_rest and t < self.t_impact+self.t_set+self.t_rest:                             #This combination works the best comparison to v=[000000]
               # p.applyExternalForce(robot.robot_id,-1,[5,-4,0],[0,0,0],flags=p.LINK_FRAME)
              # p.applyExternalForce(robot.robot_id,-1,[-0,-9.5,-0],[0,0,0],flags=p.LINK_FRAME) #keep this value
               #p.applyExternalForce(robot.robot_id,-1,[8,-8,-0],[0,0,0],flags=p.LINK_FRAME)
               #self.F_app = np.array([0,-21,0])
               #self.F_app = np.array([-30/4,-50/4,0]) #works well for thdes = 1.4
               self.F_app = np.array([-2.3,-0,0]) 
               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            else:
               self.F_app = np.array([-0,0,-0])    

               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            
            #v = 0.5*np.linalg.pinv(self.h)@(self.M*self.v_torso) + 0*eigvec[4]
            
            
            
            #v=-1*np.linalg.pinv(self.h)@(self.M*self.v_base)

            
            #v = np.insert(v,[0,3],[0,0])
            #v=np.array(v)

            #err_jvel=robot.qdot-v
            #v_array=np.array([v])
            #v=v.tolist()

            
            #v=list(v)

            thd=p.getJointStates(robot.robot_id,robot.rev)



            if t<0.01+self.t_set+self.t_rest:


             
              thinitial = self.q
              



            self.M = np.sum(robot.setLinkM())
           # print("robot_mass", self.M)
        #self.thdvelact = np.delete(self.thdvel,[0,4])
        #self.thdvelact = self.thdvel

        # self.H = cj.getH(robot.m_array)
        
        
       # for i in range(15):

          # self.linkmass=p.getDynamicsInfo(robot.robot_id, i)
        

            self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])

            self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[7]*self.v_torso/(robot.m_array[0]+robot.m_array[7]))           
        
        #dth = np.delete(robot.qdot,[0,4])

            qdot = self.getJointState()[2]
        
            dth=np.array(qdot)
       

        #vbase_r= robot.base_vel+self.skewM(robot.base_angvel)@(robot.urdf_pos[0]--robot.com_base_pos)
         #   vbase_r= robot.link_vel[0]+self.skewM(robot.link_omega[0])@(robot.urdf_pos[1]-robot.com_pos[0])
         #   vbase_l= robot.link_vel[8]+self.skewM(robot.link_omega[8])@(robot.urdf_pos[9]-robot.com_pos[8])
         
         # modified for new urdf
             
            vbase_l= self.v_base+self.skewM(robot.link_omega[1])@(robot.urdf_pos[1]-robot.com_base_pos)
            vbase_r= self.v_base+self.skewM(robot.link_omega[4])@(robot.urdf_pos[4]-robot.com_base_pos)

            #x1=vbase_l+self.skewM(robot.link_omega[10])@(-robot.urdf_pos[10]+robot.com_pos[10])
  

            #Mr = np.sum(robot.setLinkM()[1:8])
            #Ml = np.sum(robot.setLinkM()[9:16])
            Mr = np.sum(robot.setLinkM()[1:4])
            Ml = np.sum(robot.setLinkM()[5:7])

            #M_base= robot.setLinkM()[0]+robot.setLinkM()[17]+robot.setLinkM()[1]+robot.setLinkM()[9]
            M_base= robot.m_array[0]+robot.m_array[7]
        
            modi_link_mass = robot.setLinkM()
            modi_link_vel = np.append(robot.link_vel,[self.v_torso],axis=0)
            vcmdata=robot.getrobotCOMvelocity(modi_link_mass, modi_link_vel)

            vrightcm = vcmdata[0]; vleftcm = vcmdata[1]; vbasecm = vcmdata[2]; vtotalcm =  vcmdata[3]
       
        
    

        
        
            link_vell = np.append(robot.link_vel,[self.v_torso],axis=0)

            
            global mv_cm_prev 

            m_vcm_prev = m_vcm
      
        
            mv=np.zeros((8,3)) #len(robot.m_array) = 18 for new urdf

            for i in range(len(robot.m_array)):
                mv[i,:]=robot.m_array[i]*link_vell[i,:]
        
       
        
            mvcmx=0
        #for i in range(len(robot.m_array)):
            mvcmx=sum(mv[:,0])
            mvcmy=sum(mv[:,1])
            mvcmz=sum(mv[:,2])
        
        #m_vcm=np.matrix([[mvcmx],[mvcmy], [mvcmz]])
            m_vcm=np.array([mvcmx,mvcmy,mvcmz])
       
       
            m_acm = (m_vcm- m_vcm_prev)*(1/self.t_step1)
       #m_vcm = np.sum(np.tile(robot.m_array,(3,1))*link_vell.T,axis=1)

            #self.v_cm = m_vcm/self.M
            self.v_cm = vtotalcm

       

      
        
        #v_torsoo=np.zeros((3,1))
        #v_torsoo[0]=self.v_torso[0]
        #v_torsoo[1]=self.v_torso[1]
        #v_torsoo[2]=self.v_torso[2]
     
        
        #self.h = np.array([m_vcm - self.M*self.v_base]).T@np.linalg.pinv([dth]).T
            mveff= np.array([(Mr*vrightcm + Ml*vleftcm + M_base*vbasecm)])
            mvefft = mveff.T
            #self.momentumerror =  mveff-m_vcm
            self.momentumerror =  mveff
            self.momentumerror = np.reshape(self.momentumerror,(3,))
 

            
            global MVrhs_prev 
            MVrhs_prev = MVrhs



            #MVrhs = np.array([Mr*vbase_r + Ml*vbase_l + M_base*vbasecm])
            MVrhs = np.array([Mr*vbase_r + Ml*vbase_l + M_base*self.v_base])

            Marhs = (MVrhs-MVrhs_prev)*(1/self.t_step1)
            
 
            Mvrhst = MVrhs.T

            if t> self.t_set + self.t_rest+0.01:

               self.h = -Mvrhst@np.linalg.pinv([dth]).T

            else:
               
               self.h = np.zeros((3,6))

 

            err = -(m_vcm-self.h@dth.T-MVrhs)

            #mvcalc = self.h@dth.T+MVrhs-m_vcm

 
            #print("t", t)

            
            #print("self.h", self.h)
            #self.h = (m_vcm - self.M*self.v_torso).T@np.linalg.pinv([dth]).T
            eigvec = np.linalg.eig(self.h.T @ self.h)[1]
            eigval = np.linalg.eig(self.h.T @ self.h)[0]
            idx = eigval.argsort()[::-1]   
            eigval = eigval[idx]
            eigvec = eigvec[:,idx]
            
            
            
            

            self.thdvelll = np.reshape(self.thdvel, (6,1))

            self.thdvel = np.reshape(self.thdvel, (6,1))

            thdvel_prev =  np.reshape(thdvel_prev, (6,1))
            
            #eigcomp = self.chooseeig(eigvec, thdvel_prev)
            

            

            
           # eigcomp = self.chooseeig(eigvec,self.thdvelll)
            eigcomp = self.chooseeig(eigvec, thdvel_prev)
           

            eigcomp =np.reshape(eigcomp, (6,1))


           
           # eigcomp = eigcomp/1000

            


            self.dthdes = -np.linalg.pinv(self.h)@(MVrhs-m_vcm-err).T+eigcomp

            

            #check = self.h@self.dthdes+(MVrhs-m_vcm-err).T-self.h@eigcomp 

   
            self.desiredmomentumfromh = self.h@self.dthdes+Mvrhst

            self.desiredmomentumfromh = np.reshape(self.desiredmomentumfromh,(3,))
            self.thdvell = np.array(self.thdvel)

            global thdes_prev
           
            thdes_prev = self.thdes
     
            
            self.dthdes = np.reshape(self.dthdes,(6,)) 

            self.thdes = np.zeros((6,1))
           

            
            self.thdes[3] = - 0.35
            self.thdes[4] =  0.35
            #self.thdes[5] = 0


            self.thdes = np.reshape(self.thdes,(6,))
            
           

     

            self.thpos = np.reshape(self.thpos,(6,))
            self.thdvel = np.reshape(self.thdvel,(6,))
            
            
           # self.dthdes = np.array([0,0,0,0,0,0,0,0]) 

            the = self.thdes-self.thpos

            dthe =  self.dthdes.T-self.thdvel
            dthe =  self.dthdes.T-self.thdvel
            
            the = np.reshape(the,(6,))
            dthe = np.reshape(dthe,(6,))

            self.ddthdes = [00]*len(robot.rev)
            self.ddthdes = np.reshape(self.ddthdes,(6,))
        

            abar = self.ddthdes + self.Kp*the+self.Kd*dthe



            N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp = self.getNormalForce_modi()   


            
            n= np.linspace(0, 1, 50)

            area = 0
            
            if len(CP)>2:
             
                hull = ConvexHull(CP)

                area =  hull.volume

               # print('area', area)
             
                #print('hull', hull)
                        
                for zz  in range(len(n)):

                    if abs(t-n[zz])<0.1:
                       vv = 00
                     #  self.plott(np.array(CP),hull,zmp, t)

 
            
            self.ZMP = zmp

            self.CP=CP

            #print('self.CP', self.CP)

            fric1 = fric1y*fric1y_dir+fric1x*fric1x_dir
            fric2 = fric2y*fric2y_dir+fric2x*fric2x_dir


            fric = fric1+fric2 #lateral reaction force from ground to feet

            N =N1+N2
            
            F =np.array([0,0,N])+fric

            self.Reaction_force = np.reshape(F,(3,))

            fric_norm = np.linalg.norm(fric)
            
            
            
            
            
            
            X = opt.runOptimization(self.h,self.thdvel, MVrhs)
            
       

            xin = X

            
            X1 = np.array([X[0],X[1],0]) 

            #X1 = np.array([(X[0]),(X[1]),0]) 

            """
            if t<self.t_step+2:

              X1 = np.array([X[0],X[1],0])

            else: 

              X1 = np.array([0.1,0.1,0.1])
            """
            
            Mvrhs = np.reshape(MVrhs,(3,))
  
            xxx=np.real(np.linalg.pinv(self.h)@X1)

            Mvrhsxxx = 1*np.real(np.linalg.pinv(self.h)@Mvrhs)

            xxx = np.reshape(xxx,(6,))
 
           # if t<self.t_stop:
            if t<self.t_impact+self.t_set+self.t_rest:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx)

 

            elif t<self.t_impact+self.t_set+self.t_rest+20:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx) 

            else: 

              self.dthdes_new = 0.0*(np.array(np.real(self.dthdes))-1*xxx)
 
            
            self.dthdes_new = np.real(self.dthdes_new)
            
            self.dthdes_new = np.reshape(self.dthdes_new,(6,))


            
            self.macm_calc  =  self.F_app+F+self.M*np.array([robot.gravityacc])

            self.force_check = self.F_app+F+self.M*np.array([robot.gravityacc])-self.macm_calc
            
            self.force_check = np.reshape(self.force_check,(3,))
            
            Marhs = np.reshape(Marhs,(3,1))

            self.macm_calc = np.reshape(self.macm_calc,(3,1))

            self.thddacc = np.reshape(self.thddacc,(6,1))

            self.F_app = np.reshape(self.F_app,(3,1))

            dth_ = np.reshape(dth,(6,1))

            self.h_dot = -(self.h@ self.thddacc+ Marhs-self.macm_calc)@np.linalg.pinv([dth]).T



            Cx = [1,0,0]
            Cy = [0,1,0]
            xtild = [0.15,0,0]
            ytild = [0,0.15,0]

            Cx= np.reshape(Cx,(1,3))
            Cy= np.reshape(Cy,(1,3))

            xtild= np.reshape(xtild,(3,1))
            ytild= np.reshape(ytild,(3,1))
            
            
            

            if abs(self.Reaction_force[0])> abs(robot.mu*N/np.sqrt(1)):

               a1i =  str('friction limit exceeded_1')

               frx = Cx@(self.h@ self.thddacc+ self.h_dot@dth_+Marhs-self.F_app)



               if frx>0:

                self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs+robot.mu*N+self.F_app-xtild)

               else:

                self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs-robot.mu*N+self.F_app+xtild)    

               #self.dthdes_new = np.real(self.dthdes_new)+xxx


                       
            else:

                a1i =  str('friction ok_1')

 

            
            if abs(self.Reaction_force[1])> abs(robot.mu*N/np.sqrt(1)):

                a2i =  str('friction limit exceeded_2')

                fry = Cy@(self.h@ self.thddacc+ self.h_dot@dth_+Marhs-self.F_app)

 

                if fry>0:

                 self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs+robot.mu*N+self.F_app-ytild)

                else:

                 self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs-robot.mu*N+self.F_app+ytild)  

                #self.dthdes_new = np.real(self.dthdes_new)+xxx

 
                       
            else:

                a2i =  str('friction ok_2')

  
            
            self.fric_info =  np.array([t,a1i,a2i])

            
            
            
            
            p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes ,targetVelocities= self.dthdes_new)

           # p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes,
                                   # targetVelocities=self.dthdes, forces=np.zeros(len(robot.rev))) 
                                    # positionGains = np.array(len(self.dthdes)*[20]) ,
                                    # velocityGains = np.array(len(self.dthdes)*[1.65]))
            
            
            #tau = p.calculateInverseDynamics(robot.robot_id, list(self.thdes), list(self.dthdes), list(abar), flags =1)

            #tau = np.reshape(tau,(15,))

            #self.jtorques = tau[7:15]

            jointstates = p.getJointStates(robot.robot_id,robot.rev)

            self.jtorques = self.applied_torques

            

          
            
            #tau = p.calculateInverseDynamics(robot.robot_id, self.thpos, self.thdvel, abar, flags =1)
            
           # tau_g = p.calculateInverseDynamics(robot.robot_id, list(self.thdes), [0.0] * len(robot.rev), [0.0] * len(robot.rev), flags =1)
            
           # tau = np.reshape(tau,(15,))
           # tau_g = np.reshape(tau_g,(15,))
          
            
            """
            p.setJointMotorControlArray(robot.robot_id, robot.rev,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = tau[7:15])
            """
            
            ##ankle torque control
            
            """
            if t<self.t_set+ self.t_rest + self.t_impact+ 1:

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[3] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  self.jtorques[3] - ankle_torque_2)

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[7] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  self.jtorques[7] - ankle_torque_1) 
            """

            ##ankle angle control
            
            """
            if t<self.t_set+ self.t_rest + self.t_impact+ 5:

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[3] ,
                                        controlMode = p.POSITION_CONTROL, 
                                        targetPosition = 0)

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[7] ,
                                        controlMode = p.POSITION_CONTROL, 
                                        targetPosition = 0) 
            """
            

            phimax = 0 # works for Fx = 5, Fy =38, mu =0.9 for both
            T1 =20
            T2 = 1000
            
            ## HIp torque control
            """
            if area<1e-8:

                if t<T1:

                    phi = 0*(-2*phimax*t*t*t*(1/(T1*T1*T1)) + 3*phimax*t*t*(1/(T1*T1)))

                    p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.POSITION_CONTROL,targetPosition = phi)

                    p.setJointMotorControl2(robot.robot_id,robot.rev[4],p.POSITION_CONTROL,targetPosition = phi)

                else:

                    phi = 0*(2*phimax*t*t*t*(1/(T2*T2*T2)) - 3*phimax*t*t*(1/(T2*T2))) + phimax

                    phi = 1*phi

                    p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.POSITION_CONTROL,targetPosition = phi)

                    p.setJointMotorControl2(robot.robot_id,robot.rev[4],p.POSITION_CONTROL,targetPosition = phi)
            """
           ## HIp torque control 
            """    
            if t<self.t_impact+0.2:

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[0] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  0*self.jtorques[0])

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[4] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =   0*self.jtorques[4]) 
            """
            """
            p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[0] ,
                                        controlMode = p.VELOCITY_CONTROL, 
                                        targetVelocity=-0.1)
            """
            
            
            
            
            
            self.write_data()

        p.changeDynamics(robot.planeID ,  -1, lateralFriction=0.6)

        for t in np.linspace(self.t_set+self.t_impact,self.t_stop,int((self.t_stop-self.t_set-self.t_impact)/self.t_step2)):
            all_motors = Ax12(254)
            ankleL = Ax12(1)
            kneeL = Ax12(2)
            hipL = Ax12(3)
            hipR = Ax12(4)
            kneeR = Ax12(5)
            ankleR = Ax12(6)
           #self.timeee(self,t)
            self.timee = np.array([t,0])
            self.timee = np.reshape(self.timee,(2,))
            #print("time", t)  
           # p.setRealTimeSimulation(0)

            robot.drawPbText(str(t))
            #robot.drawPb1(self.c_cm)
            robot.drawPbPoint(self.c_cm)
            p.stepSimulation()
            #robot.getJointState()
            #robot.drawPb1(p.getBasePositionAndOrientation(robot.robot_id)[0])
            self.getJointState()
 

            self.thdvel = np.array([self.thdvel])


            

            thdvel_prev = self.thdvel

            

            cm = self.checkH() #gets the joint velocity self.thdvel
 
            
            #robot.drawPb2(cm)

 
            self.thddacc = (self.thdvel -  thdvel_prev)*(1/self.t_step1)
            
          


            if t > self.t_set+self.t_rest and t < self.t_impact+self.t_set+self.t_rest:                             #This combination works the best comparison to v=[000000]
               # p.applyExternalForce(robot.robot_id,-1,[5,-4,0],[0,0,0],flags=p.LINK_FRAME)
              # p.applyExternalForce(robot.robot_id,-1,[-0,-9.5,-0],[0,0,0],flags=p.LINK_FRAME) #keep this value
               #p.applyExternalForce(robot.robot_id,-1,[8,-8,-0],[0,0,0],flags=p.LINK_FRAME)
               #self.F_app = np.array([0,-21,0])
               #self.F_app = np.array([-30/4,-50/4,0]) #works well for thdes = 1.4
               self.F_app = np.array([-0,0,0]) 
               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            else:
               self.F_app = np.array([-0,0,-0])    

               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            
            #v = 0.5*np.linalg.pinv(self.h)@(self.M*self.v_torso) + 0*eigvec[4]
            
            
            
            #v=-1*np.linalg.pinv(self.h)@(self.M*self.v_base)

            
            #v = np.insert(v,[0,3],[0,0])
            #v=np.array(v)

            #err_jvel=robot.qdot-v
            #v_array=np.array([v])
            #v=v.tolist()

            
            #v=list(v)

            thd=p.getJointStates(robot.robot_id,robot.rev)



            if t<0.01+self.t_set+self.t_rest:


             
              thinitial = self.q
              



            self.M = np.sum(robot.setLinkM())
           # print("robot_mass", self.M)
        #self.thdvelact = np.delete(self.thdvel,[0,4])
        #self.thdvelact = self.thdvel

        # self.H = cj.getH(robot.m_array)
        
        
       # for i in range(15):

          # self.linkmass=p.getDynamicsInfo(robot.robot_id, i)
        

            self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])

            self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[7]*self.v_torso/(robot.m_array[0]+robot.m_array[7]))           
        
        #dth = np.delete(robot.qdot,[0,4])

            qdot = self.getJointState()[2]
        
            dth=np.array(qdot)
       

        #vbase_r= robot.base_vel+self.skewM(robot.base_angvel)@(robot.urdf_pos[0]--robot.com_base_pos)
         #   vbase_r= robot.link_vel[0]+self.skewM(robot.link_omega[0])@(robot.urdf_pos[1]-robot.com_pos[0])
         #   vbase_l= robot.link_vel[8]+self.skewM(robot.link_omega[8])@(robot.urdf_pos[9]-robot.com_pos[8])
         
         # modified for new urdf
             
            vbase_l= self.v_base+self.skewM(robot.link_omega[1])@(robot.urdf_pos[1]-robot.com_base_pos)
            vbase_r= self.v_base+self.skewM(robot.link_omega[4])@(robot.urdf_pos[4]-robot.com_base_pos)

            #x1=vbase_l+self.skewM(robot.link_omega[10])@(-robot.urdf_pos[10]+robot.com_pos[10])
  

            #Mr = np.sum(robot.setLinkM()[1:8])
            #Ml = np.sum(robot.setLinkM()[9:16])
            Mr = np.sum(robot.setLinkM()[1:4])
            Ml = np.sum(robot.setLinkM()[5:7])

            #M_base= robot.setLinkM()[0]+robot.setLinkM()[17]+robot.setLinkM()[1]+robot.setLinkM()[9]
            M_base= robot.m_array[0]+robot.m_array[7]
        
            modi_link_mass = robot.setLinkM()
            modi_link_vel = np.append(robot.link_vel,[self.v_torso],axis=0)
            vcmdata=robot.getrobotCOMvelocity(modi_link_mass, modi_link_vel)

            vrightcm = vcmdata[0]; vleftcm = vcmdata[1]; vbasecm = vcmdata[2]; vtotalcm =  vcmdata[3]
       
        
    

        
        
            link_vell = np.append(robot.link_vel,[self.v_torso],axis=0)

            

            m_vcm_prev = m_vcm
      
        
            mv=np.zeros((8,3)) #len(robot.m_array) = 18 for new urdf

            for i in range(len(robot.m_array)):
                mv[i,:]=robot.m_array[i]*link_vell[i,:]
        
       
        
            mvcmx=0
        #for i in range(len(robot.m_array)):
            mvcmx=sum(mv[:,0])
            mvcmy=sum(mv[:,1])
            mvcmz=sum(mv[:,2])
        
        #m_vcm=np.matrix([[mvcmx],[mvcmy], [mvcmz]])
            m_vcm=np.array([mvcmx,mvcmy,mvcmz])
       
       
            m_acm = (m_vcm- m_vcm_prev)*(1/self.t_step1)
       #m_vcm = np.sum(np.tile(robot.m_array,(3,1))*link_vell.T,axis=1)

            #self.v_cm = m_vcm/self.M
            self.v_cm = vtotalcm

       

      
        
        #v_torsoo=np.zeros((3,1))
        #v_torsoo[0]=self.v_torso[0]
        #v_torsoo[1]=self.v_torso[1]
        #v_torsoo[2]=self.v_torso[2]
     
        
        #self.h = np.array([m_vcm - self.M*self.v_base]).T@np.linalg.pinv([dth]).T
            mveff= np.array([(Mr*vrightcm + Ml*vleftcm + M_base*vbasecm)])
            mvefft = mveff.T
            #self.momentumerror =  mveff-m_vcm
            self.momentumerror =  mveff
            self.momentumerror = np.reshape(self.momentumerror,(3,))
 

            

            MVrhs_prev = MVrhs



            #MVrhs = np.array([Mr*vbase_r + Ml*vbase_l + M_base*vbasecm])
            MVrhs = np.array([Mr*vbase_r + Ml*vbase_l + M_base*self.v_base])

            Marhs = (MVrhs-MVrhs_prev)*(1/self.t_step1)
            
 
            Mvrhst = MVrhs.T

            if t> self.t_set + self.t_rest+0.01:

               self.h = -Mvrhst@np.linalg.pinv([dth]).T

            else:
               
               self.h = np.zeros((3,6))

 

            err = -(m_vcm-self.h@dth.T-MVrhs)

            #mvcalc = self.h@dth.T+MVrhs-m_vcm

 
            #print("t", t)

            
            #print("self.h", self.h)
            #self.h = (m_vcm - self.M*self.v_torso).T@np.linalg.pinv([dth]).T
            eigvec = np.linalg.eig(self.h.T @ self.h)[1]
            eigval = np.linalg.eig(self.h.T @ self.h)[0]
            idx = eigval.argsort()[::-1]   
            eigval = eigval[idx]
            eigvec = eigvec[:,idx]
            
            
            
            

            self.thdvelll = np.reshape(self.thdvel, (6,1))

            self.thdvel = np.reshape(self.thdvel, (6,1))

            thdvel_prev =  np.reshape(thdvel_prev, (6,1))
            
            #eigcomp = self.chooseeig(eigvec, thdvel_prev)
            

            

            
           # eigcomp = self.chooseeig(eigvec,self.thdvelll)
            eigcomp = self.chooseeig(eigvec, thdvel_prev)
           

            eigcomp =np.reshape(eigcomp, (6,1))


           
           # eigcomp = eigcomp/1000

            


            self.dthdes = -np.linalg.pinv(self.h)@(MVrhs-m_vcm-err).T+eigcomp

            

            #check = self.h@self.dthdes+(MVrhs-m_vcm-err).T-self.h@eigcomp 

   
            self.desiredmomentumfromh = self.h@self.dthdes+Mvrhst

            self.desiredmomentumfromh = np.reshape(self.desiredmomentumfromh,(3,))
            self.thdvell = np.array(self.thdvel)

            
           
            thdes_prev = self.thdes
     
            
            self.dthdes = np.reshape(self.dthdes,(6,)) 

            self.thdes = np.zeros((6,1))
           

            
            self.thdes[3] = - 0.31
            self.thdes[4] =  0.31
            #self.thdes[5] = 0


            self.thdes = np.reshape(self.thdes,(6,))
            
           

     

            self.thpos = np.reshape(self.thpos,(6,))
            self.thdvel = np.reshape(self.thdvel,(6,))
            
            
           # self.dthdes = np.array([0,0,0,0,0,0,0,0]) 

            the = self.thdes-self.thpos

            dthe =  self.dthdes.T-self.thdvel
            dthe =  self.dthdes.T-self.thdvel
            
            the = np.reshape(the,(6,))
            dthe = np.reshape(dthe,(6,))

            self.ddthdes = [00]*len(robot.rev)
            self.ddthdes = np.reshape(self.ddthdes,(6,))
        

            abar = self.ddthdes + self.Kp*the+self.Kd*dthe



            N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp = self.getNormalForce_modi()   


            
            n= np.linspace(0, 1, 50)

            area = 0
            
            if len(CP)>2:
             
                hull = ConvexHull(CP)

                area =  hull.volume

               # print('area', area)
             
                #print('hull', hull)
                        
                for zz  in range(len(n)):

                    if abs(t-n[zz])<0.1:
                       vv = 00
                     #  self.plott(np.array(CP),hull,zmp, t)

 
            
            self.ZMP = zmp

            self.CP=CP

            #print('self.CP', self.CP)

            fric1 = fric1y*fric1y_dir+fric1x*fric1x_dir
            fric2 = fric2y*fric2y_dir+fric2x*fric2x_dir


            fric = fric1+fric2 #lateral reaction force from ground to feet

            N =N1+N2
            
            F =np.array([0,0,N])+fric

            self.Reaction_force = np.reshape(F,(3,))

            fric_norm = np.linalg.norm(fric)
            
            
            
            
            
            
            X = opt.runOptimization(self.h,self.thdvel, MVrhs)
            
       

            xin = X

            
            X1 = np.array([X[0],X[1],0]) 

            #X1 = np.array([(X[0]),(X[1]),0]) 

            """
            if t<self.t_step+2:

              X1 = np.array([X[0],X[1],0])

            else: 

              X1 = np.array([0.1,0.1,0.1])
            """
            
            Mvrhs = np.reshape(MVrhs,(3,))
  
            xxx=np.real(np.linalg.pinv(self.h)@X1)

            Mvrhsxxx = 1*np.real(np.linalg.pinv(self.h)@Mvrhs)

            xxx = np.reshape(xxx,(6,))
 
           # if t<self.t_stop:
            if t<self.t_impact+self.t_set+self.t_rest:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx)

 

            elif t<self.t_impact+self.t_set+self.t_rest+20:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx) 

            else: 

              self.dthdes_new = 0.0*(np.array(np.real(self.dthdes))-1*xxx)
 
            
            self.dthdes_new = np.real(self.dthdes_new)
            
            self.dthdes_new = np.reshape(self.dthdes_new,(6,))


            
            self.macm_calc  =  self.F_app+F+self.M*np.array([robot.gravityacc])

            self.force_check = self.F_app+F+self.M*np.array([robot.gravityacc])-self.macm_calc
            
            self.force_check = np.reshape(self.force_check,(3,))
            
            Marhs = np.reshape(Marhs,(3,1))

            self.macm_calc = np.reshape(self.macm_calc,(3,1))

            self.thddacc = np.reshape(self.thddacc,(6,1))

            self.F_app = np.reshape(self.F_app,(3,1))

            dth_ = np.reshape(dth,(6,1))

            self.h_dot = -(self.h@ self.thddacc+ Marhs-self.macm_calc)@np.linalg.pinv([dth]).T



            Cx = [1,0,0]
            Cy = [0,1,0]
            xtild = [0.15,0,0]
            ytild = [0,0.15,0]

            Cx= np.reshape(Cx,(1,3))
            Cy= np.reshape(Cy,(1,3))

            xtild= np.reshape(xtild,(3,1))
            ytild= np.reshape(ytild,(3,1))
            
            
            

            if abs(self.Reaction_force[0])> abs(robot.mu*N/np.sqrt(1)):

               a1i =  str('friction limit exceeded_1')

               frx = Cx@(self.h@ self.thddacc+ self.h_dot@dth_+Marhs-self.F_app)



               if frx>0:

                self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs+robot.mu*N+self.F_app-xtild)

               else:

                self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs-robot.mu*N+self.F_app+xtild)    

               #self.dthdes_new = np.real(self.dthdes_new)+xxx


                       
            else:

                a1i =  str('friction ok_1')

 

            
            if abs(self.Reaction_force[1])> abs(robot.mu*N/np.sqrt(1)):

                a2i =  str('friction limit exceeded_2')

                fry = Cy@(self.h@ self.thddacc+ self.h_dot@dth_+Marhs-self.F_app)

 

                if fry>0:

                 self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs+robot.mu*N+self.F_app-ytild)

                else:

                 self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs-robot.mu*N+self.F_app+ytild)  

                #self.dthdes_new = np.real(self.dthdes_new)+xxx

 
                       
            else:

                a2i =  str('friction ok_2')

  
            
            self.fric_info =  np.array([t,a1i,a2i])

            
            
            
            
            p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes ,targetVelocities= self.dthdes_new)

           # p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes,
                                   # targetVelocities=self.dthdes, forces=np.zeros(len(robot.rev))) 
                                    # positionGains = np.array(len(self.dthdes)*[20]) ,
                                    # velocityGains = np.array(len(self.dthdes)*[1.65]))
            
            
            #tau = p.calculateInverseDynamics(robot.robot_id, list(self.thdes), list(self.dthdes), list(abar), flags =1)

            #tau = np.reshape(tau,(15,))

            #self.jtorques = tau[7:15]

            jointstates = p.getJointStates(robot.robot_id,robot.rev)

            #jointstates = p.getJointStates(robot.robot_id,robot.rev)

            self.thpos_deg =  [(state[0]*1*180/np.pi) for state in jointstates]

         #   print("self.thpos_deg", self.thpos_deg)

            u2d2commcode.update_servo_pos(self.thpos_deg[0], self.thpos_deg[1], self.thpos_deg[2], -self.thpos_deg[3], self.thpos_deg[4], self.thpos_deg[5])



            self.jtorques = self.applied_torques

            

          
            
            #tau = p.calculateInverseDynamics(robot.robot_id, self.thpos, self.thdvel, abar, flags =1)
            
           # tau_g = p.calculateInverseDynamics(robot.robot_id, list(self.thdes), [0.0] * len(robot.rev), [0.0] * len(robot.rev), flags =1)
            
           # tau = np.reshape(tau,(15,))
           # tau_g = np.reshape(tau_g,(15,))
          
            
            """
            p.setJointMotorControlArray(robot.robot_id, robot.rev,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = tau[7:15])
            """
            
            ##ankle torque control
            
            """
            if t<self.t_set+ self.t_rest + self.t_impact+ 1:

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[3] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  self.jtorques[3] - ankle_torque_2)

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[7] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  self.jtorques[7] - ankle_torque_1) 
            """

            ##ankle angle control
            
            """
            if t<self.t_set+ self.t_rest + self.t_impact+ 5:

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[3] ,
                                        controlMode = p.POSITION_CONTROL, 
                                        targetPosition = 0)

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[7] ,
                                        controlMode = p.POSITION_CONTROL, 
                                        targetPosition = 0) 
            """
            

            phimax = 0 # works for Fx = 5, Fy =38, mu =0.9 for both
            T1 =20
            T2 = 1000
            
            ## HIp torque control
            """
            if area<1e-8:

                if t<T1:

                    phi = 0*(-2*phimax*t*t*t*(1/(T1*T1*T1)) + 3*phimax*t*t*(1/(T1*T1)))

                    p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.POSITION_CONTROL,targetPosition = phi)

                    p.setJointMotorControl2(robot.robot_id,robot.rev[4],p.POSITION_CONTROL,targetPosition = phi)

                else:

                    phi = 0*(2*phimax*t*t*t*(1/(T2*T2*T2)) - 3*phimax*t*t*(1/(T2*T2))) + phimax

                    phi = 1*phi

                    p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.POSITION_CONTROL,targetPosition = phi)

                    p.setJointMotorControl2(robot.robot_id,robot.rev[4],p.POSITION_CONTROL,targetPosition = phi)
            """
           ## HIp torque control 
            """    
            if t<self.t_impact+0.2:

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[0] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  0*self.jtorques[0])

               p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[4] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =   0*self.jtorques[4]) 
            """
            """
            p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[0] ,
                                        controlMode = p.VELOCITY_CONTROL, 
                                        targetVelocity=-0.1)
            """
            
            
            
            
            
            self.write_data()
    
    def checkH(self):

        thd=p.getJointStates(robot.robot_id,robot.rev)
      

        #thdfixed = p.getJointStates(robot.robot_id,[0,2,4,8,10,12])

        #thdfixed = p.getJointStates(robot.robot_id,[0,1,3,5,9,11,13,16])
        
        """
        self.thdfixedvel=[] 
        for i in range(len(thdfixed)):
        
            self.thdfixedvel.append(thdfixed[i][1])
        
        """
        self.thdvel=[]

        for i in range(len(thd)):
        
            self.thdvel.append(thd[i][1])
        
        self.thpos=[]

        for i in range(len(thd)):
        
            self.thpos.append(thd[i][0])
        
        #self.dthdesired = []

        #for i in range(len(thd)):
        
            #self.dthdesired.append(dthdes[i])

        #self.thdvelocity = np.array()    

        #self.jointvelerr = self.dthdes.T-self.thdvel

        #self.thdvel =np.reshape(self.thdvel,8)

        self.dthdes  = np.real(self.dthdes)

        self.dthdes = np.reshape(self.dthdes,(6,))

        self.thdes = np.reshape(self.thdes,(6,))

        self.jointvelerr = self.dthdes.T-self.thdvel

        self.jointposerr = self.thdes.T-self.thpos

        #self.jointvelerr = self.dthdes.T

       

        robot.getCOMPos()
      

        self.M = np.sum(robot.setLinkM())
     
        self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])



        
        #dth = np.delete(robot.qdot,[0,4])

        self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[7]*self.v_torso/(robot.m_array[0]+robot.m_array[7]))           
          
        

        joints = p.getJointStates(robot.robot_id,robot.rev)
        self.q = [ i[0] for i in joints]
        self.qdot = [ i[1] for i in joints]
        self.applied_torques = [i[3] for i in joints]
        dth=np.array(self.qdot)
        #dthact=dth.T
        self.jtorques = self.applied_torques
       

        qnorm=np.linalg.norm(robot.urdf_orien[0])
   
         
        qdotall = self.getJointState()[1]

        dthall =  np.array(qdotall)
       
        

        vbase_l= self.v_base+self.skewM(robot.link_omega[1])@(robot.urdf_pos[1]-robot.com_base_pos)
        vbase_r= self.v_base+self.skewM(robot.link_omega[4])@(robot.urdf_pos[4]-robot.com_base_pos)

            #x1=vbase_l+self.skewM(robot.link_omega[10])@(-robot.urdf_pos[10]+robot.com_pos[10])
  

            #Mr = np.sum(robot.setLinkM()[1:8])
            #Ml = np.sum(robot.setLinkM()[9:16])
        Mr = np.sum(robot.setLinkM()[1:4])
        Ml = np.sum(robot.setLinkM()[5:7])

            #M_base= robot.setLinkM()[0]+robot.setLinkM()[17]+robot.setLinkM()[1]+robot.setLinkM()[9]
        M_base= robot.m_array[0]+robot.m_array[7]
        
        modi_link_mass = robot.setLinkM()
        modi_link_vel = np.append(robot.link_vel,[self.v_torso],axis=0)
        vcmdata=robot.getrobotCOMvelocity(modi_link_mass, modi_link_vel)

        #vrightcm = vcmdata[0]; vleftcm = vcmdata[1]; vbasecm = vcmdata[2]; vtotalcm =  vcmdata[3]
        
        
        #link_vel1= robot.base_vel+self.skewM(robot.base_angvel)@(robot.com_pos[0]-robot.com_base_pos)
        #link_vel1= robot.base_vel+np.matmul(self.skewM(robot.base_angvel),(robot.com_pos[0]-robot.com_base_pos))

        #link_vel= robot.link_vel[0]+self.skewM(robot.link_omega[0])@(robot.urdf_pos[1]-robot.com_pos[0])+self.skewM(robot.link_omega[1])@(-robot.urdf_pos[1]+robot.com_pos[1])
        #link_vel= robot.link_vel[2]+self.skewM(robot.link_omega[2])@(robot.urdf_pos[3]-robot.com_pos[2])+self.skewM(robot.link_omega[3])@(-robot.urdf_pos[3]+robot.com_pos[3])
        
        
        
        link_vell = np.append(robot.link_vel,[self.v_torso],axis=0)

        link_pos = np.append(robot.com_pos,[robot.com_base_pos],axis=0)
    
        mv=np.zeros((8,3))

        mc = np.zeros((8,3))

        print('link_vell.shape', link_vell.shape)  
        for i in range(len(robot.m_array)):
          mv[i,:]=robot.m_array[i]*link_vell[i,:]
          mc[i,:]=robot.m_array[i]*link_pos[i,:]
        
     
        
        mvcmx=0
        #for i in range(len(robot.m_array)):
        mvcmx=sum(mv[:,0])
        mvcmy=sum(mv[:,1])
        mvcmz=sum(mv[:,2])

        mccmx=0
        #for i in range(len(robot.m_array)):
        mccmx=sum(mc[:,0])
        mccmy=sum(mc[:,1])
        mccmz=sum(mc[:,2])
        
        #m_vcm=np.matrix([[mvcmx],[mvcmy], [mvcmz]])
        m_vcm=np.array([mvcmx,mvcmy,mvcmz])
        m_ccm=np.array([mccmx,mccmy,mccmz])
       
       

       #m_vcm = np.sum(np.tile(robot.m_array,(3,1))*link_vell.T,axis=1)

        self.v_cm = m_vcm/self.M

        self.c_cm = m_ccm/self.M

        
        return self.c_cm

    def checkA(self):
        robot.getJointState()
        robot.getCOMPos()
        cj.Dynamics.local_com_pos = robot.local_com_pos
        cj.Dynamics.com_pos = robot.com_pos
        cj.Dynamics.urdf_orien = robot.urdf_orien
        cj.Dynamics.urdf_pos = robot.urdf_pos
        cj.Dynamics.com_base_pos = np.array(robot.com_base_pos)
        cj.Dynamics.m_array = robot.m_array
        
       
        L = cj.Dynamics.getL(self,link_mass=robot.m_array, link_vel=robot.link_vel, link_omega=robot.link_omega, link_com_pos = robot.com_pos, com_base_pos=robot.com_base_pos, base_ori=robot.base_ori, base_vel=robot.base_vel, base_angvel=robot.base_angvel)
     
        dth = np.array(robot.qdot)
        r = cj.Dynamics.com_base_pos
      
        self.Alpha = np.array([L - 1*self.M*self.skewM(r)@self.v_torso]).T*np.linalg.pinv([dth]).T
        #self.Alpha = np.array([L-self.M*robot.dynamics.skewM(r)*self.v_torso])
        #self.Alpha = (L - self.M*robot.dynamics.skewM(r)@self.v_torso).T*np.linalg.pinv([dth]).T
       
        return self.Alpha

    def write_data(self):

        filename0 = "Time.csv"

        filename1 = "vcm.csv"
        filename2 = "jtorques.csv"
        filename3 = "h_err.csv"
        filename4 = "jvel.csv"
        filename5 = "omegaerr.csv"
        filename6 = "linvelerr.csv"
        filename7 = "jointvelerror.csv"
        filename8 = "jointposerror.csv"
        filename9 = "jointposition.csv"
        filename10 = "jointvelocity.csv"
        filename11 = "cm.csv"
        filename13 = "ReactionForce.csv"
        filename14 = "Frictioninfo.csv"
        filename16 = "ZMP.csv"
        filename17 = "CP.csv"

        if(self.file_written):
            write_type = 'a'
        else:
            write_type = 'w'
            self.file_written = True
        # writing to csv file
        
        with open(filename0, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.timee) 

        with open(filename1, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.v_cm)
        
        with open(filename2, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.jtorques)
        
        with open(filename3, write_type) as csvfile:
            writer = csv.writer(csvfile)
         #   writer.writerow(self.h)

        with open(filename4, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.thdvel) 

        
        with open(filename5, write_type) as csvfile:
            writer = csv.writer(csvfile)
           # writer.writerow(self.omega_err) 
            #writer.writerow(self.omega_err)
        

        with open(filename6, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.actualmomentumfromh)   
            #writer.writerow(self.momentumerror)    
        
        with open(filename7, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.jointvelerr)

        with open(filename8, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.jointposerr)   
        
        with open(filename9, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.thpos)   

        with open(filename10, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.thdvel)    

        with open(filename11, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.c_cm)  

        with open(filename13, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.Reaction_force)

        with open(filename14, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.fric_info)

        with open(filename16, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.ZMP)  

        with open(filename17, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.CP)        


if __name__ == "__main__":
    robot = Biped()
   # traj = GenTraj()
    s1 = BipedSim()
   # p1 = np.array([-0.17,0.17,-0.05])
   ## p2 = np.array([ 0.07,-0.07,0.1])
   # p3 = np.array([-0.17,0.17,-0.05])
    #traj.setTrajPt(**{"p1":p1,"p2": p2,"p3": p3})
    s1.runSim()
    

