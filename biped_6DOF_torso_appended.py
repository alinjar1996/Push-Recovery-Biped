from typing import Optional
from matplotlib.pyplot import flag
import pybullet as p
import pybullet_data
import numpy as np
import time
#from comjacob import Dynamics

class Biped:
    def __init__(self) -> None:
        #self.dynamics = Dynamics()
        self.m = 0
        self.startSim()
        self.setLinkM()
        self.allj= np.array([0,1,2,3,4,5,6])
        self.rev = np.array([1,2,3,4,5,6]) 
        #1 Rhip Yaw 3 Rhip Pitch 5 RKnee Pitch 6 RAnkle Pitch
       # self.cjoint = np.array([3,5,6,11,13,14])
        self.lineptr = None
        self.textptr = None
        self.pointptr = None
        #self.jdetails = 
        self.z = [0 for _ in range(len(self.rev))]
        self.fric_coeff=p.getDynamicsInfo(self.planeID ,  -1)[1]
        self.res_coeff=p.getDynamicsInfo(self.planeID ,  -1)[5]
        
        #print("res_coeff", self.res_coeff) 

    def startSim(self,table=True):
        self.physicsclient = p.connect(p.GUI)
        #p.setGravity(0,0,-10)
        self.gravityacc = (0,0,-9.81)
        p.setGravity(0,0,-9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeID = p.loadURDF("plane.urdf")
       #/home/udayan/Desktop/Alinjar/Biped Pybullet/bipedal_20_07/bipedal_12_01/
        self.urdf_path = "BIPED_6DOF_Torso_appended.urdf" #bipedal_2 has worked great so far
        #self.urdf_path = "/home/udayan/Desktop/Alinjar/Biped Pybullet/bipedal_24_11-20220809T113418Z-001/bipedal_24_11/bipedalmodi.urdf"
        #self.robot_id = p.loadURDF(self.urdf_path,useFixedBase= False, basePosition=[0,0,0])
        #self.robot_id = p.loadURDF(self.urdf_path,useFixedBase= False, basePosition=[0,0,0]) #this is original
        alp = np.pi/2
        self.robot_id = p.loadURDF(self.urdf_path,useFixedBase= False, basePosition=[0,0,0.000], baseOrientation = [np.sin(alp/2)*np.cos(np.pi/2),np.sin(alp/2)*np.cos(np.pi/2),np.sin(alp/2)*np.cos(0),np.cos(alp/2)])
      #  self.robot_id_fixed = p.loadURDF(self.urdf_path,useFixedBase= True, basePosition=[0,0,0])
        self.mu =0.6 #0.2 for no sliding result
        p.changeDynamics(self.planeID ,  -1, lateralFriction=self.mu) #keep it 0.6 for no tipping result
        p.changeDynamics(self.planeID ,  -1, restitution=0)

    def setLinkM(self):
        """ mass of base + motor_r + motor_l""" #total torso/base
        linkmass_data = []
        for i in range(7):
          linkmass_data.append(p.getDynamicsInfo(self.robot_id,  i)[0])

        self.m_base = p.getDynamicsInfo(self.robot_id,  -1)[0]
        #print("base_mass", self.m_base)
        self.m_array = np.append(linkmass_data, self.m_base)

        M = np.sum(self.m_array)

      #  print (self.m_array)
       # print("M", M)
        #print("Mass", self.m_array)
        #print("Mass_error", self.m_array[0:16]-linkmass_data)

       # link_mass = p.getDynamicsInfo(self.robot_id,  -1)[0]
        #print("self.m_array", self.m_array)

        return self.m_array
       


    def getCOMPos(self):
        # link_data = p.getLinkStates(self.robot_id,self.cjoint)
        link_data  = p.getLinkStates(self.robot_id,[ i for i in range(7)],computeLinkVelocity=1)
        self.com_pos = np.array([i[0] for i in link_data])
        self.local_com_pos = np.array([i[2] for i in link_data])
        self.urdf_pos = np.array([i[4] for i in link_data]) #origin of ith link: to be confirmed
        #self.urdf_orien = np.array([p.getEulerFromQuaternion(i[5]) for i in link_data])
        self.urdf_orien = np.array([i[5] for i in link_data]) #in quaternion
        self.link_vel = np.array([i[6] for i in link_data])
        self.link_omega = np.array([i[7] for i in link_data])
        self.com_base_pos = p.getBasePositionAndOrientation(self.robot_id)[0]
        #self.base_vel = p.p.getLinkStates(self.robot_id,-1,computeLinkVelocity=1)
        self.base_ori=p.getBasePositionAndOrientation(self.robot_id)[1]
        self.base_vel= np.array(p.getBaseVelocity(self.robot_id)[0])
        self.base_angvel= np.array(p.getBaseVelocity(self.robot_id)[1])
 
    
    def getrobotCOMvelocity(self,modi_link_mass,modi_link_vel):
        
        modi_link_mass_right = modi_link_mass[1:4]

        modi_link_mass_left = modi_link_mass[4:7]

        modi_link_mass_base =  modi_link_mass[0]+modi_link_mass[7]

        modi_link_vel_right = modi_link_vel[1:4]

        modi_link_vel_left = modi_link_vel[4:7]

        modi_link_vel_base = (modi_link_mass[0]*modi_link_vel[0]+modi_link_mass[7]*modi_link_vel[7])/(modi_link_mass[0]+modi_link_mass[7])


        Mr= np.sum(modi_link_mass_right); Ml= np.sum(modi_link_mass_left); M=np.sum(modi_link_mass)

        M_base =  modi_link_mass_base

        mv=np.zeros(modi_link_vel.shape); mvr=np.zeros(modi_link_vel_right.shape); mvl=np.zeros(modi_link_vel_left.shape)


        for qr in range(len(modi_link_mass_right)):

          mvr[qr,:]=modi_link_mass_right[qr]*modi_link_vel_right[qr,:]
        
        for ql in range(len(modi_link_mass_left)):

          mvl[ql,:]=modi_link_mass_left[ql]*modi_link_vel_left[ql,:]
        
        for q in range(len(modi_link_mass)):

          mv[q,:]=modi_link_mass[q]*modi_link_vel[q,:]
        
        #print("mvsize", mv.shape)
        
        #mvcmx=0
        #for i in range(len(robot.m_array)):
        mvcmx=sum(mv[:,0]); mvcmy=sum(mv[:,1]); mvcmz=sum(mv[:,2])

        vcmx= mvcmx/M;  vcmy= mvcmy/M;  vcmz= mvcmz/M

        m_vcm=np.array([mvcmx,mvcmy,mvcmz]);  vcm=np.array([vcmx,vcmy,vcmz])

        mvrcmx=sum(mvr[:,0]); mvrcmy=sum(mvr[:,1]); mvrcmz=sum(mvr[:,2])

        vrcmx= mvrcmx/Mr;  vrcmy= mvrcmy/Mr;  vrcmz= mvrcmz/Mr

        m_vrcm=np.array([mvrcmx,mvrcmy,mvrcmz]);  vrcm=np.array([vrcmx,vrcmy,vrcmz])

        
        mvlcmx=sum(mvl[:,0]); mvlcmy=sum(mvl[:,1]); mvlcmz=sum(mvl[:,2])

        vlcmx= mvlcmx/Ml;  vlcmy= mvlcmy/Ml;  vlcmz= mvlcmz/Ml

        m_vlcm=np.array([mvlcmx,mvlcmy,mvlcmz]);  vlcm=np.array([vlcmx,vlcmy,vlcmz])
        
        #m_vcm=np.matrix([[mvcmx],[mvcmy], [mvcmz]])
       
        vbase= modi_link_vel_base
        #print("shape_mvcm",m_vcm.shape)
        return vrcm, vlcm, vbase, vcm



    def drawPb1(self, point):
        if self.lineptr != None:
                p.removeUserDebugItem(self.lineptr)
                # pass
        self.lineptr = p.addUserDebugLine(point,[0,0,0],lineColorRGB=[1,0,0],lineWidth=5)

    def drawPb2(self, point):
        if self.lineptr != None:
                p.removeUserDebugItem(self.lineptr)
                # pass
        self.lineptr = p.addUserDebugLine(point,[0,0,0],lineColorRGB=[0,1,0],lineWidth=5) 

    def drawPbPoint(self, point):
       # if self.lineptr != None:
             #   p.removeUserDebugItem(self.lineptr)
                # pass
        self.lineptr = p.addUserDebugLine(point,point+[0.001,0.001,0.001],lineColorRGB=[1,0,0],lineWidth=5)  

    def drawPbPoint2(self, point):
       # if self.lineptr != None:
             #   p.removeUserDebugItem(self.lineptr)
                # pass
        self.pointptr = p.addUserDebugPoints([point,point],pointColorsRGB=[[1,0,0],[1,0,0]])       

    def drawPbText(self, text):
        if self.textptr != None:
                p.removeUserDebugItem(self.textptr)
                # pass
        self.textptr = p.addUserDebugText(text,textPosition = [0,0,0.5], textColorRGB = [1,0,0],textSize = 1)   
    
          
    
    def setInitialState(self):
        for _ in range(100):
            p.setJointMotorControlArray(self.robot_id,self.rev,controlMode=p.POSITION_CONTROL, targetPositions=self.z)
            time.sleep(1/100)
    
    def turnOffActuators(self):
        p.setJointMotorControlArray(self.robot_id,self.rev,controlMode=p.VELOCITY_CONTROL, forces= self.z)
    
    def turnOffDamping(self):
        for i in self.rev:
            p.changeDynamics(self.robot_id, i, linearDamping=0.0, angularDamping=0.0, jointDamping=0.0)
    
    def getJointState(self):
        joints = p.getJointStates(self.robot_id,self.rev)
        self.q = [ i[0] for i in joints]
        self.qdot = [ i[1] for i in joints]
        self.applied_torques = [i[3] for i in joints]
        
      #  print("qdot:",self.qdot)
        
        self.jdetails=[]
        #for i in joints:
        for ii in range(len(self.rev)):
          self.jdetails.append(p.getJointInfo(self.robot_id,ii)[13])
      
      #  print("jdetails:",self.jdetails)
       # print("jd",self.jdetails)
        alljoints = p.getJointStates(self.robot_id, self.allj)
        self.qall = [ i[0] for i in alljoints]
        self.qdotall = [ i[1] for i in alljoints]


        #jointinfo = p.getJointInfo(self.robot_id,0)

        #print('joininfo', self.jdetails)
        #print('joininfolength', len(self.jdetails))

        #jointt = p.getJointInfo(self.robot_id,2)
        #print("jointt", jointt)

        return self.q

    def massMatrix(self):

        joints = p.getJointStates(self.robot_id,self.rev)
        self.q = [ i[0] for i in joints]
        
        mass_matrix = np.asarray(p.calculateMassMatrix(self.robot_id,self.q))

        mass_matrix_short = mass_matrix [6:15,6:15]

        #print("mass_matrix_short", mass_matrix_short.shape)

      
       # print("mass_matrix", mass_matrix.shape)

        mass_matrixinv = np.linalg.inv(mass_matrix_short)

       # print("mass_matrixinv", mass_matrixinv.shape)

        return mass_matrixinv
    
    def coriolisVector(self):
        """
        Why use Flag: https://github.com/bulletphysics/bullet3/issues/3188
        """
        coriolis_gravity_vector = np.array(p.calculateInverseDynamics(self.robot_id,objPositions = self.angles,objVelocities = self.w,objAccelerations = self.z,flags = 1))
        coriolis = coriolis_gravity_vector - self.gravityVector()

        return coriolis

    def gravityVector(self):
        
        gravity_vector = p.calculateInverseDynamics( self.robot_id, objPositions = self.angles,objVelocities = self.z,objAccelerations = self.z, flags= 1)
        gravity_vector = np.array(gravity_vector)
        
        return gravity_vector
        
if __name__ == '__main__':
    
   
    b = Biped()
    time.sleep(10)
    #b.getCOMPos()
    b.getJointState()
  #  b.setLinkM()
    #b.massMatrix()
  #  b.setJointOrin()
   # b.getP()
   # b.R(np.array([0,0,0.5]), np.array([1,1,0.1]))
   # b.getH()