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
      #  self.setLinkM()
        self.allj= list(range(0,49))
        self.rev = np.array([4,7,12,16,17,24,27,30,35,39,40,47]) 
        #self.cjoint = np.array([3,5,6,11,13,14])
        self.lineptr = None
        self.textptr = None
        self.pointptr = None
        #self.jdetails = 
        self.z = [0 for _ in range(len(self.rev))]
        self.fric_coeff=p.getDynamicsInfo(self.planeID ,  -1)
        #print("friction coeff", self.fric_coeff[1]) 

    def startSim(self,table=True):
        self.physicsclient = p.connect(p.GUI)
        self.gravityacc = (0,0,-10)
        p.setGravity(0,0,-10)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeID = p.loadURDF("plane.urdf")
        self.urdf_path = "Biped.urdf"
        self.robot_id = p.loadURDF(self.urdf_path,useFixedBase= False, basePosition=[0,0,0])
        self.mu =0.6 #0.2 for no sliding result
        p.changeDynamics(self.planeID ,  -1, lateralFriction=self.mu) #keep it 0.6 for no tipping result
        p.changeDynamics(self.planeID ,  -1, restitution=0)
       # p.resetDebugVisualizerCamera(cameraDistance = 0.75, cameraYaw= 45, cameraPitch = 0, cameraTargetPosition = [2,2,0.4])
  
    def equicmatrix(self,c_vec):
        
        cx  = c_vec[0]
        cy  = c_vec[1]
        cz  = c_vec[2]

        csquareequiv= np.matrix ([[cy*cy+cz*cz, -cx*cy, -cx*cz], [-cx*cy, cx*cx+cz*cz, -cy*cz], 
                                 [-cx*cz, -cy*cz, cx*cx+cy*cy]])
        return  csquareequiv    

    
       
    #def skewsymmetric(self,A):

        #self.B = np.matrix([0])
    def getJointState(self):
        joints = p.getJointStates(self.robot_id,self.rev)
        self.q = [ i[0] for i in joints]
        self.qdot = [ i[1] for i in joints]
        self.applied_torques = [i[3] for i in joints]
        
      #  print("qdot:",self.qdot)
        
        self.jdetails=[]
        #for i in joints:
        for ii in range(len(self.rev)):
          self.jdetails.append(p.getJointInfo(self.robot_id,ii)[1])
      
      #  print("jdetails:",self.jdetails)
        #print("jd",self.jdetails)
        alljoints = p.getJointStates(self.robot_id, self.allj)
        self.qall = [ i[0] for i in alljoints]
        self.qdotall = [ i[1] for i in alljoints]


        #jointinfo = p.getJointInfo(self.robot_id,0)

        #print('joininfo', self.jdetails)
        #print('joininfolength', len(self.jdetails))

        #jointt = p.getJointInfo(self.robot_id,2)
        #print("jointt", jointt)

        return self.q
    
    def setLinkM(self):
        """ mass of base + motor_r + motor_l""" #total torso/base
        linkmass_data = []
        for i in range(49):
          linkmass_data.append(p.getDynamicsInfo(self.robot_id,  i)[0])

        self.m_base = p.getDynamicsInfo(self.robot_id,  -1)[0]
        #print("base_mass", self.m_base)
        self.m_array = np.append(linkmass_data, self.m_base)

       # print("self.m_array", self.m_array)

       # M = np.sum(self.m_array)

       # print (np.shape(self.m_array))
       # print("M", M)
       # print("Mass", self.m_array)
        #print("Mass_error", self.m_array[0:16]-linkmass_data)

       # link_mass = p.getDynamicsInfo(self.robot_id,  -1)[0]
        #print("self.m_array", self.m_array)

        return self.m_array

    def getrobotCOMvelocity(self,modi_link_mass,modi_link_vel):
        
        modi_link_mass_right = modi_link_mass[4:26]

        modi_link_mass_left = modi_link_mass[27:49]
        
        modi_link_mass_base =  modi_link_mass[0]+ modi_link_mass[1]+ modi_link_mass[2]+ modi_link_mass[3]+modi_link_mass[26]+modi_link_mass[49]

        modi_link_vel_right = modi_link_vel[4:26]

        modi_link_vel_left = modi_link_vel[27:49]

        modi_link_vel_base =  (modi_link_mass[0]*modi_link_vel[0]+ modi_link_mass[1]*modi_link_vel[1]+ modi_link_mass[2]*modi_link_vel[2]+modi_link_mass[3]*modi_link_vel[3]+modi_link_mass[26]*modi_link_vel[26]+modi_link_mass[49]*modi_link_vel[49])/ modi_link_mass_base


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
     

    def getCOMPos(self):
        # link_data = p.getLinkStates(self.robot_id,self.cjoint)
        link_data  = p.getLinkStates(self.robot_id,[ i for i in range(49)],computeLinkVelocity=1)
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
       # linkmass_data  = p.getDynamicsInfo(self.robot_id,  0)

        self.linkinfo = []

        self.com_base_pos = np.array(self.com_base_pos)

        """
        print("link_length_hip_r", np.linalg.norm(self.urdf_pos[12]-self.urdf_pos[4]))
        print("link_length_thigh_r", np.linalg.norm(self.urdf_pos[16]-self.urdf_pos[12]))
        print("link_length_shank_r", np.linalg.norm(self.urdf_pos[17]-self.urdf_pos[16]))

        
        print("link_length_hip_l", np.linalg.norm(self.urdf_pos[35]-self.urdf_pos[27]))
        print("link_length_thigh_l", np.linalg.norm(self.urdf_pos[39]-self.urdf_pos[35]))
        print("link_length_shank_l", np.linalg.norm(self.urdf_pos[40]-self.urdf_pos[39]))
        """
        #print("Two leg Hip joint distance", np.linalg.norm(self.urdf_pos[27]-self.urdf_pos[4]))
        #self.drawPb1(self.urdf_pos[27])
        #self.drawPb1(self.urdf_pos[4])

       # time.sleep(100)

       # print("self.com_base_pos", self.com_base_pos)

        j=0

        while j<49:
        #self.linkinfo =  p.getDynamicsInfo(self.robot_id, [i for i in range(49)])
          #print("j", j)
          self.linkinfo.append(p.getDynamicsInfo(self.robot_id, j)[0])
          j += 1
        
       # print("self.linkinfo", self.linkinfo)

        base_mass = p.getDynamicsInfo(self.robot_id, -1)[0]

        #total_mass_array = np.append([base_mass],self.linkinfo,axis=0)

        total_mass_array = self.linkinfo



        #print("total_mass_array", np.sum(total_mass_array))

        ## calculating link masses of 12 DOF biped

        Torso_mass = base_mass+ np.sum(total_mass_array[0:4])+total_mass_array[26]

        Lhip_mass_1 = np.sum(total_mass_array[4:7])
        Lhip_mass_2 = np.sum(total_mass_array[7:12])
        Lhip_mass = Lhip_mass_1 + Lhip_mass_2
        Lthigh_mass = np.sum(total_mass_array[12:16])
        Lshank_mass = np.sum(total_mass_array[16:17])
        Lankle_mass_1 = np.sum(total_mass_array[17:24])
        Lankle_mass_2 = np.sum(total_mass_array[24:26])
        Lfoot_mass = Lankle_mass_1 + Lankle_mass_2
        
        """
        print("left hip mass", Lhip_mass)
        print("left thigh mass", Lthigh_mass)
        print("left shank mass", Lshank_mass)
        print("left foot mass",Lfoot_mass)
        """

        Rhip_mass_1 = np.sum(total_mass_array[27:30])
        Rhip_mass_2 = np.sum(total_mass_array[30:35])
        Rhip_mass = Rhip_mass_1 + Rhip_mass_2
        Rthigh_mass = np.sum(total_mass_array[35:39])
        Rshank_mass = np.sum(total_mass_array[39:40])
        Rankle_mass_1 = np.sum(total_mass_array[40:47])
        Rankle_mass_2 = np.sum(total_mass_array[47:49])
        Rfoot_mass = Rankle_mass_1 +  Rankle_mass_2
        """
        print("Right hip mass", Rhip_mass)
        print("Right thigh mass", Rthigh_mass)
        print("Right shank mass", Rshank_mass)
        print("Right foot mass",Rfoot_mass)
        """
        total_mass_array_12dof = [Lhip_mass_1,Lhip_mass_2, Lthigh_mass, Lshank_mass, Lankle_mass_1, Lankle_mass_2,Rhip_mass_1, Rhip_mass_2, Rthigh_mass, Rshank_mass, Rankle_mass_1, Rankle_mass_2, Torso_mass]

        self.m_array = np.append(total_mass_array, base_mass)

        
      #  print("total_mass_array_12dof", total_mass_array_12dof)

       # print("sum_total_mass_array_12dof", np.sum(total_mass_array_12dof))
        total_mass_array_8dof = [Lhip_mass_1+Lhip_mass_2, Lthigh_mass, Lshank_mass, Lankle_mass_1+Lankle_mass_2,Rhip_mass_1+Rhip_mass_2, Rthigh_mass, Rshank_mass, Rankle_mass_1+Rankle_mass_2, Torso_mass]
        
       # print("total_mass_array_8dof", total_mass_array_8dof)
       # print("Lhip_mass_1",Torso_mass)
       # self.mass = np.array(i[0] for i in self.linkinfo)

        #print("total_mass", np.sum(total_mass_array_8dof))
        #print("total_mass", np.sum(total_mass_array_12dof))
        
        base_com = (base_mass* self.com_base_pos + total_mass_array[0]*self.com_pos[0]+total_mass_array[1]*self.com_pos[1]+total_mass_array[2]*self.com_pos[2]+total_mass_array[3]*self.com_pos[3]+total_mass_array[26]*self.com_pos[26])/(Torso_mass)

        #print ("base_com", base_com)
        
        Lhip_com =  (total_mass_array[4]*self.com_pos[4]+total_mass_array[5]*self.com_pos[5]+total_mass_array[6]*self.com_pos[6]+
                       total_mass_array[7]*self.com_pos[7]+total_mass_array[8]*self.com_pos[8]+total_mass_array[9]*self.com_pos[9]+
                       total_mass_array[10]*self.com_pos[10]+total_mass_array[11]*self.com_pos[11])/(Lhip_mass_1+Lhip_mass_2)

        #print ("Left Hip COM", Lhip_com)

        Lthighcom =  (total_mass_array[12]*self.com_pos[12]+total_mass_array[13]*self.com_pos[13]+
                       total_mass_array[14]*self.com_pos[14]+total_mass_array[15]*self.com_pos[15])/(Lthigh_mass)
        
        #print("Left thigh COM", Lthighcom)

        Lshankcom =    total_mass_array[16]*self.com_pos[16]/(Lshank_mass)  

        #print(" Left shank COM", Lshankcom)

        Lfoot_com =  (total_mass_array[17]*self.com_pos[17]+total_mass_array[18]*self.com_pos[18]+total_mass_array[19]*self.com_pos[19]+
                      total_mass_array[20]*self.com_pos[20]+total_mass_array[21]*self.com_pos[21]+total_mass_array[22]*self.com_pos[22]+
                      total_mass_array[23]*self.com_pos[23]+total_mass_array[24]*self.com_pos[24]+total_mass_array[25]*self.com_pos[25])/(Lankle_mass_1+Lankle_mass_2)
        #print("Left foot COM", Lfoot_com)

        Rhip_com =  (total_mass_array[27]*self.com_pos[27]+total_mass_array[28]*self.com_pos[28]+total_mass_array[29]*self.com_pos[29]+
                       total_mass_array[30]*self.com_pos[30]+total_mass_array[31]*self.com_pos[31]+total_mass_array[32]*self.com_pos[32]+
                       total_mass_array[33]*self.com_pos[33]+total_mass_array[34]*self.com_pos[34])/(Rhip_mass_1+Rhip_mass_2)
        #print ("Right Hip COM", Rhip_com)

        Rthighcom =  (total_mass_array[35]*self.com_pos[35]+total_mass_array[36]*self.com_pos[36]+
                       total_mass_array[37]*self.com_pos[37]+total_mass_array[38]*self.com_pos[38])/(Rthigh_mass)
        
        #print("Rightthigh COM", Rthighcom)


        Rshankcom =    total_mass_array[39]*self.com_pos[39]/(Rshank_mass)
        #print("Right  shank COM", Rshankcom)

        Rfoot_com =  (total_mass_array[40]*self.com_pos[40]+total_mass_array[41]*self.com_pos[41]+total_mass_array[42]*self.com_pos[42]+
                      total_mass_array[43]*self.com_pos[43]+total_mass_array[44]*self.com_pos[44]+total_mass_array[45]*self.com_pos[45]+
                      total_mass_array[46]*self.com_pos[46]+total_mass_array[47]*self.com_pos[47]+total_mass_array[48]*self.com_pos[48])/(Rankle_mass_1+Rankle_mass_2)
        #print("Right foot COM", Rfoot_com)


        self.inertia_all_matrix=[]

      
        # Base link Inertia matrix
        base_inertia = np.array(p.getDynamicsInfo(self.robot_id , -1)[2])
        base_inertia_matrix = np.diag(base_inertia)
        #print("Base inertia matrix ", base_inertia_matrix)

        #Calculating Inertial matrix of all links
        j=0
        while j<49:
            inertia_data2=np.array((p.getDynamicsInfo(self.robot_id,j)[2]))
            inertia_matrix=np.diag(inertia_data2)
            #print(j)
            #print("inertiadata2:",inertia_matrix)
            self.inertia_all_matrix.append(inertia_matrix)
            j=j+1

        #print("inertia_total_matrix", self.inertia_all_matrix)
     
        

        # Inertia of Left hip 
        MOI_Lhip =  np.zeros((3,3))
    
        for i in range(4,12):
            c_vec_Lhip = Lhip_com - self.com_pos[i]
            c_sqaure_Lhip= self.equicmatrix(c_vec_Lhip)
            I_matrix_Lhip = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_Lhip))))
            MOI_Lhip += I_matrix_Lhip

       # print("MOI of left hip", MOI_Lhip)

       # time.sleep(1)
        
         # Inertia of Left thigh
        MOI_Lthigh =  np.zeros((3,3))

        for i in range(12,16):
            c_vec_Lthigh = Lthighcom - self.com_pos[i]
            c_sqaure_Lthigh= self.equicmatrix(c_vec_Lthigh)
            I_matrix_Lthigh = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_Lthigh))))
            MOI_Lthigh += I_matrix_Lthigh

        #print("MOI of left thigh", MOI_Lthigh)
       # time.sleep(1)

         # Inertia of Left shank
        MOI_Lshank =  np.zeros((3,3))

        for i in range(16,17):
            c_vec_Lshank = Lshankcom - self.com_pos[i]
            c_sqaure_Lshank= self.equicmatrix(c_vec_Lshank)
            I_matrix_Lshank = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_Lshank))))
            MOI_Lshank += I_matrix_Lshank

        #print("MOI of left shank", MOI_Lshank)
       # time.sleep(1)
        
         # Inertia of Left foot
        MOI_Lfoot =  np.zeros((3,3))

        for i in range(17,26):
            c_vec_Lfoot = Lfoot_com - self.com_pos[i]
            c_sqaure_Lfoot= self.equicmatrix(c_vec_Lfoot)
            I_matrix_Lfoot = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_Lfoot))))
            MOI_Lfoot += I_matrix_Lfoot

        #print("MOI of left foot", MOI_Lfoot)
      #  time.sleep(1)

         # Inertia of Right hip
        MOI_Rhip =  np.zeros((3,3))

        for i in range(27,34):
            c_vec_Rhip = Rhip_com - self.com_pos[i]
            c_sqaure_Rhip= self.equicmatrix(c_vec_Rhip)
            I_matrix_Rhip = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_Rhip))))
            MOI_Rhip += I_matrix_Rhip

        #print("MOI of Right hip", MOI_Rhip)
       # time.sleep(1)

         # Inertia of Right thigh
        MOI_Rthigh =  np.zeros((3,3))

        for i in range(35,38):
            c_vec_Rthigh = Rthighcom - self.com_pos[i]
            c_sqaure_Rthigh= self.equicmatrix(c_vec_Rthigh)
            I_matrix_Rthigh = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_Rthigh))))
            MOI_Rthigh += I_matrix_Rthigh

        #print("MOI of Right thigh", MOI_Rthigh)
      #  time.sleep(1)

           # Inertia of Right shank

        MOI_Rshank =  np.zeros((3,3))

        for i in range(39,40):
            c_vec_Rshank = Rshankcom - self.com_pos[i]
            c_sqaure_Rshank= self.equicmatrix(c_vec_Rshank)
            I_matrix_Rshank = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_Rshank))))
            MOI_Rshank += I_matrix_Rshank

        #print("MOI of Right shank", MOI_Rshank)
     #   time.sleep(1)

           # Inertia of Right foot

        MOI_Rfoot =  np.zeros((3,3))

        for i in range(40,49):
            c_vec_Rfoot = Rfoot_com - self.com_pos[i]
            c_sqaure_Rfoot= self.equicmatrix(c_vec_Rfoot)
            I_matrix_Rfoot = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_Rfoot))))
            MOI_Rfoot += I_matrix_Rfoot

        #print("MOI of Right foot", MOI_Rfoot)
     #   time.sleep(1)

           # Inertia of Base



        MOI_base_1 =  np.zeros((3,3))
        MOI_Base =  np.zeros((3,3))

         # Inertia calc for base link
        c_vec_base_1 =base_com - self.com_base_pos
        c_sqaure_base_1= self.equicmatrix(c_vec_base_1)
        I_matrix_B1 = ((base_inertia_matrix)+((base_mass)*(c_sqaure_base_1)))
        #print(I_matrix_B1)

        #Inertia calc for Link-26

        c_vec_base_2 = base_com - self.com_pos[26]
        c_sqaure_base_2= self.equicmatrix(c_vec_base_2)
        I_matrix_B2 = ((self.inertia_all_matrix[26])+((total_mass_array[26])*(c_sqaure_base_2)))
        #print(I_matrix_B2)
        

        #Inertia clc for link 0 to 3

        for i in range(0,4):
            c_vec_base = base_com - self.com_pos[i]
            c_sqaure_base= self.equicmatrix(c_vec_base)
            I_matrix_base = (((self.inertia_all_matrix[i]) +((total_mass_array[i])*(c_sqaure_base))))
            MOI_base_1 += I_matrix_base
        
        MOI_Base = MOI_base_1 + I_matrix_B1 + I_matrix_B2

      #  time.sleep(100)

        #print("MOI of Base", MOI_Base)
     #   time.sleep(1)

        #return  self.m_array
    
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
        self.lineptr = p.addUserDebugLine(point,point-[0.001,0.001,0.001],lineColorRGB=[1,0,0],lineWidth=5)  

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

    def control(self):
        self.thdes = np.zeros((12,1))
        p.setJointMotorControlArray(self.robot_id,self.rev,p.POSITION_CONTROL,targetPositions = self.thdes )
        #time.sleep(100)

if __name__ == '__main__':
    b = Biped()
    b.getCOMPos()
 #   b.setLinkM()
  #  b.control()

   


    



