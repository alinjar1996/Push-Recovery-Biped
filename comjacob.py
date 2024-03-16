
from audioop import lin2adpcm
import numpy as np
import pybullet as p
import pybullet_data
from biped import Biped

class Dynamics:
    def __init__(self) -> None:
      #  self.physicsclient = p.connect(p.GUI)
       # p.setGravity(0,0,-10)
       # p.setAdditionalSearchPath(pybullet_data.getDataPath())
       # self.planeID = p.loadURDF("plane.urdf")
       # self.urdf_path = "/home/udayan/Desktop/Alinjar/Biped Pybullet/bipedal_15_11-20220809T113418Z-001/bipedal_15_11/bipedal.urdf"
       # self.robot_id = p.loadURDF(self.urdf_path,useFixedBase= False, basePosition=[0,0,0])
       # self.urdf_path = "/home/udayan/Desktop/Alinjar/Biped Pybullet/bipedal_15_11-20220809T113418Z-001/bipedal_15_11/bipedal.urdf"
       # self.robot_id = p.loadURDF(self.urdf_path,useFixedBase= False, basePosition=[0,0,0])
        #self.urdf_pos = []
        #self.urdf_orien = []
        #self.local_com_pos = 0
        self.com_pos = []
        #self.com_base_pos = 0
        #self.com_base = []
        #modi_a_link_com = []
        #modi_b_link_com = []
        #modi_a_link_origin = []
        #modi_b_link_origin = []
        #modi_a_link_mass = []
        #modi_b_link_mass = []
    
    def RR(self,quat): #rotfromquat
        q0=quat[0]
        q1=quat[1]
        q2=quat[2]
        q3=quat[3]
        #q0,q1,q2,q3 = quat

        #RR = np.matrix([[1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                       # [2*(q1*q2+q0*q3),  1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1)],
                       # [2*(q1*q3-q0*q2),  2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)]])

        RR = np.array([[1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                        [2*(q1*q2+q0*q3),  1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1)],
                        [2*(q1*q3-q0*q2),  2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)]])


        

   
        # T = np.concatenate((np.vstack((R,[0,0,0])), np.array([[x,y,z,1]]).T), axis=1)
        return RR

    def R(self,orin):
        a,b,g = orin
        r_a = np.array([[np.cos(a), -np.sin(a), 0],
                        [np.sin(a),  np.cos(a), 0],
                        [        0,          0, 1]])

        r_b = np.array([[np.cos(b),   0, np.sin(b)],
                        [        0,   1,         0],
                        [-np.sin(b),  0, np.cos(b)]])

        r_g = np.array([[1,         0,          0],
                        [0, np.cos(g), -np.sin(g)],
                        [0, np.sin(g),  np.cos(g)]])

        R = r_a@r_b@r_g
        # T = np.concatenate((np.vstack((R,[0,0,0])), np.array([[x,y,z,1]]).T), axis=1)
        return R

    def setJointOrin(self,link_orien):
        """

        Joint orientation in global frame

        """
        self.urdf_orien=link_orien

        global j
        j = {
            # for left leg
            'e1a':self.RR(self.urdf_orien[0])@np.array([0.0, 0.0, -1.0]), # hipR joint
            'e2a':self.RR(self.urdf_orien[2])@np.array([1.0, 0.0, 0.0]), # thighR joint
            'e3a':self.RR(self.urdf_orien[4])@np.array([1.0, 0.0, 0.0]), # kneeR joint
            'e4a':self.RR(self.urdf_orien[5])@np.array([1.0, 0.0, 0.0]), # ankleR joint
            # for right leg
            'e1b':self.RR(self.urdf_orien[8])@np.array([0.0, 0.0, -1.0]), # hipL joint
            'e2b':self.RR(self.urdf_orien[10])@np.array([-1.0, 0.0, 0.0]), # thighL joint
            'e3b':self.RR(self.urdf_orien[12])@np.array([-1.0, 0.0, 0.0]), # kneeL joint
            'e4b':self.RR(self.urdf_orien[13])@np.array([-1.0, 0.0, 0.0]), # ankleL joint
        }
        return j


    def vDiffab(a, b):
        return b - a
        
    
        
   
    def skewM(self,e):
        """

        converting vector into a skew symmetric matrix 
        for vector multiplication

        """
        skewM = np.array([[    0,  -e[2],   e[1]],
                          [ e[2],      0,  -e[0]],
                          [-e[1],   e[0],     0]])
        return skewM

    def itensor(self,I):
        return np.array([[I['ixx'],I['ixy'],I['ixz']],
                         [I['ixy'],I['iyy'],I['iyz']],
                         [I['ixz'],I['iyz'],I['izz']]])

    def getH(self, link_com_pos, link_orien, link_pos, com_base_pos, jaxeslocal):
        """

        H gives us the jacobian.

        """
        #self.getlinkdetails()
        #self.getP(link_com_pos, link_orien, link_pos)

        """
        
        p_ij: j th origin to i th CG

        """
        
        """ mass of base + motor_r + motor_l"""
        self.m_base = 0.13857256283713892 
        self.motor_r = 0.328829831780374
        self.motor_l = 0.3288298236863838

        """Two angle links after base"""
        self.m_angle_l = 0.055302620656788114
        self.m_angle_r = 0.055302620656792215

        """Two motors after base"""
        self.m_fixed_motor_l = 0.3642794428368541
        self.m_fixed_motor_r = 0.3642794428368541
        
        """Two knee angles """
        self.m_knee_angle_l = 0.058088429592991084
        self.m_knee_angle_r = 0.058088429592991084

        """Two knee motors"""
        self.m_knee_motor_l = 0.3655195920887555
        self.m_knee_motor_r = 0.3655195920887556

        """Two below knee angles"""
        self.m_below_knee_angle_l = 0.07171879062702044
        self.m_below_knee_angle_r = 0.07171879062702044

        """Ankle Motors"""
        self.m_ankle_motor_l = 0.3723917469007978
        self.m_ankle_motor_r = 0.3723917469007978

        """feet"""
        self.m_feet_l = 0.17675714575126572
        self.m_feet_r = 0.17675714575126572
        
        #self.m_array = np.array([self.motor_l,self.m_angle_l,self.m_fixed_motor_l,self.m_knee_angle_l,self.m_knee_motor_l,self.m_below_knee_angle_l,self.m_ankle_motor_l,self.m_feet_l,
                                #self.motor_r,self.m_angle_r,self.m_fixed_motor_r,self.m_knee_angle_r,self.m_knee_motor_r,self.m_below_knee_angle_r,self.m_ankle_motor_r,self.m_feet_r,
                                #self.m_base])


        self.m_array = np.array([ self.motor_r,self.m_angle_r,self.m_fixed_motor_r,self.m_knee_angle_r,self.m_knee_motor_r,self.m_below_knee_angle_r,self.m_ankle_motor_r,self.m_feet_r,
                                self.motor_l,self.m_angle_l,self.m_fixed_motor_l,self.m_knee_angle_l,self.m_knee_motor_l,self.m_below_knee_angle_l,self.m_ankle_motor_l,self.m_feet_l,
                                self.m_base])

       # print("mass array", self.m_array)

        #self.com_pos=link_com_pos
        #self.urdf_orien=link_orien
        #self.urdf_pos= link_pos

        #self.m_array=self.setLinkM()
        #self.getlinkdetails()
        #self.com_base = (self.motor_l*self.com_pos[2] + self.motor_r*self.com_pos[5] + self.m_base*self.com_base_pos)/(self.motor_l + self.motor_r + self.m_base)
        

        ##self.com_thigh_l = (self.m_array[4]*self.com_pos[4] + self.m_array[3]*self.com_pos[3])/(self.m_array[4] + self.m_array[4])
        ##self.com_thigh_r = (self.m_array[12]*self.com_pos[12] + self.m_array[11]*self.com_pos[11])/(self.m_array[12] + self.m_array[11])
        
        
        #modi_a_link_com[0] = (self.m_angle_l*self.com_pos[1]+self.m_fixed_motor_l*self.com_pos[2])/(self.m_angle_l+self.m_fixed_motor_l)
        #modi_a_link_com[1] = (self.m_knee_angle_l*self.com_pos[3]+self.m_knee_motor_l*self.com_pos[4])/(self.m_knee_angle_l+self.m_knee_motor_l)
        #modi_a_link_com[2] =  self.m_below_knee_angle_l*self.com_pos[5]/self.m_below_knee_angle_l     


        modi_a_link_mass = []
        modi_a_link_mass.append(self.m_array[1]+self.m_array[2])
        modi_a_link_mass.append(self.m_array[3]+self.m_array[4])
        modi_a_link_mass.append(self.m_array[5])
        modi_a_link_mass.append(self.m_array[6]+self.m_array[7])
        
        modi_a_link_mass = np.array(modi_a_link_mass)
        #modi_a_link_mass[0] = self.m_array[1]+self.m_array[2]
        #modi_a_link_mass[1] = self.m_array[3]+self.m_array[4]
        #modi_a_link_mass[2] = self.m_array[5]
        #modi_a_link_mass[3] = self.m_array[6]+self.m_array[7]
        
        modi_b_link_mass = []

        modi_b_link_mass.append(self.m_array[9]+self.m_array[10])
        modi_b_link_mass.append(self.m_array[11]+self.m_array[12])
        modi_b_link_mass.append(self.m_array[13])
        modi_b_link_mass.append(self.m_array[14]+self.m_array[15])

        modi_b_link_mass = np.array(modi_b_link_mass)
        
        #modi_b_link_mass = []
        #modi_b_link_mass[0] = self.m_array[9]+self.m_array[10]
        #modi_b_link_mass[1] = self.m_array[11]+self.m_array[12]
        #modi_b_link_mass[2] = self.m_array[13]
        #modi_b_link_mass[3] = self.m_array[14]+self.m_array[15]
        
        #self.com_pos=robot.robot_id
        
        com_base = (self.motor_l*link_com_pos[0] + self.motor_r*link_com_pos[8] + self.m_base*com_base_pos)/(self.motor_l + self.motor_r + self.m_base)

        modi_a_link_com = []

        modi_a_link_com.append((self.m_array[1]*link_com_pos[1]+self.m_array[2]*link_com_pos[2])/(self.m_array[1]+self.m_array[2]))
        modi_a_link_com.append((self.m_array[3]*link_com_pos[3]+self.m_array[4]*link_com_pos[4])/(self.m_array[3]+self.m_array[4]))
        modi_a_link_com.append((self.m_array[5]*link_com_pos[5]/self.m_array[5]))
        modi_a_link_com.append((self.m_array[6]*link_com_pos[6]+self.m_array[7]*link_com_pos[7])/(self.m_array[6]+self.m_array[7]))

        modi_a_link_com = np.array(modi_a_link_com)

        #modi_a_link_com[0] = (self.m_array[1]*link_com_pos[1]+self.m_array[2]*link_com_pos[2])/(self.m_array[1]+self.m_array[2])
        #modi_a_link_com[1] = (self.m_array[3]*link_com_pos[3]+self.m_array[4]*link_com_pos[4])/(self.m_array[3]+self.m_array[4])
        #modi_a_link_com[2] =  self.m_array[5]*link_com_pos[5]/self.m_array[5]
        #modi_a_link_com[3] = (self.m_array[6]*link_com_pos[6]+self.m_array[7]*link_com_pos[7])/(self.m_array[6]+self.m_array[7])

        modi_b_link_com = []
       
        modi_b_link_com.append((self.m_array[9]*link_com_pos[9]+self.m_array[10]*link_com_pos[10])/(self.m_array[9]+self.m_array[10]))
        modi_b_link_com.append((self.m_array[11]*link_com_pos[11]+self.m_array[12]*link_com_pos[12])/(self.m_array[11]+self.m_array[12]))
        modi_b_link_com.append((self.m_array[13]*link_com_pos[13]/self.m_array[13]))
        modi_b_link_com.append((self.m_array[14]*link_com_pos[14]+self.m_array[15]*link_com_pos[15])/(self.m_array[14]+self.m_array[15]))

        modi_b_link_com = np.array(modi_b_link_com)
        #modi_b_link_com[0] = (self.m_array[9]*link_com_pos[9]+self.m_array[10]*link_com_pos[10])/(self.m_array[9]+self.m_array[10])
        #modi_b_link_com[1] = (self.m_array[11]*link_com_pos[11]+self.m_array[12]*link_com_pos[12])/(self.m_array[11]+self.m_array[12])
        #modi_b_link_com[2] =  self.m_array[13]*link_com_pos[13]/self.m_array[13]
        #modi_b_link_com[3] = (self.m_array[14]*link_com_pos[14]+self.m_array[15]*link_com_pos[15])/(self.m_array[14]+self.m_array[15])
    

        modi_a_link_origin = []
        
        modi_a_link_origin.append(link_pos[1])
        modi_a_link_origin.append(link_pos[3])
        modi_a_link_origin.append(link_pos[5])
        modi_a_link_origin.append(link_pos[6])


        #modi_a_link_origin[0] = self.urdf_pos[1]
        #modi_a_link_origin[1] = self.urdf_pos[3]
        #modi_a_link_origin[2] = self.urdf_pos[5]
        #modi_a_link_origin[3] = self.urdf_pos[6]

        modi_b_link_origin = []

        
        modi_b_link_origin.append(link_pos[9])
        modi_b_link_origin.append(link_pos[11])
        modi_b_link_origin.append(link_pos[13])
        modi_b_link_origin.append(link_pos[14])
        
        #modi_b_link_origin[0] = self.urdf_pos[9]
        #modi_b_link_origin[1] = self.urdf_pos[11]
        #modi_b_link_origin[2] = self.urdf_pos[13]
        #modi_b_link_origin[3] = self.urdf_pos[14]


       
        #self.com_feet_l = (self.m_array[6]*self.com_pos[6] + self.m_array[7]*self.com_pos[7])/(self.m_array[6] + self.m_array[7])
        #self.com_feet_r = (self.m_array[14]*self.com_pos[14] + self.m_array[15]*self.com_pos[15])/(self.m_array[14] + self.m_array[15])

        #self.rho = {
            
            #'33a': self.com_feet_l - self.urdf_pos[6],
            #'22a': self.com_pos[5] - self.urdf_pos[5],
            #'11a': self.com_thigh_l - self.urdf_pos[3],

           # '33b': self.com_feet_r - self.urdf_pos[14],
            #'22b': self.com_pos[13] - self.urdf_pos[13],
            #'11b': self.com_thigh_r - self.urdf_pos[13],

            #'32a': self.com_pos[5] - self.urdf_pos[6],
           # '21a': self.com_thigh_l - self.urdf_pos[5],
            #'31a': self.com_thigh_l - self.urdf_pos[6],

            #'32b': self.com_pos[13] - self.urdf_pos[14],
            #'21b': self.com_thigh_r - self.urdf_pos[13],
            #'31b': self.com_thigh_r - self.urdf_pos[14],
        #}

        self.rho = {
            
            #'ija': modi_a_link_com[i] - modi_a_link_origin[j],
            '44a': modi_a_link_com[3] - modi_a_link_origin[3],
            '43a': modi_a_link_com[3] - modi_a_link_origin[2],
            '42a': modi_a_link_com[3] - modi_a_link_origin[1],
            '41a': modi_a_link_com[3] - modi_a_link_origin[0],

            '33a': modi_a_link_com[2] - modi_a_link_origin[2],
            '32a': modi_a_link_com[2] - modi_a_link_origin[1],
            '31a': modi_a_link_com[2] - modi_a_link_origin[0],
            
            '22a': modi_a_link_com[1] - modi_a_link_origin[1],
            '21a': modi_a_link_com[1] - modi_a_link_origin[0],


            '11a': modi_a_link_com[0] - modi_a_link_origin[0],



            '44b': modi_b_link_com[3] - modi_b_link_origin[3],
            '43b': modi_b_link_com[3] - modi_b_link_origin[2],
            '42b': modi_b_link_com[3] - modi_b_link_origin[1],
            '41b': modi_b_link_com[3] - modi_b_link_origin[0],

            '33b': modi_b_link_com[2] - modi_b_link_origin[2],
            '32b': modi_b_link_com[2] - modi_b_link_origin[1],
            '31b': modi_b_link_com[2] - modi_b_link_origin[0],
            
            '22b': modi_b_link_com[1] - modi_b_link_origin[1],
            '21b': modi_b_link_com[1] - modi_b_link_origin[0],


            '11b': modi_b_link_com[0] - modi_b_link_origin[0],
        }



        #self.setJointOrin(link_orien)
        """

        Joint orientation in global frame

        """
        #self.urdf_orien=link_orien

        global j
        j = {
            # for left leg
            

            'e1a':self.RR_modi(link_orien[1])@jaxeslocal[1], # hipR joint
            'e2a':self.RR_modi(link_orien[3])@jaxeslocal[3], # thighR joint
            'e3a':self.RR_modi(link_orien[5])@jaxeslocal[5], # kneeR joint
            'e4a':self.RR_modi(link_orien[6])@jaxeslocal[6], # ankleR joint
           # 'e1a':self.RR(link_orien[1])@np.array([0.0, 0.0, -1.0]), # hipR joint
           # 'e2a':self.RR(link_orien[3])@np.array([-1.0, 0.0, 0.0]), # thighR joint
           # 'e3a':self.RR(link_orien[5])@np.array([-1.0, 0.0, 0.0]), # kneeR joint
           # 'e4a':self.RR(link_orien[6])@np.array([-1.0, 0.0, 0.0]), # ankleR joint

            # for right leg

            'e1b':self.RR_modi(link_orien[9])@jaxeslocal[9], # hipL joint
            'e2b':self.RR_modi(link_orien[11])@jaxeslocal[11], # thighL joint
            'e3b':self.RR_modi(link_orien[13])@jaxeslocal[13], # kneeL joint
            'e4b':self.RR_modi(link_orien[14])@jaxeslocal[14], # ankleL joint
        }
       # print("matrix", self.RR_modi(link_orien[1]))
       # print ("e1a", j['e1a'])
       # print ("e1a_shape", j['e1a'].shape)
       # print("modi_a_link_mass[3]",modi_a_link_mass)
       # print("modi_a_link_mass shape",modi_a_link_mass.shape)
       # print("modi_a_link_mass[0]",modi_a_link_mass[0])
       # print("modi_a_link_mass[2]",modi_a_link_mass[2])
       # print("e4a'",self.skewM(j['e4a']))
       # print("44a'",self.rho['44a'])
       # print("44a'",self.rho['44a'])
       # print("44a:3'",self.rho['44a'][:3])

        h4a = modi_a_link_mass[3] * self.skewM(j['e4a'])@self.rho['44a'][:3]
        h3a = modi_a_link_mass[2] * self.skewM(j['e3a'])@self.rho['33a'][:3]+  modi_a_link_mass[3] * self.skewM(j['e3a'])@self.rho['43a'][:3]
        h2a = modi_a_link_mass[1] * self.skewM(j['e2a'])@self.rho['22a'][:3]+  modi_a_link_mass[2] * self.skewM(j['e2a'])@self.rho['32a'][:3] + \
        modi_a_link_mass[3] * self.skewM(j['e2a'])@self.rho['42a'][:3]
        h1a = modi_a_link_mass[0] * self.skewM(j['e1a'])@self.rho['11a'][:3]+  modi_a_link_mass[1] * self.skewM(j['e1a'])@self.rho['21a'][:3] + \
        modi_a_link_mass[2] * self.skewM(j['e1a'])@self.rho['31a'][:3] + modi_a_link_mass[3] * self.skewM(j['e1a'])@self.rho['41a'][:3]

        h4b = modi_b_link_mass[3] * self.skewM(j['e4b'])@self.rho['44b'][:3]
        h3b = modi_b_link_mass[2] * self.skewM(j['e3b'])@self.rho['33b'][:3]+  modi_b_link_mass[3] * self.skewM(j['e3b'])@self.rho['43b'][:3]
        h2b = modi_b_link_mass[1] * self.skewM(j['e2b'])@self.rho['22b'][:3]+  modi_b_link_mass[2] * self.skewM(j['e2b'])@self.rho['32b'][:3] + \
        modi_b_link_mass[3] * self.skewM(j['e2b'])@self.rho['42b'][:3]
        h1b = modi_b_link_mass[0] * self.skewM(j['e1b'])@self.rho['11b'][:3]+  modi_b_link_mass[1] * self.skewM(j['e1b'])@self.rho['21b'][:3] + \
        modi_b_link_mass[2] * self.skewM(j['e1b'])@self.rho['31b'][:3] + modi_b_link_mass[3] * self.skewM(j['e1b'])@self.rho['41b'][:3]

        
        #h3a = (self.m_array[7] + self.m_array[6])*self.skewM(j['e3a'])@self.rho['33a'][:3]
        #h2a = self.m_array[5]*self.skewM(j['e2a'])@self.rho['22a'][:3] + (self.m_array[7]+self.m_array[6])*self.skewM(j['e2a'])@self.rho['32a'][:3]
        #h1a = (self.m_array[3] + self.m_array[4])*(self.skewM(j['e1a']))@self.rho['11a'][:3] + (self.m_array[5])*self.skewM(j['e1a'])@self.rho['21a'][:3] + (self.m_array[7]+self.m_array[6])*self.skewM(j['e1a'])@self.rho['31a'][:3]

        #h3b = (self.m_array[15] + self.m_array[14])*self.skewM(j['e3b'])@self.rho['33b'][:3]
        #h2b = self.m_array[13]*self.skewM(j['e2b'])@self.rho['22b'][:3] + (self.m_array[15] + self.m_array[14])*self.skewM(j['e2b'])@self.rho['32b'][:3]
        #h1b = (self.m_array[11] + self.m_array[12])*(self.skewM(j['e1b']))@self.rho['11b'][:3] + (self.m_array[13])*self.skewM(j['e1b'])@self.rho['21b'][:3] + (self.m_array[15]+ self.m_array[14])*self.skewM(j['e1b'])@self.rho['31b'][:3]
        
        #Ha = np.stack((h1a,h2a,h3a,h4a),axis=-1)
        #Hb = np.stack((h1b,h2b,h3b,h4b),axis=-1)
        #print("h1ashape", h1a.shape)
        #print("h2ashape", h2a.shape)
        
        #print("h1a", h1a)
        Ha = np.stack((h1a,h2a,h3a,h4a),axis=-1)
        Hb = np.stack((h1b,h2b,h3b,h4b),axis=-1)
        H = np.hstack((Ha,Hb))
        
       # print("Ha", Ha)
       # print("Hashape", Ha.shape)
        
        return H

    def setI(self):
        self.I_base = self.itensor({"ixx":9.334774124638429e-05, "iyy": 0.0001300594976052366,"izz": 0.00022331485714314082, "ixy":-6.131072598073752e-08, "iyz":-9.540979117872439e-18,"ixz": 7.806255641895632e-18})
        self.I_motor_l = self.itensor({"ixx":9.777783053127598e-05, "iyy": 5.235894220499346e-05, "izz": 8.310747928435419e-05, "ixy": -2.591007643082052e-07, "iyz": -2.195755937496602e-06, "ixz": -2.9020329900915007e-07})
        self.I_motor_r = self.itensor({"ixx":9.777785485510365e-05, "iyy": 5.235895358417578e-05, "izz": 8.310749541756701e-05, "ixy": -2.591068321055115e-07, "iyz": -2.1957483612028428e-06, "ixz": -2.9020180450541055e-07})
        
        self.I_angle_l = self.itensor({"ixx":2.8853277508819435e-05, "iyy": 1.0107535652667272e-05, "izz": 2.3303786714633668e-05, "ixy": 1.6263032587282567e-19, "iyz": 1.794109458414829e-07, "ixz": 2.2442984970449942e-17})
        self.I_angle_r = self.itensor({"ixx":2.8853277508822037e-05, "iyy": 1.0107535652662501e-05, "izz": 2.330378671464668e-05, "ixy": -4.336808689942018e-18, "iyz": 1.794109458393145e-07, "ixz": 2.5370330836160804e-17})
        
        self.I_fixed_motor_l = self.itensor({"ixx":8.985014179405115e-05, "iyy": 0.00010326952603981199, "izz": 6.864312589224646e-05, "ixy": -1.7544987531356582e-06, "iyz": -6.087616118075068e-07, "ixz": -2.091503418286296e-06})
        self.I_fixed_motor_r = self.itensor({"ixx":8.985014179405809e-05, "iyy": 0.00010326952603981546, "izz": 6.864312589224472e-05, "ixy": -1.7544987531356582e-06, "iyz": -6.087616118075068e-07, "ixz": -2.0915034182871634e-06})

        self.I_knee_angle_l = self.itensor({"ixx":2.9086068087362353e-05, "iyy": 5.066664926590579e-05, "izz": 2.6291089091557383e-05, "ixy": 0.0, "iyz": 0.0, "ixz": 4.065758146820642e-18})
        self.I_knee_angle_r = self.itensor({"ixx":2.9086068087361486e-05, "iyy": 5.066664926590579e-05, "izz": 2.629108909155695e-05, "ixy": 0.0, "iyz": -8.673617379884035e-19, "ixz": 4.336808689942018e-18})

        self.I_knee_motor_l = self.itensor({"ixx":0.00010105876708862635, "iyy": 0.00011795049942860299, "izz": 6.382351138828866e-05, "ixy": 1.7380391830607023e-11, "iyz": 7.413200088512983e-11, "ixz": 1.4133891378406791e-06})
        self.I_knee_motor_r = self.itensor({"ixx":0.00010105902722260321, "iyy": 0.00011795075956257985, "izz": 6.38235113882904e-05, "ixy": 1.7380392264287892e-11, "iyz": -3.295933491409553e-11, "ixz": -1.4133468966767704e-06})

        self.I_below_knee_angle_l = self.itensor({"ixx":7.614988197868298e-05, "iyy": 9.864979757538109e-05, "izz": 3.348288885569959e-05, "ixy": 2.6020852139652106e-18, "iyz": 2.427192941159735e-06, "ixz": 5.095750210681871e-18})
        self.I_below_knee_angle_r = self.itensor({"ixx":7.614988197868298e-05, "iyy": 9.864979757538109e-05, "izz": 3.3482888855699156e-05, "ixy": 2.3852447794681098e-18, "iyz": 2.4271929411595183e-06, "ixz": 8.185726402265558e-18})

        self.I_ankle_motor_l = self.itensor({"ixx":9.532977742557694e-05, "iyy": 0.00010575949836121555, "izz": 7.439031097003154e-05, "ixy": -1.7188906561993456e-08, "iyz": 1.806096631585142e-06, "ixz": 1.437447658839434e-06})
        self.I_ankle_motor_r = self.itensor({"ixx":9.532977742557694e-05, "iyy": 0.00010575949836121555, "izz": 7.439031097003154e-05, "ixy": -1.7188906561993456e-08, "iyz": 1.806096631585142e-06, "ixz": 1.437447658839434e-06})

        """feet"""
        self.I_feet_l = self.itensor({"ixx":0.00017971831702207064, "iyy": 7.930603923658326e-05, "izz": 0.00021805293391331717, "ixy": 1.1553538897730344e-06, "iyz": -5.520206361792795e-07, "ixz": 3.686517321193287e-06})
        self.I_feet_r = self.itensor({"ixx":0.0001797183170220715, "iyy": 7.930603923658351e-05, "izz": 0.00021805293391331977, "ixy": 1.155353889772167e-06, "iyz": -5.520206361789813e-07, "ixz": 3.6865173211963022e-06})

        """Combined Intertia of Link 1"""
        #self.I_link_1_l = self.I_knee_angle_l + self.I_knee_motor_l + self.m_knee_motor_l*(self.com_pos[3] - self.com_pos[4]).T@(self.com_pos[3] - self.com_pos[4])
        #self.I_link_1_r = self.I_knee_angle_l + self.I_knee_motor_l + self.m_knee_motor_l*(self.com_pos[11] - self.com_pos[12]).T@(self.com_pos[11] - self.com_pos[12])

        #self.I_link_3_l = self.I_ankle_motor_l + self.I_feet_l + self.m_knee_angle_l*(self.com_pos[6] - self.com_pos[7]).T@(self.com_pos[6]- self.com_pos[7])
        #self.I_link_1_r = self.I_ankle_motor_l + self.I_feet_l + self.m_knee_angle_l*(self.com_pos[14] - self.com_pos[15]).T@(self.com_pos[14]- self.com_pos[15])
    

    def verifyH(self, link_vel, base_vel):
        """

        H gives us the jacobian.

        """
        #self.getlinkdetails()
        #self.getP(link_com_pos, link_orien, link_pos)

        """
        
        p_ij: j th origin to i th CG

        """
        
        """ mass of base + motor_r + motor_l"""
        self.m_base = 0.13857256283713892 
        self.motor_r = 0.328829831780374
        self.motor_l = 0.3288298236863838

        """Two angle links after base"""
        self.m_angle_l = 0.055302620656788114
        self.m_angle_r = 0.055302620656792215

        """Two motors after base"""
        self.m_fixed_motor_l = 0.3642794428368541
        self.m_fixed_motor_r = 0.3642794428368541
        
        """Two knee angles """
        self.m_knee_angle_l = 0.058088429592991084
        self.m_knee_angle_r = 0.058088429592991084

        """Two knee motors"""
        self.m_knee_motor_l = 0.3655195920887555
        self.m_knee_motor_r = 0.3655195920887556

        """Two below knee angles"""
        self.m_below_knee_angle_l = 0.07171879062702044
        self.m_below_knee_angle_r = 0.07171879062702044

        """Ankle Motors"""
        self.m_ankle_motor_l = 0.3723917469007978
        self.m_ankle_motor_r = 0.3723917469007978

        """feet"""
        self.m_feet_l = 0.17675714575126572
        self.m_feet_r = 0.17675714575126572
        
        #self.m_array = np.array([self.motor_l,self.m_angle_l,self.m_fixed_motor_l,self.m_knee_angle_l,self.m_knee_motor_l,self.m_below_knee_angle_l,self.m_ankle_motor_l,self.m_feet_l,
                                #self.motor_r,self.m_angle_r,self.m_fixed_motor_r,self.m_knee_angle_r,self.m_knee_motor_r,self.m_below_knee_angle_r,self.m_ankle_motor_r,self.m_feet_r,
                                #self.m_base])


        self.m_array = np.array([ self.motor_r,self.m_angle_r,self.m_fixed_motor_r,self.m_knee_angle_r,self.m_knee_motor_r,self.m_below_knee_angle_r,self.m_ankle_motor_r,self.m_feet_r,
                                self.motor_l,self.m_angle_l,self.m_fixed_motor_l,self.m_knee_angle_l,self.m_knee_motor_l,self.m_below_knee_angle_l,self.m_ankle_motor_l,self.m_feet_l,
                                self.m_base])

       # print("mass array", self.m_array)

        #self.com_pos=link_com_pos
        #self.urdf_orien=link_orien
        #self.urdf_pos= link_pos

        #self.m_array=self.setLinkM()
        #self.getlinkdetails()
        #self.com_base = (self.motor_l*self.com_pos[2] + self.motor_r*self.com_pos[5] + self.m_base*self.com_base_pos)/(self.motor_l + self.motor_r + self.m_base)
        

        ##self.com_thigh_l = (self.m_array[4]*self.com_pos[4] + self.m_array[3]*self.com_pos[3])/(self.m_array[4] + self.m_array[4])
        ##self.com_thigh_r = (self.m_array[12]*self.com_pos[12] + self.m_array[11]*self.com_pos[11])/(self.m_array[12] + self.m_array[11])
        
        
        #modi_a_link_com[0] = (self.m_angle_l*self.com_pos[1]+self.m_fixed_motor_l*self.com_pos[2])/(self.m_angle_l+self.m_fixed_motor_l)
        #modi_a_link_com[1] = (self.m_knee_angle_l*self.com_pos[3]+self.m_knee_motor_l*self.com_pos[4])/(self.m_knee_angle_l+self.m_knee_motor_l)
        #modi_a_link_com[2] =  self.m_below_knee_angle_l*self.com_pos[5]/self.m_below_knee_angle_l     


        modi_a_link_mass = []
        modi_a_link_mass.append(self.m_array[1]+self.m_array[2])
        modi_a_link_mass.append(self.m_array[3]+self.m_array[4])
        modi_a_link_mass.append(self.m_array[5])
        modi_a_link_mass.append(self.m_array[6]+self.m_array[7])
        
        modi_a_link_mass = np.array(modi_a_link_mass)
        #modi_a_link_mass[0] = self.m_array[1]+self.m_array[2]
        #modi_a_link_mass[1] = self.m_array[3]+self.m_array[4]
        #modi_a_link_mass[2] = self.m_array[5]
        #modi_a_link_mass[3] = self.m_array[6]+self.m_array[7]
        
        modi_b_link_mass = []

        modi_b_link_mass.append(self.m_array[9]+self.m_array[10])
        modi_b_link_mass.append(self.m_array[11]+self.m_array[12])
        modi_b_link_mass.append(self.m_array[13])
        modi_b_link_mass.append(self.m_array[14]+self.m_array[15])

        modi_b_link_mass = np.array(modi_b_link_mass)
        
        #modi_b_link_mass = []
        #modi_b_link_mass[0] = self.m_array[9]+self.m_array[10]
        #modi_b_link_mass[1] = self.m_array[11]+self.m_array[12]
        #modi_b_link_mass[2] = self.m_array[13]
        #modi_b_link_mass[3] = self.m_array[14]+self.m_array[15]
        
        #self.com_pos=robot.robot_id
        
        vel_base = (self.m_array[0]*link_vel[0] + self.m_array[8]*link_vel[8] + self.m_array[16]*base_vel)/(self.m_array[0]+self.m_array[8]+self.m_array[16])
        
        mass_base= self.m_array[0]+self.m_array[8]+self.m_array[16]

        modi_a_link_vel = []

        modi_a_link_vel.append((self.m_array[1]*link_vel[1]+self.m_array[2]*link_vel[2])/(self.m_array[1]+self.m_array[2]))
        modi_a_link_vel.append((self.m_array[3]*link_vel[3]+self.m_array[4]*link_vel[4])/(self.m_array[3]+self.m_array[4]))
        modi_a_link_vel.append((self.m_array[5]*link_vel[5]/self.m_array[5]))
        modi_a_link_vel.append((self.m_array[6]*link_vel[6]+self.m_array[7]*link_vel[7])/(self.m_array[6]+self.m_array[7]))

        modi_a_link_vel = np.array(modi_a_link_vel)
        
        total_mass_a=np.sum(modi_a_link_mass)

        modi_a_link_com_vel=(modi_a_link_mass[0]*modi_a_link_vel[0]+modi_a_link_mass[1]*modi_a_link_vel[1]+\
                            modi_a_link_mass[2]*modi_a_link_vel[2]+modi_a_link_mass[3]*modi_a_link_vel[3])/total_mass_a

        #modi_a_link_com[0] = (self.m_array[1]*link_com_pos[1]+self.m_array[2]*link_com_pos[2])/(self.m_array[1]+self.m_array[2])
        #modi_a_link_com[1] = (self.m_array[3]*link_com_pos[3]+self.m_array[4]*link_com_pos[4])/(self.m_array[3]+self.m_array[4])
        #modi_a_link_com[2] =  self.m_array[5]*link_com_pos[5]/self.m_array[5]
        #modi_a_link_com[3] = (self.m_array[6]*link_com_pos[6]+self.m_array[7]*link_com_pos[7])/(self.m_array[6]+self.m_array[7])

        modi_b_link_vel = []
       
        modi_b_link_vel.append((self.m_array[9]*link_vel[9]+self.m_array[10]*link_vel[10])/(self.m_array[9]+self.m_array[10]))
        modi_b_link_vel.append((self.m_array[11]*link_vel[11]+self.m_array[12]*link_vel[12])/(self.m_array[11]+self.m_array[12]))
        modi_b_link_vel.append((self.m_array[13]*link_vel[13]/self.m_array[13]))
        modi_b_link_vel.append((self.m_array[14]*link_vel[14]+self.m_array[15]*link_vel[15])/(self.m_array[14]+self.m_array[15]))

        modi_b_link_vel = np.array(modi_b_link_vel)
        total_mass_b=np.sum(modi_b_link_mass)

        modi_b_link_com_vel=(modi_b_link_mass[0]*modi_b_link_vel[0]+modi_b_link_mass[1]*modi_b_link_vel[1]+\
                            modi_b_link_mass[2]*modi_b_link_vel[2]+modi_b_link_mass[3]*modi_b_link_vel[3])/total_mass_b
        
       
        com_vel=(total_mass_a*modi_a_link_com_vel+total_mass_b*modi_b_link_com_vel+mass_base*vel_base)/(total_mass_a+total_mass_b+mass_base)    

        return com_vel                 
        #modi_b_link_com[0] = (self.m_array[9]*link_com_pos[9]+self.m_array[10]*link_com_pos[10])/(self.m_array[9]+self.m_array[10])
        #modi_b_link_com[1] = (self.m_array[11]*link_com_pos[11]+self.m_array[12]*link_com_pos[12])/(self.m_array[11]+self.m_array[12])
        #modi_b_link_com[2] =  self.m_array[13]*link_com_pos[13]/self.m_array[13]
        #modi_b_link_com[3] = (self.m_array[14]*link_com_pos[14]+self.m_array[15]*link_com_pos[15])/(self.m_array[14]+self.m_array[15])
    

        #modi_a_link_origin = []
        
        #modi_a_link_origin.append(link_pos[1])
        #modi_a_link_origin.append(link_pos[3])
        #modi_a_link_origin.append(link_pos[5])
        #modi_a_link_origin.append(link_pos[6])


        #modi_a_link_origin[0] = self.urdf_pos[1]
        #modi_a_link_origin[1] = self.urdf_pos[3]
        #modi_a_link_origin[2] = self.urdf_pos[5]
        #modi_a_link_origin[3] = self.urdf_pos[6]

        #modi_b_link_origin = []

        
        #modi_b_link_origin.append(link_pos[9])
        #modi_b_link_origin.append(link_pos[11])
        #modi_b_link_origin.append(link_pos[13])
        #modi_b_link_origin.append(link_pos[14])
        
        #modi_b_link_origin[0] = self.urdf_pos[9]
        #modi_b_link_origin[1] = self.urdf_pos[11]
        #modi_b_link_origin[2] = self.urdf_pos[13]
        #modi_b_link_origin[3] = self.urdf_pos[14]


       
        #self.com_feet_l = (self.m_array[6]*self.com_pos[6] + self.m_array[7]*self.com_pos[7])/(self.m_array[6] + self.m_array[7])
        #self.com_feet_r = (self.m_array[14]*self.com_pos[14] + self.m_array[15]*self.com_pos[15])/(self.m_array[14] + self.m_array[15])

        #self.rho = {
            
            #'33a': self.com_feet_l - self.urdf_pos[6],
            #'22a': self.com_pos[5] - self.urdf_pos[5],
            #'11a': self.com_thigh_l - self.urdf_pos[3],

           # '33b': self.com_feet_r - self.urdf_pos[14],
            #'22b': self.com_pos[13] - self.urdf_pos[13],
            #'11b': self.com_thigh_r - self.urdf_pos[13],

            #'32a': self.com_pos[5] - self.urdf_pos[6],
           # '21a': self.com_thigh_l - self.urdf_pos[5],
            #'31a': self.com_thigh_l - self.urdf_pos[6],

            #'32b': self.com_pos[13] - self.urdf_pos[14],
            #'21b': self.com_thigh_r - self.urdf_pos[13],
            #'31b': self.com_thigh_r - self.urdf_pos[14],
        #}

        



        #self.setJointOrin(link_orien)
        """

        Joint orientation in global frame

        """
        #self.urdf_orien=link_orien

        
        
        #h3a = (self.m_array[7] + self.m_array[6])*self.skewM(j['e3a'])@self.rho['33a'][:3]
        #h2a = self.m_array[5]*self.skewM(j['e2a'])@self.rho['22a'][:3] + (self.m_array[7]+self.m_array[6])*self.skewM(j['e2a'])@self.rho['32a'][:3]
        #h1a = (self.m_array[3] + self.m_array[4])*(self.skewM(j['e1a']))@self.rho['11a'][:3] + (self.m_array[5])*self.skewM(j['e1a'])@self.rho['21a'][:3] + (self.m_array[7]+self.m_array[6])*self.skewM(j['e1a'])@self.rho['31a'][:3]

        #h3b = (self.m_array[15] + self.m_array[14])*self.skewM(j['e3b'])@self.rho['33b'][:3]
        #h2b = self.m_array[13]*self.skewM(j['e2b'])@self.rho['22b'][:3] + (self.m_array[15] + self.m_array[14])*self.skewM(j['e2b'])@self.rho['32b'][:3]
        #h1b = (self.m_array[11] + self.m_array[12])*(self.skewM(j['e1b']))@self.rho['11b'][:3] + (self.m_array[13])*self.skewM(j['e1b'])@self.rho['21b'][:3] + (self.m_array[15]+ self.m_array[14])*self.skewM(j['e1b'])@self.rho['31b'][:3]
        
        #Ha = np.stack((h1a,h2a,h3a,h4a),axis=-1)
        #Hb = np.stack((h1b,h2b,h3b,h4b),axis=-1)
       
        
        
    def getL(self,link_mass, link_vel,link_omega, link_com_pos, com_base_pos, base_ori, base_vel, base_angvel):

        #self.m_array=self.setLinkM()
        #self.getP()
        #self.setI()
        
        #v_com_thigh_l = (self.m_array[4]*link_vel[4] + self.m_array[3]*link_vel[3])/(self.m_array[4] + self.m_array[3])
        #v_com_thigh_r = (self.m_array[12]*link_vel[12] + self.m_array[11]*link_vel[11])/(self.m_array[12] + self.m_array[11])

        #v_com_feet_l = (self.m_array[6]*link_vel[6] + self.m_array[7]*link_vel[7])/(self.m_array[6] + self.m_array[7])
        #v_com_feet_r = (self.m_array[14]*link_vel[14] + self.m_array[15]*link_vel[15])/(self.m_array[14] + self.m_array[15])

        com_pos = self.getCOMPos(link_mass=link_mass, link_com_pos=link_com_pos, com_base_pos=com_base_pos)
        
        #"""Left Leg Angular Momentum"""
        #L1 = self.m_link['l1a']*self.skewM(self.com_thigh_l - com_pos)@v_com_thigh_l + self.R(self.urdf_orien[0])@self.I_link_1_l@link_omega[3] + \
             #   self.m_link['l2a']*self.skewM(link_com_pos[5] - com_pos)@link_vel[5] + self.R(self.urdf_orien[1])@self.I_below_knee_angle_l@link_omega[5] + \
              #  self.m_link['l3a']*self.skewM(self.com_feet_l - com_pos)@v_com_feet_l + self.R(self.urdf_orien[2])@self.I_link_3_l@link_omega[6] 
        #print ("mass", self.m_array[0])
       # print ("xx", self.skewM(com_pos-link_com_pos[0])@link_vel[0])

        Llin1= link_mass[0]*self.skewM(com_pos-link_com_pos[0])@link_vel[0]+ link_mass[1]*self.skewM(com_pos-link_com_pos[1])@link_vel[1] + \
            link_mass[2]*self.skewM(com_pos-link_com_pos[2])@link_vel[2]+link_mass[3]*self.skewM(com_pos-link_com_pos[3])@link_vel[3] + \
            link_mass[4]*self.skewM(com_pos-link_com_pos[4])@link_vel[4]+link_mass[5]*self.skewM(com_pos-link_com_pos[5])@link_vel[5] + \
            link_mass[6]*self.skewM(com_pos-link_com_pos[6])@link_vel[6]+link_mass[7]*self.skewM(com_pos-link_com_pos[7])@link_vel[7]
        
       # print("Llin1 size", Llin1.shape)

        Llin2=link_mass[8]*self.skewM(com_pos-link_com_pos[8])@link_vel[8]+link_mass[9]*self.skewM(com_pos-link_com_pos[9])@link_vel[9]+\
            link_mass[10]*self.skewM(com_pos-link_com_pos[10])@link_vel[10]+link_mass[11]*self.skewM(com_pos-link_com_pos[11])@link_vel[11]+  link_mass[12]*self.skewM(com_pos-link_com_pos[12])@link_vel[12]+link_mass[13]*self.skewM(com_pos-link_com_pos[13])@link_vel[13] +  link_mass[14]*self.skewM(com_pos-link_com_pos[14])@link_vel[14]+link_mass[15]*self.skewM(com_pos-link_com_pos[15])@link_vel[15] +\
            link_mass[16]*self.skewM(com_pos-com_base_pos)@base_vel
        
        #Lang= self.RR(self.urdf_orien[0])@self.I_motor_l@self.RR(self.urdf_orien[0]).T@link_omega[0]+\
             # self.RR(self.urdf_orien[1])@self.I_angle_l@self.RR(self.urdf_orien[1]).T@link_omega[1]+\
             # self.RR(self.urdf_orien[2])@self.I_fixed_motor_l@self.RR(self.urdf_orien[2]).T@link_omega[2]+\
             # self.RR(self.urdf_orien[3])@self.I_knee_angle_l@self.RR(self.urdf_orien[3]).T@link_omega[3]+\
              #self.RR(self.urdf_orien[4])@self.I_knee_motor_l@self.RR(self.urdf_orien[4]).T@link_omega[4]+\
             # self.RR(self.urdf_orien[5])@self.I_below_knee_angle_l@self.RR(self.urdf_orien[5]).T@link_omega[5]+\
             # self.RR(self.urdf_orien[6])@self.I_ankle_motor_l@self.RR(self.urdf_orien[6]).T@link_omega[6]+\
             # self.RR(self.urdf_orien[7])@self.I_feet_l@self.RR(self.urdf_orien[7]).T@link_omega[7]+\
             # self.RR(self.urdf_orien[8])@self.I_motor_r@self.RR(self.urdf_orien[8]).T@link_omega[8]+\
             # self.RR(self.urdf_orien[9])@self.I_angle_r@self.RR(self.urdf_orien[9]).T@link_omega[9]+\
             # self.RR(self.urdf_orien[10])@self.I_fixed_motor_r@self.RR(self.urdf_orien[10]).T@link_omega[10]+\
             # self.RR(self.urdf_orien[11])@self.I_knee_angle_r@self.RR(self.urdf_orien[11]).T@link_omega[11]+\
             # self.RR(self.urdf_orien[12])@self.I_knee_motor_r@self.RR(self.urdf_orien[12]).T@link_omega[12]+\
              #self.RR(self.urdf_orien[13])@self.I_below_knee_angle_r@self.RR(self.urdf_orien[13]).T@link_omega[13]+\
              #self.RR(self.urdf_orien[14])@self.I_ankle_motor_r@self.RR(self.urdf_orien[14]).T@link_omega[14]+\
              #self.RR(self.urdf_orien[15])@self.I_feet_r@self.RR(self.urdf_orien[15]).T@link_omega[15]+\
              #self.RR(base_ori)@self.I_base@self.RR(base_ori).T@base_angvel   

       # Lang= self.I_motor_l*link_omega[0]+\
            #  self.I_angle_l*link_omega[1]+\
            #  self.I_fixed_motor_l*link_omega[2]+\
            #  self.I_knee_angle_l*link_omega[3]+\
            #  self.I_knee_motor_l*link_omega[4]+\
            #  self.I_below_knee_angle_l*link_omega[5]+\
            #  self.I_ankle_motor_l*link_omega[6]+\
             # self.I_feet_l*link_omega[7]+\
            #  self.I_motor_r*link_omega[8]+\
            #  self.I_angle_r*link_omega[9]+\
            #  self.I_fixed_motor_r*link_omega[10]+\
            #  self.I_knee_angle_r*link_omega[11]+\
            #  self.I_knee_motor_r*link_omega[12]+\
            #  self.I_below_knee_angle_r*link_omega[13]+\
            #  self.I_ankle_motor_r*link_omega[14]+\
            #  self.I_feet_r*link_omega[15]+\
            #  self.I_base*base_angvel 
               
               
           
        
        
        """Right Leg Angular Momentum"""
       # L2 = self.m_link['l1b']*self.skewM(self.com_thigh_r - com_pos)@v_com_thigh_r + self.R(self.urdf_orien[3])@self.I_link_1_r@link_omega[11]  + \
              #  self.m_link['l2b']*self.skewM(self.com_pos[13] - com_pos)@link_vel[13] + self.R(self.urdf_orien[4])@self.I_below_knee_angle_r@link_omega[13] + \
            #    self.m_link['l3b']*self.skewM(self.com_feet_l - com_pos)@v_com_feet_r + self.R(self.urdf_orien[5])@self.I_link_3_l@link_omega[14] 

        L = Llin1+Llin2

        return L

   

    #   return ()
    def getCOMPos(self, link_mass, link_com_pos, com_base_pos):
       # return (self.m_link['l1a']*self.com_thigh_l + self.m_link['l2a']*self.com_pos[5] + self.m_link['l3a']*self.com_feet_l + \
          #  self.m_link['l1b']*self.com_thigh_r + self.m_link['l2b']*self.com_pos[13] + self.m_link['l3b']*self.com_feet_l + \
               # self.m_link['base']*self.com_base_pos) / ( self.m_link['l1a'] + self.m_link['l2a'] + self.m_link['l3a'] + self.m_link['l1b'] + self.m_link['l2b'] + self.m_link['l3b'] + self.m_link['base'])
        #self.m_array = link_mass
        self.COM_pos=(link_mass[0]*link_com_pos[0]+ link_mass[1]*link_com_pos[1]+ link_mass[2]*link_com_pos[2]+ link_mass[3]*link_com_pos[3]+\
                link_mass[4]*link_com_pos[4]+ link_mass[5]*link_com_pos[5]+ link_mass[6]*link_com_pos[6]+ link_mass[7]*link_com_pos[7]+\
                link_mass[8]*link_com_pos[8]+ link_mass[9]*link_com_pos[9]+ link_mass[10]*link_com_pos[10]+ link_mass[11]*link_com_pos[11]+\
                link_mass[12]*link_com_pos[12]+ link_mass[13]*link_com_pos[13]+ link_mass[14]*link_com_pos[14]+ link_mass[15]*link_com_pos[15] + link_mass[16]*com_base_pos)/\
                np.sum(link_mass)

        return(self.COM_pos)
        

    def setLinkM(self):
        """ mass of base + motor_r + motor_l"""
        self.m_base = 0.13857256283713892 
        self.motor_r = 0.328829831780374
        self.motor_l = 0.3288298236863838

        """Two angle links after base"""
        self.m_angle_l = 0.055302620656788114
        self.m_angle_r = 0.055302620656792215

        """Two motors after base"""
        self.m_fixed_motor_l = 0.3642794428368541
        self.m_fixed_motor_r = 0.3642794428368541
        
        """Two knee angles """
        self.m_knee_angle_l = 0.058088429592991084
        self.m_knee_angle_r = 0.058088429592991084

        """Two knee motors"""
        self.m_knee_motor_l = 0.3655195920887555
        self.m_knee_motor_r = 0.3655195920887556

        """Two below knee angles"""
        self.m_below_knee_angle_l = 0.07171879062702044
        self.m_below_knee_angle_r = 0.07171879062702044

        """Ankle Motors"""
        self.m_ankle_motor_l = 0.3723917469007978
        self.m_ankle_motor_r = 0.3723917469007978

        """feet"""
        self.m_feet_l = 0.17675714575126572
        self.m_feet_r = 0.17675714575126572
        
        self.m_array = np.array([self.motor_l,self.m_angle_l,self.m_fixed_motor_l,self.m_knee_angle_l,self.m_knee_motor_l,self.m_below_knee_angle_l,self.m_ankle_motor_l,self.m_feet_l,
                                self.motor_r,self.m_angle_r,self.m_fixed_motor_r,self.m_knee_angle_r,self.m_knee_motor_r,self.m_below_knee_angle_r,self.m_ankle_motor_r,self.m_feet_r,
                                self.m_base])
        
        #self.m_link = {
           # 'base': self.m_base + self.motor_l + self.motor_r,
          #  'l1a': self.m_knee_angle_l + self.m_knee_motor_l,
           # 'l2a': self.m_below_knee_angle_l,
           # 'l3a': self.m_ankle_motor_l + self.m_feet_l,
           # 'l1b': self.m_knee_angle_r + self.m_knee_motor_r,
           # 'l2b': self.m_below_knee_angle_r,
           # 'l3b': self.m_ankle_motor_r + self.m_feet_r
       # }
        #mass_array = self.m_array
        #M = np.sum(self.m_array)

        return self.m_array

if __name__ == "__main__":
    robot = Biped()
   # traj = GenTraj()
   # s1 = BipedSim()
   # p1 = np.array([-0.17,0.17,-0.05])
   ## p2 = np.array([ 0.07,-0.07,0.1])
   # p3 = np.array([-0.17,0.17,-0.05])
    #traj.setTrajPt(**{"p1":p1,"p2": p2,"p3": p3})
   # s1.runSim()
