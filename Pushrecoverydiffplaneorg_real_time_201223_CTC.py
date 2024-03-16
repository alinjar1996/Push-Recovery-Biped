import pybullet as p
import time
import numpy as np
from biped_new_200923 import Biped
#from traj import GenTraj
import csv
import comjacob as cj
import optimize_xy as opt
from scipy.spatial import ConvexHull, convex_hull_plot_2d


class BipedSim:
    def __init__(self):
        self.t_stop = 100
        
        #self.t_step = 1/240
        self.t_step1 = 1/240
        self.t_step2 = 25/240 #earlier 17/240
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
        self.ankle_torque4 = 0
        self.ankle_torque10 = 0
        self.ankle_torque = np.array([0,0])
 
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

        f1a = p.getContactPoints(robot.robot_id,robot.planeID,47,-1)

        f2a = p.getContactPoints(robot.robot_id,self.plane_2,24,-1)

        f1b = p.getContactPoints(robot.robot_id,robot.planeID,48,-1)

        f2b = p.getContactPoints(robot.robot_id,self.plane_2,25,-1)
        

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

        CP3D = [] # Contact_points on 3D
        
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
            CP3D.append(f1a[i][6][0:3]) #contact point on 3D

            #Force1a = np.array()
            C1a.append(f1a[i][5]) #contact point on robot

            N1C1x.append(f1a[i][9]*f1a[i][6][0])
            N1C1y.append(f1a[i][9]*f1a[i][6][1])

         #   N1C1x += f1a[i][9]*f1a[i][6][0]
          #  N1C1y += f1a[i][9]*f1a[i][6][1]

           # N1c1 = np.dot()

            #at1a.append(np.dot(np.array([0,0,Norm1a]),np.array([C1a])))


        ankle_joint_pos = robot.urdf_pos[47]

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
            CP3D.append(f1b[i][6][0:3])

            C1b.append(f1b[i][5])

            N1C1x.append(f1b[i][9]*f1b[i][6][0])
            N1C1y.append(f1b[i][9]*f1b[i][6][1])


        ankle_joint_pos = robot.urdf_pos[48]
        
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
            CP3D.append(f2a[i][6][0:3])

            C2a.append(f2a[i][5])

        ankle_joint_pos = robot.urdf_pos[24]    
        
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
            CP3D.append(f2b[i][6][0:3])

            C2b.append(f2b[i][5])
        

        ankle_joint_pos = robot.urdf_pos[25] 

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


        
        xzmp=(np.sum(N1C1x)+np.sum(N2C2x))/(N1+N2)
        yzmp=(np.sum(N1C1y)+np.sum(N2C2y))/(N1+N2)

        zmp=[xzmp, yzmp]

       # print("CP3D", CP3D)

        return N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp, CP3D

    def chooseeig(self,v,thd,t):

        xnorm=np.zeros((9,1))
       
        for i in range(3,len(v)):
          x= v[:,i]-thd
          xnorm[i-3]=np.linalg.norm(x)
        
        xnormmin = min(xnorm)

        for k in range(len(xnorm)):

          if  xnorm[k] ==  xnormmin:

            if t <1:  
            
               choseneig =  5.5*v[:,k]
            elif t >1 and t<2:

                choseneig =  2*v[:,k]
            else:

               choseneig =  1*v[:,k]

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

    def plotpolyhedralconvexcone(self, N, mu, macm, t):

       from mpl_toolkits.mplot3d import Axes3D
       from mpl_toolkits.mplot3d.art3d import Poly3DCollection
       import matplotlib.pyplot as plt

       fig = plt.figure()
        #ax = Axes3D(fig, auto_add_to_figure =  False)
       ax = Axes3D(fig)
       fig.add_axes(ax)

       #N = 30
        #mu =0.5

       c = -mu*N/np.sqrt(2)

       x= [0, -c, c, c, -c]
       y= [0, -c, -c, c, c]
       z= [0, N, N, N, N]

       xa = [0,-c]; ya =[0,-c]; za = [0,N]
       xb = [0,c]; yb =[0,-c]; zb = [0,N]
       xc = [0,c]; yc =[0, c]; zc = [0,N]
       xd = [0,-c]; yd =[0, c]; zd = [0,N]

       xab = [-c,c];yab = [-c,-c];zab = [N,N]
       xad = [-c,-c];yad = [-c,c];zad = [N,N]
       xbc = [c,c];ybc = [-c,c];zbc = [N,N]
       xcd = [c,-c];ycd = [c,c];zcd = [N,N]

        #z= [ 0, 0, 0, 0]

       x1 = [0,macm[0]]
       y1 = [0,macm[1]]
       z1 = [0,macm[2]]

        #verts = [list(zip(x,y,z))]


        #ax.add_collection3d(Poly3DCollection(verts))
       ax.plot(xa,ya,za, color = 'red')
       ax.plot(xb,yb,zb, color = 'red')
       ax.plot(xc,yc,zc, color = 'red')
       ax.plot(xd,yd,zd, color = 'red')

       ax.plot(xab,yab,zab, color = 'red')
       ax.plot(xad,yad,zad, color = 'red')
       ax.plot(xbc,ybc,zbc, color = 'red')
       ax.plot(xcd,ycd,zcd, color = 'red')

       ax.plot(x1,y1,z1, color = 'green')

       plt.title("t=" + str(t))

       plt.show()
    
    def plotvirtualhorizontalplanepoints(self,CP3D,CM,PZ):


       #PZ is the Z coordinate for Virtual horizontal plane

       PX = []; PY= []; VHP = []

       CMX = CM[0]; CMY = CM[1]; CMZ = CM[2]



       for i in range(len(CP3D)):
            
            ax  = CP3D[i][0]
            ay  = CP3D[i][1]
            az  = CP3D[i][2]


            
            #ay  = CP3D[i,1]
            #az  = CP3D[i,2]

            Pxi = (((PZ-CMZ)/(az-CMZ))*(ax-CMX))+CMX

            #PX.append(Pxi)
            
            Pyi = (((PZ-CMZ)/(az-CMZ))*(ay-CMY))+CMY

            #PY.append(Pyi)

            #VHP = [0,0]

            VHP.append((Pxi,Pyi))


       return VHP  
       

    def getJointState(self):
        joints = p.getJointStates(robot.robot_id,robot.rev)
        self.q = [ i[0] for i in joints]
        self.qdot = [ i[1] for i in joints]
        self.applied_torques = [i[3] for i in joints]
        

        
        self.jdetails=[]
        #for i in joints:
        for ii in range(49):
          self.jdetails.append(p.getJointInfo(robot.robot_id,ii)[13])
      

        alljoints = p.getJointStates(robot.robot_id, robot.allj)
        qall = [ i[0] for i in alljoints]
        qdotall = [ i[1] for i in alljoints]


        jointinfo = p.getJointInfo(robot.robot_id,9)


        return self.q, qdotall, self.qdot


    def runSim(self):
        #robot.setInitialState()
        #robot.turnOffActuators()
        #robot.turnOffDamping()
        #p.enableJointForceTorqueSensor(robot.robot_id,7,enableSensor=1)

        # while True:
        nj = 12; #number of revolute joints

        self.dthdes = np.zeros((nj))
        self.thdes = np.zeros((nj))
        self.ddthdes = np.zeros((nj))
        self.jtorques = np.zeros((nj))
        self.jtorques_filt = np.zeros((nj))
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
        


        
        p.changeDynamics(robot.robot_id ,  -1, mass=0)

        #There are total three time loops inh this code. First time loop (for loop) is for bringing biped to a double leg stance at offset height. 
        # Next two loops are the same as double leg stance at the same plane   

        for t in  np.linspace(0,self.t_set,int((self.t_set-0)/self.t_step1)):

            self.timee = np.array([t,0])
            self.timee = np.reshape(self.timee,(2,))

            p.stepSimulation()

            
            self.checkH()

            #global thdes


            #thdes = np.zeros((8,1))
            #v = np.zeros((8,1))

            
            self.thdes[2] = 1.1
            self.thdes[3] = -1.1
            #thdes[6] = 0.1

            p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes ,targetVelocities= self.dthdes)

           # p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[2], controlMode = p.POSITION_CONTROL,targetPosition=-0.1)
           # p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[1], controlMode = p.POSITION_CONTROL,targetPosition=0.1)

            #k_a = self.invkin()

            #self.invkin()
            
          


            self.write_data()

           
          #  time.sleep(1/240)
        

        self.urdf_path1 = "/home/udayan/Desktop/Alinjar/Biped Pybullet/bipedal_03082023_12DOF_and_6_DOF/bipedal_12_01/box_small.urdf"

        self.plane_2  = p.loadURDF(self.urdf_path1,useFixedBase= False, basePosition=[0.02+1,-0.05+1-0.05,0.025])
        
       

        p.changeDynamics(self.plane_2 ,  -1, lateralFriction=0.7)
        #p.changeDynamics(self.plane_2 ,  -1, lateralFriction=0.6)
        p.changeDynamics(robot.planeID ,  -1, lateralFriction=0.5)
        #p.changeDynamics(robot.planeID ,  -1, lateralFriction=0.7)
        
        for t in np.linspace(self.t_set,self.t_set+self.t_impact,int((self.t_impact)/self.t_step1)):
        
         #self.timeee(self,t)
            self.timee = np.array([t,0])
            self.timee = np.reshape(self.timee,(2,))
       
           # p.setRealTimeSimulation(0)
            p.stepSimulation()
          #  p.addUserDebugText(str(t),textPosition = [0,0,0.5], textColorRGB = [1,0,0],textSize = 1, lifeTime = 0)
          
            #robot.getJointState()
            self.getJointState()
            
            robot.drawPbText(str(t))
            #robot.drawPb1(self.c_cm)
            #print("self.c_cm", self.c_cm)
           

            self.thdvel = np.array([self.thdvel])

  

            global thdvel_prev

            thdvel_prev = self.thdvel

            

            self.checkH() #gets the joint velocity self.thdvel
    
            robot.drawPbPoint(self.c_cm)


 
            self.thddacc = (self.thdvel -  thdvel_prev)*(1/self.t_step1)
            
          




            if t > self.t_set+self.t_rest and t < self.t_impact+self.t_set+self.t_rest:                             #This combination works the best comparison to v=[000000]
               # p.applyExternalForce(robot.robot_id,-1,[5,-4,0],[0,0,0],flags=p.LINK_FRAME)
              # p.applyExternalForce(robot.robot_id,-1,[-0,-9.5,-0],[0,0,0],flags=p.LINK_FRAME) #keep this value
               #p.applyExternalForce(robot.robot_id,-1,[8,-8,-0],[0,0,0],flags=p.LINK_FRAME)
              # self.F_app = np.array([-15*0.8/3,-48*0.8/3,0]) # earlier as in October 2023
               self.F_app = np.array([-15*0.8/3,-48*0.8/3,0])
               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            else:
               self.F_app = np.array([0,0,-0])    

               

            p.changeDynamics(robot.robot_id ,  -1, mass=0.05)  #changing mass of base of biped
            
            #v = 0.5*np.linalg.pinv(self.h)@(self.M*self.v_torso) + 0*eigvec[4]
            
           
            
            #v=list(v)

            thd=p.getJointStates(robot.robot_id,robot.rev)



            if t<0.01+self.t_set+self.t_rest:


             
              thinitial = self.q
              
            


            self.M = np.sum(robot.setLinkM())
 
            self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])

            #self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[8]*robot.link_vel[8]+robot.m_array[16]*self.v_torso)/(robot.m_array[0]+robot.m_array[8]+robot.m_array[16])
            
            self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[1]*robot.link_vel[1]+\
                         +robot.m_array[2]*robot.link_vel[2]+robot.m_array[3]*robot.link_vel[3]+\
                         +robot.m_array[26]*robot.link_vel[26]+robot.m_array[49]*self.v_torso/(robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]))        

        
        #dth = np.delete(robot.qdot,[0,4])

            qdot = self.getJointState()[2]
        
            dth=np.array(qdot)
       

        #vbase_r= robot.base_vel+self.skewM(robot.base_angvel)@(robot.urdf_pos[0]--robot.com_base_pos)
            vbase_r= robot.link_vel[3]+self.skewM(robot.link_omega[3])@(robot.urdf_pos[4]-robot.com_pos[3])

            vbase_l= robot.link_vel[26]+self.skewM(robot.link_omega[26])@(robot.urdf_pos[27]-robot.com_pos[26])

         #   x1=vbase_l+self.skewM(robot.link_omega[9])@(-robot.urdf_pos[9]+robot.com_pos[9])


            Mr = np.sum(robot.setLinkM()[4:26])
            Ml = np.sum(robot.setLinkM()[27:49])

            #M_base= robot.setLinkM()[16]+robot.setLinkM()[0]+robot.setLinkM()[8]

            M_base = robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]
        
            modi_link_mass = robot.setLinkM() #same just base mass at the end

            modi_link_vel = np.append(robot.link_vel,[self.v_torso],axis=0)

            vcmdata=robot.getrobotCOMvelocity(modi_link_mass, modi_link_vel)

            vrightcm = vcmdata[0]; vleftcm = vcmdata[1]; vbasecm = vcmdata[2]; vtotalcm =  vcmdata[3]
                   
            link_vell = np.append(robot.link_vel,[self.v_torso],axis=0)

            global m_vcm_prev

            m_vcm_prev = m_vcm
     
        
            mv=np.zeros((50,3))

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
               
               self.h = np.zeros((3,12))

   

            err = -(m_vcm-self.h@dth.T-MVrhs)

            #mvcalc = self.h@dth.T+MVrhs-m_vcm

  
            #print("t", t)

            
         
            #self.h = (m_vcm - self.M*self.v_torso).T@np.linalg.pinv([dth]).T
            eigvec = np.linalg.eig(self.h.T @ self.h)[1]
            eigval = np.linalg.eig(self.h.T @ self.h)[0]
            idx = eigval.argsort()[::-1]   
            eigval = eigval[idx]
            eigvec = eigvec[:,idx]
            
            
            
          
            self.thdvelll = np.reshape(self.thdvel, (12,1))

            self.thdvel = np.reshape(self.thdvel, (12,1))

            thdvel_prev =  np.reshape(thdvel_prev, (12,1))
            
            #eigcomp = self.chooseeig(eigvec, thdvel_prev)
            

      
            
           # eigcomp = self.chooseeig(eigvec,self.thdvelll)
            eigcomp = self.chooseeig(eigvec, thdvel_prev,t)
           

            eigcomp =np.reshape(eigcomp, (12,1))


            #eigcomp =np.reshape(np.real(eigvec[:,2]), (8,1))
           # eigcomp = eigcomp/1000

   
            self.dthdes = -np.linalg.pinv(self.h)@(MVrhs-m_vcm-err).T+eigcomp

            #self.dthdes = np.zeros((8,1))

            #check = self.h@self.dthdes+(MVrhs-m_vcm-err).T-self.h@eigcomp 


           # self.dthdes = np.array([10,10,10,10,10,10,10,10]) 
           # self.dthdes = np.reshape (self.dthdes,(8,1))
  
            self.desiredmomentumfromh = self.h@self.dthdes+Mvrhst

            self.desiredmomentumfromh = np.reshape(self.desiredmomentumfromh,(3,))
            self.thdvell = np.array(self.thdvel)

            global thdes_prev
           
            thdes_prev = self.thdes
  
            
            self.dthdes = np.reshape(self.dthdes,(12,)) 

            self.thdes = np.zeros((12,1))
            #v = np.zeros((8,1))

            
            self.thdes[2] = 1.1
            self.thdes[3] = -1.1


            self.thdes = np.reshape(self.thdes,(12,))
            
 

            self.thpos = np.reshape(self.thpos,(12,))
            self.thdvel = np.reshape(self.thdvel,(12,))
            
            
           # self.dthdes = np.array([0,0,0,0,0,0,0,0]) 

            the = self.thdes-self.thpos

            dthe =  self.dthdes.T-self.thdvel
            dthe =  self.dthdes.T-self.thdvel
            
            the = np.reshape(the,(12,))
            dthe = np.reshape(dthe,(12,))

            self.ddthdes = [00]*len(robot.rev)
            self.ddthdes = np.reshape(self.ddthdes,(12,))
        

            abar = self.ddthdes + self.Kp*the+self.Kd*dthe



            N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp, CP3D = self.getNormalForce_modi()   

            

            n= np.linspace(0.5, 1, 50)
            
            
            PZ = 0 # Height of VHP

            CM =  self.c_cm

            VHP = self.plotvirtualhorizontalplanepoints(CP3D,CM,PZ)

            #print("VHP", VHP)

            if len(VHP)>2:
                
                hull_CP = ConvexHull(CP)

                hull = ConvexHull(VHP)

                area =  hull_CP.volume

                #print('area', area)
             
               # print('hull', hull)
                        
                for zz  in range(len(n)):

                    if abs(t-n[zz])<0.001:
                       vv = 00
                      
                       #self.plott(np.array(CP),hull,zmp, t)
                       
                       #self.plott(np.array(VHP),hull,zmp, t)


              
            
            self.ZMP = zmp
            self.CP= VHP

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

            xxx = np.reshape(xxx,(12,))
  
            if t<self.t_impact+self.t_set+self.t_rest:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx)

      

            elif t<self.t_impact+self.t_set+self.t_rest+10:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx) 

            else: 

              self.dthdes_new = 0.0*(np.array(np.real(self.dthdes))-1*xxx)
 
            
            self.dthdes_new = np.real(self.dthdes_new)
            
            self.dthdes_new = np.reshape(self.dthdes_new,(12,))


            
            self.macm_calc  =  self.F_app+F+self.M*np.array([robot.gravityacc])

            self.force_check = self.F_app+F+self.M*np.array([robot.gravityacc])-self.macm_calc
            
            self.force_check = np.reshape(self.force_check,(3,))
            
            Marhs = np.reshape(Marhs,(3,1))

            self.macm_calc = np.reshape(self.macm_calc,(3,1))

            self.thddacc = np.reshape(self.thddacc,(12,1))

            self.F_app = np.reshape(self.F_app,(3,1))

            dth_ = np.reshape(dth,(12,1))

            self.h_dot = -(self.h@ self.thddacc+ Marhs-self.macm_calc)@np.linalg.pinv([dth]).T


            nn= np.linspace(0, 2, 40)
            
      
            ##function_polyhedral cone    

            xyz = self.macm_calc.T+self.M*np.array([robot.gravityacc])    

            
         

            xyz =  np.reshape(xyz,(3,1))
                        
            for zz  in range(len(nn)):

                if abs(t-nn[zz])<0.001:
                       vv = 00
                       
                       #self.plotpolyhedralconvexcone(N, 0.9, -xyz, t)
                       


            
            

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

            
            
            #p.addUserDebugPoints(self.c_cm, pointColorsRGB = [1,0,0])

           # p.addUserDebugLine([0,0,0],self.c_cm,[1,0,0])
            
            
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

            
            
            """
            p.setJointMotorControlArray(robot.robot_id, robot.rev,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = tau[7:15])
            """
            
            ##ankle torque control
            
            global jtorques_filt_prev
            jtorques_filt_prev = self.jtorques_filt

            wc = 10
            ws = 1/self.t_step1

            wr = wc/ws

            sf = 1 /(1+wr)
            

            for i in range(12):
                self.jtorques_filt[i] = (1-sf)*jtorques_filt_prev[i] + sf*self.jtorques[i]
               #self.jtorques_filt[1] = (1-sf)*jtorques_filt_prev[1] + sf*self.jtorques[1]


            # p.setJointMotorControlArray(robot.robot_id, robot.rev,
            #                             controlMode = p.TORQUE_CONTROL, 
            #                             forces = self.jtorques_filt)

            if t<7:
                p.setJointMotorControlArray(robot.robot_id, robot.rev,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = self.jtorques_filt)
            
            N= N1 + N2
            
            if N == 0:
              beta1 = 0.5
              beta2 = 0.5
            else: 
              beta1 = N1/N
              beta2 = N2/N

            ##ankle torque control
            mgc1 = (self.c_cm[1]-robot.urdf_pos[17][1])*self.M*9.81
            mgc2 = (self.c_cm[1]-robot.urdf_pos[40][1])*self.M*9.81

            if t<7:

                p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[4],
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  0*self.jtorques[4] - ankle_torque_2+beta1*mgc1)

                p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[10],
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  0*self.jtorques[10] - ankle_torque_1+beta2*mgc2) 
            
            self.ankle_torque4 = - ankle_torque_2+beta1*mgc1
            self.ankle_torque10 = - ankle_torque_1+beta2*mgc2

            self.ankle_torque = np.array([self.ankle_torque4,self.ankle_torque10])
            self.ankle_torque = np.reshape(self.ankle_torque,(2,))

            phimax = 0.1
            # T1 =2
            # T2 = 50
            # if t < 7:
            #    if area>1e-3:
            #        hipt1 = self.jtorques[0]
            #        hipt2 = self.jtorques[6]

            #    if area<1e-3:

            #         tmi= t/T2

            #         tm = np.matrix([[0], [tmi], [tmi*tmi], [tmi*tmi*tmi]])

            #         A1 = hipt1*np.linalg.pinv(tm) 
            #         A2 = hipt2*np.linalg.pinv(tm) 

            #         hiptaustrat1 = A1@tm

            #         A2 = hipt2*np.linalg.pinv(tm) 

            #         hiptaustrat2 = A2@tm

            #         #print("hiptaustrat1", hiptaustrat1)

            #         p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.TORQUE_CONTROL,force = hiptaustrat1)

            #         p.setJointMotorControl2(robot.robot_id,robot.rev[6],p.TORQUE_CONTROL,force = hiptaustrat2)


            # phimax = 0.05 # works for Fx = 5, Fy =38, mu =0.9 for both
            # T1 =200
            # T2 = 1000
            
            ## HIp torque control
            """
            if area<1e-2:

                if t<T1:

                    phi = 0*(-2*phimax*t*t*t*(1/(T1*T1*T1)) + 3*phimax*t*t*(1/(T1*T1)))

                    p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.POSITION_CONTROL,targetPosition = phi)

                    p.setJointMotorControl2(robot.robot_id,robot.rev[4],p.POSITION_CONTROL,targetPosition = phi)

                else:

                    phi = 0*(2*phimax*t*t*t*(1/(T2*T2*T2)) - 3*phimax*t*t*(1/(T2*T2)) + phimax)

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


        for t in np.linspace(self.t_set+self.t_impact,self.t_stop,int((self.t_stop-self.t_set-+self.t_impact)/self.t_step2)):
            
            
        #p.changeDynamics(robot.planeID ,  -1, lateralFriction=0.7)
        
         #self.timeee(self,t)
            self.timee = np.array([t,0])
            self.timee = np.reshape(self.timee,(2,))
       
           # p.setRealTimeSimulation(0)
            p.stepSimulation()
          #  p.addUserDebugText(str(t),textPosition = [0,0,0.5], textColorRGB = [1,0,0],textSize = 1, lifeTime = 0)
          
            #robot.getJointState()
            self.getJointState()
            
            robot.drawPbText(str(t))
            #robot.drawPb1(self.c_cm)
            

            self.thdvel = np.array([self.thdvel])

  

            

            thdvel_prev = self.thdvel

            

            self.checkH() #gets the joint velocity self.thdvel

            robot.drawPbPoint(self.c_cm)
    


 
            self.thddacc = (self.thdvel -  thdvel_prev)*(1/self.t_step1)
            
          




            if t > self.t_set+self.t_rest and t < self.t_impact+self.t_set+self.t_rest:                             #This combination works the best comparison to v=[000000]
               # p.applyExternalForce(robot.robot_id,-1,[5,-4,0],[0,0,0],flags=p.LINK_FRAME)
              # p.applyExternalForce(robot.robot_id,-1,[-0,-9.5,-0],[0,0,0],flags=p.LINK_FRAME) #keep this value
               #p.applyExternalForce(robot.robot_id,-1,[8,-8,-0],[0,0,0],flags=p.LINK_FRAME)
               self.F_app = np.array([0,0,0])
               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            else:
               self.F_app = np.array([0,0,-0])    

               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            p.changeDynamics(robot.robot_id ,  -1, mass=0.05)  #changing mass of base of biped
            
            #v = 0.5*np.linalg.pinv(self.h)@(self.M*self.v_torso) + 0*eigvec[4]
            
           
            
            #v=list(v)

            thd=p.getJointStates(robot.robot_id,robot.rev)



            if t<0.01+self.t_set+self.t_rest:


             
              thinitial = self.q
              
            


            self.M = np.sum(robot.setLinkM())
 
            self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])

            #self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[8]*robot.link_vel[8]+robot.m_array[16]*self.v_torso)/(robot.m_array[0]+robot.m_array[8]+robot.m_array[16])
            
            self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[1]*robot.link_vel[1]+\
                         +robot.m_array[2]*robot.link_vel[2]+robot.m_array[3]*robot.link_vel[3]+\
                         +robot.m_array[26]*robot.link_vel[26]+robot.m_array[49]*self.v_torso/(robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]))        

        
        #dth = np.delete(robot.qdot,[0,4])

            qdot = self.getJointState()[2]
        
            dth=np.array(qdot)
       

        #vbase_r= robot.base_vel+self.skewM(robot.base_angvel)@(robot.urdf_pos[0]--robot.com_base_pos)
            vbase_r= robot.link_vel[3]+self.skewM(robot.link_omega[3])@(robot.urdf_pos[4]-robot.com_pos[3])

            vbase_l= robot.link_vel[26]+self.skewM(robot.link_omega[26])@(robot.urdf_pos[27]-robot.com_pos[26])

         #   x1=vbase_l+self.skewM(robot.link_omega[9])@(-robot.urdf_pos[9]+robot.com_pos[9])


            Mr = np.sum(robot.setLinkM()[4:26])
            Ml = np.sum(robot.setLinkM()[27:49])

            #M_base= robot.setLinkM()[16]+robot.setLinkM()[0]+robot.setLinkM()[8]

            M_base = robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]
        
            modi_link_mass = robot.setLinkM() #same just base mass at the end

            modi_link_vel = np.append(robot.link_vel,[self.v_torso],axis=0)

            vcmdata=robot.getrobotCOMvelocity(modi_link_mass, modi_link_vel)

            vrightcm = vcmdata[0]; vleftcm = vcmdata[1]; vbasecm = vcmdata[2]; vtotalcm =  vcmdata[3]
                   
            link_vell = np.append(robot.link_vel,[self.v_torso],axis=0)

            

            m_vcm_prev = m_vcm
     
        
            mv=np.zeros((50,3))

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
               
               self.h = np.zeros((3,12))

   

            err = -(m_vcm-self.h@dth.T-MVrhs)

            #mvcalc = self.h@dth.T+MVrhs-m_vcm

  
            #print("t", t)

            
         
            #self.h = (m_vcm - self.M*self.v_torso).T@np.linalg.pinv([dth]).T
            eigvec = np.linalg.eig(self.h.T @ self.h)[1]
            eigval = np.linalg.eig(self.h.T @ self.h)[0]
            idx = eigval.argsort()[::-1]   
            eigval = eigval[idx]
            eigvec = eigvec[:,idx]
            
            
            
          
            self.thdvelll = np.reshape(self.thdvel, (12,1))

            self.thdvel = np.reshape(self.thdvel, (12,1))

            thdvel_prev =  np.reshape(thdvel_prev, (12,1))
            
            #eigcomp = self.chooseeig(eigvec, thdvel_prev)
            

      
            
           # eigcomp = self.chooseeig(eigvec,self.thdvelll)
            eigcomp = self.chooseeig(eigvec, thdvel_prev,t)
           

            eigcomp =np.reshape(eigcomp, (12,1))


            #eigcomp =np.reshape(np.real(eigvec[:,2]), (8,1))
           # eigcomp = eigcomp/1000

   
            self.dthdes = -np.linalg.pinv(self.h)@(MVrhs-m_vcm-err).T+eigcomp

            #self.dthdes = np.zeros((8,1))

            #check = self.h@self.dthdes+(MVrhs-m_vcm-err).T-self.h@eigcomp 


           # self.dthdes = np.array([10,10,10,10,10,10,10,10]) 
           # self.dthdes = np.reshape (self.dthdes,(8,1))
  
            self.desiredmomentumfromh = self.h@self.dthdes+Mvrhst

            self.desiredmomentumfromh = np.reshape(self.desiredmomentumfromh,(3,))
            self.thdvell = np.array(self.thdvel)

            
           
            thdes_prev = self.thdes
  
            
            self.dthdes = np.reshape(self.dthdes,(12,)) 

            self.thdes = np.zeros((12,1))
            #v = np.zeros((8,1))

            
            self.thdes[2] = 1.1
            self.thdes[3] = -1.1


            self.thdes = np.reshape(self.thdes,(12,))
            
 

            self.thpos = np.reshape(self.thpos,(12,))
            self.thdvel = np.reshape(self.thdvel,(12,))
            
            
           # self.dthdes = np.array([0,0,0,0,0,0,0,0]) 

            the = self.thdes-self.thpos

            dthe =  self.dthdes.T-self.thdvel
            dthe =  self.dthdes.T-self.thdvel
            
            the = np.reshape(the,(12,))
            dthe = np.reshape(dthe,(12,))

            self.ddthdes = [00]*len(robot.rev)
            self.ddthdes = np.reshape(self.ddthdes,(12,))
        

            abar = self.ddthdes + self.Kp*the+self.Kd*dthe



            N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp, CP3D = self.getNormalForce_modi()   

            

            n= np.linspace(0.6, 15, 300)
            
            
            PZ = 0 # Height of VHP

            CM =  self.c_cm

            #VHP is Virtual Horizontal Plane. It's concept is mentioned in the appendix of the thesis.

            VHP = self.plotvirtualhorizontalplanepoints(CP3D,CM,PZ)

            #print("VHP", VHP)

            

            if len(VHP)>2:
                
                hull_CP = ConvexHull(CP)

                hull = ConvexHull(VHP)

                area =  hull_CP.volume

                
                        
                for zz  in range(len(n)):

                    if abs(t-n[zz])<0.1:
                       vv = 00
                      
                       #self.plott(np.array(CP),hull,zmp, t)
                       
                       #self.plott(np.array(VHP),hull,zmp, t)


              
            
            self.ZMP = zmp
            self.CP= VHP

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

            xxx = np.reshape(xxx,(12,))
  
            if t<self.t_impact+self.t_set+self.t_rest:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx)

      

            elif t<self.t_impact+self.t_set+self.t_rest+10:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx) 

            else: 

              self.dthdes_new = 0.0*(np.array(np.real(self.dthdes))-1*xxx)
 
            
            self.dthdes_new = np.real(self.dthdes_new)
            
            self.dthdes_new = np.reshape(self.dthdes_new,(12,))


            
            self.macm_calc  =  self.F_app+F+self.M*np.array([robot.gravityacc])

            self.force_check = self.F_app+F+self.M*np.array([robot.gravityacc])-self.macm_calc
            
            self.force_check = np.reshape(self.force_check,(3,))
            
            Marhs = np.reshape(Marhs,(3,1))

            self.macm_calc = np.reshape(self.macm_calc,(3,1))

            self.thddacc = np.reshape(self.thddacc,(12,1))

            self.F_app = np.reshape(self.F_app,(3,1))

            dth_ = np.reshape(dth,(12,1))

            self.h_dot = -(self.h@ self.thddacc+ Marhs-self.macm_calc)@np.linalg.pinv([dth]).T


            nn= np.linspace(0, 2, 40)
            
      
            ##function_polyhedral cone    

            xyz = self.macm_calc.T+self.M*np.array([robot.gravityacc])    

            
         

            xyz =  np.reshape(xyz,(3,1))
                        
            for zz  in range(len(nn)):

                if abs(t-nn[zz])<0.001:
                       vv = 00
                       
                       #self.plotpolyhedralconvexcone(N, 0.9, -xyz, t)
                       


            
            

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

            
            
            #p.addUserDebugPoints(self.c_cm, pointColorsRGB = [1,0,0])

           # p.addUserDebugLine([0,0,0],self.c_cm,[1,0,0])
            
            
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

            
            
            """
            p.setJointMotorControlArray(robot.robot_id, robot.rev,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = tau[7:15])
            """
            
            
            
            jtorques_filt_prev = self.jtorques_filt

            wc = 1
            ws = 1/self.t_step2

            wr = wc/ws

            sf = 1 /(1+wr)
            

            for i in range(12):
                self.jtorques_filt[i] = (1-sf)*jtorques_filt_prev[i] + sf*self.jtorques[i]
               #self.jtorques_filt[1] = (1-sf)*jtorques_filt_prev[1] + sf*self.jtorques[1]


            
            if t<15:
                p.setJointMotorControlArray(robot.robot_id, robot.rev,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = self.jtorques_filt)
            
            N= N1 + N2
            
            if N == 0:
               beta1 = 0.5
               beta2 = 0.5 
            else: 
               beta1 = N1/N
               beta2 = N2/N
            
           # beta1 = N1/N
           # beta2 = N2/N
            ##ankle torque control
            mgc1 = (self.c_cm[1]-robot.urdf_pos[17][1])*self.M*9.81
            mgc2 = (self.c_cm[1]-robot.urdf_pos[40][1])*self.M*9.81

            if t<15:

                p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[4],
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  0*self.jtorques[4] - ankle_torque_2+beta1*mgc1)

                p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[10],
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  0*self.jtorques[10] - ankle_torque_1+beta2*mgc2) 
            
            self.ankle_torque4 = - ankle_torque_2+beta1*mgc1
            self.ankle_torque10 = - ankle_torque_1+beta2*mgc2

            self.ankle_torque = np.array([self.ankle_torque4,self.ankle_torque10])
            self.ankle_torque = np.reshape(self.ankle_torque,(2,))

            phimax = 0.1
            # T1 =2
            # T2 = 50
            # if t < 7:
            #    if area>1e-3:
            #        hipt1 = self.jtorques[0]
            #        hipt2 = self.jtorques[6]

            #    if area<1e-3:

            #         tmi= t/T2

            #         tm = np.matrix([[0], [tmi], [tmi*tmi], [tmi*tmi*tmi]])

            #         A1 = hipt1*np.linalg.pinv(tm) 
            #         A2 = hipt2*np.linalg.pinv(tm) 

            #         hiptaustrat1 = A1@tm

            #         A2 = hipt2*np.linalg.pinv(tm) 

            #         hiptaustrat2 = A2@tm

            #         #print("hiptaustrat1", hiptaustrat1)

            #         p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.TORQUE_CONTROL,force = hiptaustrat1)

            #         p.setJointMotorControl2(robot.robot_id,robot.rev[6],p.TORQUE_CONTROL,force = hiptaustrat2)
            # phimax = 0.05 # works for Fx = 5, Fy =38, mu =0.9 for both
            # T1 =200
            # T2 = 1000
            
            ## HIp torque control
            """
            if area<1e-2:

                if t<T1:

                    phi = 0*(-2*phimax*t*t*t*(1/(T1*T1*T1)) + 3*phimax*t*t*(1/(T1*T1)))

                    p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.POSITION_CONTROL,targetPosition = phi)

                    p.setJointMotorControl2(robot.robot_id,robot.rev[4],p.POSITION_CONTROL,targetPosition = phi)

                else:

                    phi = 0*(2*phimax*t*t*t*(1/(T2*T2*T2)) - 3*phimax*t*t*(1/(T2*T2)) + phimax)

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
      
        """
        thdfixed = p.getJointStates(robot.robot_id,[0,2,4,8,10,12])
        
        
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

        self.dthdes = np.reshape(self.dthdes,(12,))

        self.thdes = np.reshape(self.thdes,(12,))

        self.jointvelerr = self.dthdes.T-self.thdvel

        self.jointposerr = self.thdes.T-self.thpos

        robot.getCOMPos()

        

        
        

        self.M = np.sum(robot.setLinkM())
 
        self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])

            #self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[8]*robot.link_vel[8]+robot.m_array[16]*self.v_torso)/(robot.m_array[0]+robot.m_array[8]+robot.m_array[16])
        """
        self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[1]*robot.link_vel[1]+\
                      +robot.m_array[2]*robot.link_vel[2]+robot.m_array[3]*robot.link_vel[3]+\
                      +robot.m_array[26]*robot.link_vel[26]+robot.m_array[49]*self.v_torso/(robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]))        

        
        #dth = np.delete(robot.qdot,[0,4])

        qdot = self.getJointState()[2]
        
        dth=np.array(qdot)
       

        #vbase_r= robot.base_vel+self.skewM(robot.base_angvel)@(robot.urdf_pos[0]--robot.com_base_pos)
        vbase_r= robot.link_vel[3]+self.skewM(robot.link_omega[3])@(robot.urdf_pos[4]-robot.com_pos[3])

        vbase_l= robot.link_vel[26]+self.skewM(robot.link_omega[26])@(robot.urdf_pos[27]-robot.com_pos[26])

         #   x1=vbase_l+self.skewM(robot.link_omega[9])@(-robot.urdf_pos[9]+robot.com_pos[9])


        Mr = np.sum(robot.setLinkM()[4:26])
        Ml = np.sum(robot.setLinkM()[27:49])

            #M_base= robot.setLinkM()[16]+robot.setLinkM()[0]+robot.setLinkM()[8]

        M_base = robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]
        
        modi_link_mass = robot.setLinkM() #same just base mass at the end

        modi_link_vel = np.append(robot.link_vel,[self.v_torso],axis=0)

        vcmdata=robot.getrobotCOMvelocity(modi_link_mass, modi_link_vel)

        vrightcm = vcmdata[0]; vleftcm = vcmdata[1]; vbasecm = vcmdata[2]; vtotalcm =  vcmdata[3]

        """

        link_vell = np.append(robot.link_vel,[self.v_torso],axis=0)

        link_pos = np.append(robot.com_pos,[robot.com_base_pos],axis=0)

            

        #m_vcm_prev = m_vcm
     
        
        mv=np.zeros((50,3))
        mc=np.zeros((50,3))

        #print("len(robot.m_array)", len(robot.m_array))

        for i in range(len(robot.m_array)):
            mv[i,:]=robot.m_array[i]*link_vell[i,:]
            mc[i,:]=robot.m_array[i]*link_pos[i,:]    
        
       
        
        mvcmx=0
        #for i in range(len(robot.m_array)):
        mvcmx=sum(mv[:,0])
        mvcmy=sum(mv[:,1])
        mvcmz=sum(mv[:,2])

        mccmx=sum(mc[:,0])
        mccmy=sum(mc[:,1])
        mccmz=sum(mc[:,2])
        
        #m_vcm=np.matrix([[mvcmx],[mvcmy], [mvcmz]])
        #m_vcm=np.array([mvcmx,mvcmy,mvcmz])

        
        #m_vcm=np.matrix([[mvcmx],[mvcmy], [mvcmz]])
        m_vcm=np.array([mvcmx,mvcmy,mvcmz])
        m_ccm=np.array([mccmx,mccmy,mccmz])
       
       

       #m_vcm = np.sum(np.tile(robot.m_array,(3,1))*link_vell.T,axis=1)

        self.v_cm = m_vcm/self.M

        self.c_cm = m_ccm/self.M

        
      
        
        

        #dthdes = np.linalg.pinv(self.h)@mvefft
        
        #self.H = cj.Dynamics.getH(self, link_com_pos=robot.com_pos, link_orien= robot.urdf_orien, link_pos=robot.urdf_pos, com_base_pos=np.array(robot.com_base_pos), jaxeslocal=robot.jdetails)
        #self.com_vel = cj.Dynamics.verifyH(self, link_vel= robot.link_vel, base_vel= robot.base_vel)
        #MVCM = self.M * self.com_vel
        #check=m_vcm-MVCM
        
      

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
        filename18 = "ankletorqueangmom.csv"

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

        with open(filename18, write_type) as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.ankle_torque)             


if __name__ == "__main__":
    robot = Biped()
   # traj = GenTraj()
    s1 = BipedSim()
   # p1 = np.array([-0.17,0.17,-0.05])
   ## p2 = np.array([ 0.07,-0.07,0.1])
   # p3 = np.array([-0.17,0.17,-0.05])
    #traj.setTrajPt(**{"p1":p1,"p2": p2,"p3": p3})
    s1.runSim()
    

