from cmath import nan
from re import M
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
      
        self.t_step1 = 1/240 #During Force application (Required to realise full impact)
        self.t_step2 = 17/240 #Real time step
        self.file_written = False
       
        self.Kp = 10
        self.Kd = 2*np.sqrt(10)
 
        self.linkmass_data=[]
        self.linkmass = []
        self.jointvelerr = []
        self.momentumerror = []
        self.desiredmomentumfromh = []
        self.actualmomentumfromh = []
        self.Reaction_force = []
        self.timee = []
        self.ZMP = []
      
        self.modi_a_link_mass = np.zeros((10))
        self.modi_b_link_mass = np.zeros((10))
    """ 
    def timeee(self,t):
        total_time = self.t_stop:
        return  total_time
    """
    def RR_modi(self,quat): #rotation matrix from quaternion
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
        return self.tpos - robot.q
    
    def edot(self):
        return self.tdpos - robot.qdot

    def getNormalForce_modi(self): #Getting normal forces

        p.setRealTimeSimulation(0)

        p.stepSimulation()

        f1a = p.getContactPoints(robot.robot_id,robot.planeID,47,-1)
        f2a = p.getContactPoints(robot.robot_id,robot.planeID,24,-1)

        f1b = p.getContactPoints(robot.robot_id,robot.planeID,48,-1)
        f2b = p.getContactPoints(robot.robot_id,robot.planeID,25,-1)
        

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

          
            C1a.append(f1a[i][5]) #contact point on robot

            N1C1x.append(f1a[i][9]*f1a[i][6][0])
            N1C1y.append(f1a[i][9]*f1a[i][6][1])

    

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

            CP.append(f1b[i][6][0:2]) #contact point on ground

            C1b.append(f1b[i][5]) #contact point on robot

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

            CP.append(f2a[i][6][0:2]) #contact point on ground

            C2a.append(f2a[i][5]) #contact point on robot

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

            CP.append(f2b[i][6][0:2]) #contact point on ground

            C2b.append(f2b[i][5]) #contact point on robot
        

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

        return N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp   

    def chooseeig(self,v,thd):

        xnorm=np.zeros((12,1))
       
        for i in range(len(v)):
          x= v[:,i]-thd
          xnorm[i]=np.linalg.norm(x)
        
        xnormmin = min(xnorm)

        for k in range(len(v)):

          if  xnorm[k] ==  xnormmin:
            
            choseneig =  3.2*v[:,k]

        

        return choseneig

          
    def plott(self,points,hull, zmp, t): #For ZMP and COntact Points plot
              
        import matplotlib.pyplot as plt
        plt.rc('font', size=20)
        plt.plot(points[:,0], points[:,1], 'o')

        for simplex in hull.simplices:
          plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
        
        
        plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
        #plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'ro')
        plt.plot(zmp[0],zmp[1], 'gs')

        plt.title("t=" + str(t))
        plt.show() 


    def runSim(self):
        
        nj = 12; #number of revolute joints

        self.dthdes = np.zeros((nj))
        self.thdes = np.zeros((nj))
        self.ddthdes = np.zeros((nj))
        self.jtorques = np.zeros((nj))
        self.jtorques_filt = np.zeros((nj))
        self.thdvel =  np.zeros((nj))
        self.dthdes_new = np.zeros((nj))
        self.t_impact = 0.05*2
        self.F_app = np.array([0,-27.2,-0])
        self.c_cm = np.zeros((3))
        MVrhs = np.zeros((3))
        m_vcm = np.zeros((3))
        xin = np.matrix([[0],[0]])
        p.changeDynamics(robot.planeID ,  -1, lateralFriction=0.5)
        #time.sleep(0.5)
    
        #There are two time loops for this contact condition. Fotr other two contact conditions there are three loops for each
        # code. The necessary comments to denote meaning of each line in the codes are mentioned in the next time loop. Those informations
        # are sufficient to understand the other time loops (for loops in code) for all contact conditions.
        for t in np.linspace(0,0+self.t_impact,int(self.t_impact/self.t_step1)):
            
            self.timee = np.array([t,0])
            self.timee = np.reshape(self.timee,(2,))

           # p.setRealTimeSimulation(0)
            p.stepSimulation()
            p.removeAllUserDebugItems
            robot.getJointState()
            robot.drawPbText(str(t))
            robot.drawPbPoint(self.c_cm)


            global thdvel_prev

            thdvel_prev = self.thdvel

            
            self.checkH() #gets the joint velocity self.thdvel


            self.thddacc = (self.thdvel -  thdvel_prev)*(1/self.t_step1)
            

         
            if t < self.t_impact:                            
              
               self.F_app = np.array([-15*1.12/3,-24*1.12/3,0])
               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            else:
               self.F_app = np.array([0,0,-0])    

               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)
            

            thd=p.getJointStates(robot.robot_id,robot.rev)

            if t<0.000001:
             
              thinitial = robot.q #To set up desired joint angles; used later
              
        
            

            self.M = np.sum(robot.setLinkM())

            
      
        
        
       
            self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])

            
            
            self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[1]*robot.link_vel[1]+\
                         +robot.m_array[2]*robot.link_vel[2]+robot.m_array[3]*robot.link_vel[3]+\
                         +robot.m_array[26]*robot.link_vel[26]+robot.m_array[49]*self.v_torso/(robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]))
                        
        
     
        
        
            dth=np.array(robot.qdot)
       

       
            # Velocity at Hip Joint Origin at Right and Left Leg
            
            vbase_r= robot.link_vel[3]+self.skewM(robot.link_omega[3])@(robot.urdf_pos[4]-robot.com_pos[3]) 
            vbase_l= robot.link_vel[26]+self.skewM(robot.link_omega[26])@(robot.urdf_pos[27]-robot.com_pos[26])

         


            Mr = np.sum(robot.setLinkM()[4:26]) #Right leg mass
            Ml = np.sum(robot.setLinkM()[27:49]) #Left leg mass

           #Total torso mass

            M_base = robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]
        
            modi_link_mass = robot.setLinkM() #mass array with torso mass at the end

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

            mvcmx=sum(mv[:,0])
            mvcmy=sum(mv[:,1])
            mvcmz=sum(mv[:,2])
        
       
            m_vcm=np.array([mvcmx,mvcmy,mvcmz])
       
       
            m_acm = (m_vcm- m_vcm_prev)*(1/self.t_step1)
       

            
            self.v_cm = vtotalcm #CoM velocity

            mveff= np.array([(Mr*vrightcm + Ml*vleftcm + M_base*vbasecm)])
            mvefft = mveff.T
            
            self.momentumerror =  mveff
            self.momentumerror = np.reshape(self.momentumerror,(3,))
           

            global MVrhs_prev

            MVrhs_prev = MVrhs



           
            MVrhs = np.array([Mr*vbase_r + Ml*vbase_l + M_base*self.v_base])

            Marhs = (MVrhs-MVrhs_prev)*(1/self.t_step1)
            
           # print("np.linalg.norm(dth)",np.linalg.norm(dth))

            Mvrhst = MVrhs.T

            #The below if-else condition is not required

            #Self.h is the linear momentum jacobian matrix. Calculated this way because the calculation with analytical expressions require 
            #accurate calcul;ation of vectors from origin to CoM of each link which Pybullet could not perform.
            if np.linalg.norm(dth)== nan: 
              dthio = 0.02*np.ones((12,1))
              self.h = -Mvrhst@np.linalg.pinv([dthio]).T
            else:
              self.h = -Mvrhst@np.linalg.pinv([dth]).T

            err = -(m_vcm-self.h@dth.T-MVrhs)


            eigvec = np.linalg.eig(self.h.T @ self.h)[1]
            eigval = np.linalg.eig(self.h.T @ self.h)[0]
            idx = eigval.argsort()[::-1]   
            eigval = eigval[idx]
            eigvec = eigvec[:,idx]

            
            
 

            self.thdvelll = np.reshape(self.thdvel, (12,1))

            thdvel_prev =  np.reshape(thdvel_prev, (12,1))

  

            
  
            eigcomp = self.chooseeig(eigvec, thdvel_prev)
           

            eigcomp =np.reshape(eigcomp, (12,1))


  
            self.dthdes = -np.linalg.pinv(self.h)@(MVrhs-m_vcm-err).T+ eigcomp 

            check = self.h@self.dthdes+(MVrhs-m_vcm-err).T-self.h@eigcomp 

 
            self.desiredmomentumfromh = self.h@self.dthdes+Mvrhst

            self.desiredmomentumfromh = np.reshape(self.desiredmomentumfromh,(3,))
            self.thdvell = np.array(self.thdvel)

            global thdes_prev

            global dthdes_prev
           
            thdes_prev = self.thdes

            dthdes_prev = self.dthdes_new
  
            
            self.dthdes = np.reshape(self.dthdes,(12,)) 
            
            self.thdes = thinitial
  
            self.thpos = np.reshape(self.thpos,(12,))
            self.thdvel = np.reshape(self.thdvel,(12,))
            
            
         
            the = self.thdes-self.thpos

            dthe =  self.dthdes.T-self.thdvel
            dthe =  self.dthdes.T-self.thdvel
            
            the = np.reshape(the,(12,))
            dthe = np.reshape(dthe,(12,))

            self.ddthdes = [00]*len(robot.rev)
            self.ddthdes = np.reshape(self.ddthdes,(12,))
        

            abar = self.ddthdes + self.Kp*the+self.Kd*dthe


            


            N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp = self.getNormalForce_modi()   

           
            
            n= np.linspace(0, 1, 10)
            
            if len(CP)>2:
             
                hull = ConvexHull(CP)

                area =  hull.volume

              # print('area', area)
             
                #print('hull', hull)
                        
                for zz  in range(len(n)):

                    if abs(t-n[zz])<0.1:
                       vv = 00
                       #self.plott(np.array(CP),hull,zmp, t)


           
            
            self.ZMP = zmp

            fric1 = fric1y*fric1y_dir+fric1x*fric1x_dir
            fric2 = fric2y*fric2y_dir+fric2x*fric2x_dir

          

            fric = fric1+fric2 #lateral reaction force from ground to feet

            N =N1+N2
            
            F =np.array([0,0,N])+fric

            self.Reaction_force = np.reshape(F,(3,))

            fric_norm = np.linalg.norm(fric)

                      
            x_in_prev = xin

           
            
            
            X = opt.runOptimization(self.h,self.thdvel, MVrhs) #Input to Optmization
            
            

            xin = X

            
            X1 = np.array([X[0],X[1],0]) 

            
            
            Mvrhs = np.reshape(MVrhs,(3,))
          
            xxx=np.real(np.linalg.pinv(self.h)@X1)

            Mvrhsxxx = 1*np.real(np.linalg.pinv(self.h)@Mvrhs)

            xxx = np.reshape(xxx,(12,))

            #Calculating desired joint velocity after optimization

            if t<self.t_impact:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx)

             

            elif t<self.t_impact+10:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx) 

            else: 

              self.dthdes_new = 0.0*(np.array(np.real(self.dthdes))-1*xxx)
          
            
            self.dthdes_new = np.real(self.dthdes_new)
            
            self.dthdes_new = np.reshape(self.dthdes_new,(12,))

           #calculating jacobian 
 
            # rho16 = robot.urdf_pos[24]-robot.urdf_pos[4]
            # rho26 = robot.urdf_pos[24]-robot.urdf_pos[7]
            # rho36 = robot.urdf_pos[24]-robot.urdf_pos[12]
            # rho46 = robot.urdf_pos[24]-robot.urdf_pos[16]
            # rho56 = robot.urdf_pos[24]-robot.urdf_pos[17]
            # rho66 = robot.urdf_pos[24]-robot.urdf_pos[24]

            # #RR_modi





            self.macm_calc  =  self.F_app+F+self.M*np.array([robot.gravityacc])

            self.force_check = self.F_app+F+self.M*np.array([robot.gravityacc])-self.macm_calc
            
            self.force_check = np.reshape(self.force_check,(3,))
            
            Marhs = np.reshape(Marhs,(3,1))

            self.macm_calc = np.reshape(self.macm_calc,(3,1))

            self.thddacc = np.reshape(self.thddacc,(12,1))

            self.F_app = np.reshape(self.F_app,(3,1))

            dth_ = np.reshape(dth,(12,1))

            self.h_dot = -(self.h@ self.thddacc+ Marhs-self.macm_calc)@np.linalg.pinv([dth]).T



            Cx = [1,0,0]
            Cy = [0,1,0]
            xtild = [0.15,0,0]
            ytild = [0,0.15,0]

            Cx= np.reshape(Cx,(1,3))
            Cy= np.reshape(Cy,(1,3))

            xtild= np.reshape(xtild,(3,1))
            ytild= np.reshape(ytild,(3,1))
            
            self.F_est = self.macm_calc-F-self.M*np.array([robot.gravityacc])
            
            #Friction condition check in X-direction, could not do it directly on the optimization. But this works mostly.
            #Same way to do for Y-direction

            if abs(self.Reaction_force[0])> abs(robot.mu*N/np.sqrt(1)):

               a1i =  str('friction limit exceeded_1')

               frx = Cx@(self.h@ self.thddacc+ self.h_dot@dth_+Marhs-self.F_app)

               

               if frx>0:

                self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs+robot.mu*N+self.F_app-xtild)

               else:

                self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs-robot.mu*N+self.F_app+xtild)    

             
                       
            else:

                a1i =  str('friction ok_1')

              

            
            if abs(self.Reaction_force[1])> abs(robot.mu*N/np.sqrt(1)):

                a2i =  str('friction limit exceeded_2')

                fry = Cy@(self.h@ self.thddacc+ self.h_dot@dth_+Marhs-self.F_app)

                

                if fry>0:

                 self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs+robot.mu*N+self.F_app-ytild)

                else:

                 self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs-robot.mu*N+self.F_app+ytild)  

               
                       
            else:

                a2i =  str('friction ok_2')

             
            
            self.fric_info =  np.array([t,a1i,a2i])


            #Position control needs to be activated for torque control given next
            p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes,
                                    targetVelocities=self.dthdes_new)
           

           

            dthe =  self.dthdes_new.T-self.thdvel
            
            the = np.reshape(the,(12,))
            dthe = np.reshape(dthe,(12,))


            self.dthdes_new = np.reshape(self.dthdes_new,(12,))

            self.jointvelerr = dthe

         #   print ("self.dthdes_new", self.dthdes_new)

            self.ddthdes = (self.dthdes_new-dthdes_prev)*(0/self.t_step1)
            self.ddthdes = np.reshape(self.ddthdes,(12,))
        

            abar = self.ddthdes + self.Kp*the+self.Kd*dthe

          #  tau = p.calculateInverseDynamics(robot.robot_id, list(self.thdes), list(self.dthdes_new), list(abar), flags =1)



            

            jointstates = p.getJointStates(robot.robot_id,robot.rev)
            
            #Required input torque 

            self.jtorques = robot.applied_torques


            global jtorques_filt_prev
            jtorques_filt_prev = self.jtorques_filt
            
            #Torque filtered with low-pass frequency
            wc = 15
            ws = 1/self.t_step1

            wr = wc/ws

            sf = 1 /(1+wr)
            

            for i in range(12):
                self.jtorques_filt[i] = (1-sf)*jtorques_filt_prev[i] + sf*self.jtorques[i]
               #self.jtorques_filt[1] = (1-sf)*jtorques_filt_prev[1] + sf*self.jtorques[1]

            #Torque Control 
            p.setJointMotorControlArray(robot.robot_id, robot.rev,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = self.jtorques_filt)
            
            
            ##ankle torque control
            mgc1 = (self.c_cm[1]-robot.urdf_pos[17][1])*M*9.81
            mgc2 = (self.c_cm[1]-robot.urdf_pos[40][1])*M*9.81

            #if t<self.t_impact+0.3:

            p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[4] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  - ankle_torque_2+0.5*mgc1)

            p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[10] ,
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  - ankle_torque_1+0.5*mgc2) 
            

            #Wrote the next four lines to claculate linear momentum. However, that's not necessary .
           

            dth=np.array([robot.qdot])

            dthT = dth.T

           


            self.actualmomentumfromh = self.h@dthT+Mvrhst

            

            self.actualmomentumfromh = np.reshape(self.actualmomentumfromh ,(3,))

            
            
            #For exporting all the values to an excel file
            self.write_data()
           


        

        for t in np.linspace(self.t_impact,self.t_impact+self.t_stop,int((self.t_stop-self.t_impact)/self.t_step2)):
            
            self.timee = np.array([t,0])
            self.timee = np.reshape(self.timee,(2,))

           # p.setRealTimeSimulation(0)
            p.stepSimulation()
            robot.getJointState()
            
            robot.drawPbText(str(t))
            robot.drawPbPoint(self.c_cm)
            p.removeAllUserDebugItems


            

            thdvel_prev = self.thdvel

            
            self.checkH() #gets the joint velocity self.thdvel


            self.thddacc = (self.thdvel -  thdvel_prev)*(1/self.t_step1)
            

         
            if t < self.t_impact:                             #This combination works the best comparison to v=[000000]
               # p.applyExternalForce(robot.robot_id,-1,[5,-4,0],[0,0,0],flags=p.LINK_FRAME)
              # p.applyExternalForce(robot.robot_id,-1,[-0,-9.5,-0],[0,0,0],flags=p.LINK_FRAME) #keep this value
               #p.applyExternalForce(robot.robot_id,-1,[8,-8,-0],[0,0,0],flags=p.LINK_FRAME)
               self.F_app = np.array([0,-0,0])
               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)

            else:
               self.F_app = np.array([0,0,-0])    

               p.applyExternalForce(robot.robot_id,-1,self.F_app,[0,0,0],flags=p.LINK_FRAME)
            

            thd=p.getJointStates(robot.robot_id,robot.rev)

            if t<0.000001:
             
              thinitial = robot.q
              
        
            

            self.M = np.sum(robot.setLinkM())

            
      
        
        
       
            self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])

            #self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[8]*robot.link_vel[8]+robot.m_array[16]*self.v_torso)/(robot.m_array[0]+robot.m_array[8]+robot.m_array[16])
            
            self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[1]*robot.link_vel[1]+\
                         +robot.m_array[2]*robot.link_vel[2]+robot.m_array[3]*robot.link_vel[3]+\
                         +robot.m_array[26]*robot.link_vel[26]+robot.m_array[49]*self.v_torso/(robot.m_array[0]+robot.m_array[1]+robot.m_array[2]+robot.m_array[3]+robot.m_array[26]+robot.m_array[49]))
                         #+robot.m_array[16]*self.v_torso)/(robot.m_array[0]+robot.m_array[8]+robot.m_array[16])

        
     
        
        #dth = np.delete(robot.qdot,[0,4])
            dth=np.array(robot.qdot)
        #dthact=dth.T

       
          #  vbase_r= robot.link_vel[0]+self.skewM(robot.link_omega[0])@(robot.urdf_pos[1]-robot.com_pos[0])
          #  vbase_l= robot.link_vel[8]+self.skewM(robot.link_omega[8])@(robot.urdf_pos[9]-robot.com_pos[8])
            
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

            mvcmx=sum(mv[:,0])
            mvcmy=sum(mv[:,1])
            mvcmz=sum(mv[:,2])
        
       
            m_vcm=np.array([mvcmx,mvcmy,mvcmz])
       
       
            m_acm = (m_vcm- m_vcm_prev)*(1/self.t_step1)
       

            #self.v_cm = m_vcm/self.M
            self.v_cm = vtotalcm

            mveff= np.array([(Mr*vrightcm + Ml*vleftcm + M_base*vbasecm)])
            mvefft = mveff.T
            #self.momentumerror =  mveff-m_vcm
            self.momentumerror =  mveff
            self.momentumerror = np.reshape(self.momentumerror,(3,))
           

            

            MVrhs_prev = MVrhs



           
            MVrhs = np.array([Mr*vbase_r + Ml*vbase_l + M_base*self.v_base])

            Marhs = (MVrhs-MVrhs_prev)*(1/self.t_step1)
            


            Mvrhst = MVrhs.T
            self.h = -Mvrhst@np.linalg.pinv([dth]).T

            err = -(m_vcm-self.h@dth.T-MVrhs)


            eigvec = np.linalg.eig(self.h.T @ self.h)[1]
            eigval = np.linalg.eig(self.h.T @ self.h)[0]
            idx = eigval.argsort()[::-1]   
            eigval = eigval[idx]
            eigvec = eigvec[:,idx]

            
            
 

            self.thdvelll = np.reshape(self.thdvel, (12,1))

            thdvel_prev =  np.reshape(thdvel_prev, (12,1))

  

            
  
            eigcomp = self.chooseeig(eigvec, thdvel_prev)
           

            eigcomp =np.reshape(eigcomp, (12,1))


  
            self.dthdes = -np.linalg.pinv(self.h)@(MVrhs-m_vcm-err).T+ eigcomp 

            check = self.h@self.dthdes+(MVrhs-m_vcm-err).T-self.h@eigcomp 

 
            self.desiredmomentumfromh = self.h@self.dthdes+Mvrhst

            self.desiredmomentumfromh = np.reshape(self.desiredmomentumfromh,(3,))
            self.thdvell = np.array(self.thdvel)

            
           
            thdes_prev = self.thdes
  
            
            self.dthdes = np.reshape(self.dthdes,(12,)) 
            
            self.thdes = thinitial
  
            self.thpos = np.reshape(self.thpos,(12,))
            self.thdvel = np.reshape(self.thdvel,(12,))
            
            
         
            the = self.thdes-self.thpos

            dthe =  self.dthdes.T-self.thdvel
            dthe =  self.dthdes.T-self.thdvel
            
            the = np.reshape(the,(12,))
            dthe = np.reshape(dthe,(12,))

            self.ddthdes = [00]*len(robot.rev)
            self.ddthdes = np.reshape(self.ddthdes,(12,))
        

            abar = self.ddthdes + self.Kp*the+self.Kd*dthe


            


            N1,N2,fric1x,fric1y,fric2x,fric2y, fric1x_dir, fric1y_dir, fric2x_dir, fric2y_dir, ankle_torque_1, ankle_torque_2, CP, zmp = self.getNormalForce_modi()   

            
           # robot.drawPbline(np.array([zmp[0],zmp[1],0]), self.t_step2)
            
            n= np.linspace(1, 25, 100)
            
            if len(CP)>2:
             
                hull = ConvexHull(CP)

                area =  hull.volume

              # print('area', area)
             
                #print('hull', hull)
                        
                for zz  in range(len(n)):

                    if abs(t-n[zz])<0.1:
                       vv = 00
                       #self.plott(np.array(CP),hull,zmp, t)


           
            
            self.ZMP = zmp

            fric1 = fric1y*fric1y_dir+fric1x*fric1x_dir
            fric2 = fric2y*fric2y_dir+fric2x*fric2x_dir

          

            fric = fric1+fric2 #lateral reaction force from ground to feet

            N =N1+N2
            
            F =np.array([0,0,N])+fric

            self.Reaction_force = np.reshape(F,(3,))

            fric_norm = np.linalg.norm(fric)

                      
            x_in_prev = xin

           
            
            
            X = opt.runOptimization(self.h,self.thdvel, MVrhs)
            
            #print('X',X) 

            xin = X

            
            X1 = np.array([X[0],X[1],0]) 

            
            
            Mvrhs = np.reshape(MVrhs,(3,))
          
            xxx=np.real(np.linalg.pinv(self.h)@X1)

            Mvrhsxxx = 1*np.real(np.linalg.pinv(self.h)@Mvrhs)

            xxx = np.reshape(xxx,(12,))

            if t<self.t_impact:

              self.dthdes_new = 1*(np.array(np.real(self.dthdes))-1*xxx+ 0*Mvrhsxxx)

             

            elif t<self.t_impact+10:

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



            Cx = [1,0,0]
            Cy = [0,1,0]
            xtild = [0.15,0,0]
            ytild = [0,0.15,0]

            Cx= np.reshape(Cx,(1,3))
            Cy= np.reshape(Cy,(1,3))

            xtild= np.reshape(xtild,(3,1))
            ytild= np.reshape(ytild,(3,1))
            
            
            self.F_est = self.macm_calc-F-self.M*np.array([robot.gravityacc])

            if abs(self.Reaction_force[0])> abs(robot.mu*N/np.sqrt(1)):

               a1i =  str('friction limit exceeded_1')

               frx = Cx@(self.h@ self.thddacc+ self.h_dot@dth_+Marhs-self.F_app)

               #print('fricx',frx)

               if frx>0:

                self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs+robot.mu*N+self.F_app-xtild)

               else:

                self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs-robot.mu*N+self.F_app+xtild)    

               #self.dthdes_new = np.real(self.dthdes_new)+xxx

               #print('friction limit exceeded_1')
                       
            else:

                a1i =  str('friction ok_1')

               #print('friction ok_1') 

            
            if abs(self.Reaction_force[1])> abs(robot.mu*N/np.sqrt(1)):

                a2i =  str('friction limit exceeded_2')

                fry = Cy@(self.h@ self.thddacc+ self.h_dot@dth_+Marhs-self.F_app)

                #print('fricy', fry )

                if fry>0:

                 self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs+robot.mu*N+self.F_app-ytild)

                else:

                 self.dthdes_new = np.linalg.pinv(self.h_dot)@(-self.h@ self.thddacc-Marhs-robot.mu*N+self.F_app+ytild)  

                #self.dthdes_new = np.real(self.dthdes_new)+xxx

               #print('friction limit exceeded_2')
                       
            else:

                a2i =  str('friction ok_2')

             #  print('friction ok_2') 
            
            self.fric_info =  np.array([t,a1i,a2i])



            p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes,
                                    targetVelocities=self.dthdes_new)
           

            """
            p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL,targetPositions = self.thdes,
                                    targetVelocities=self.dthdes, forces=np.zeros(len(robot.rev))) 
                                    # positionGains = np.array(len(self.dthdes)*[20]) ,
                                    # velocityGains = np.array(len(self.dthdes)*[1.65]))
            """

            #self.thdes = [0]*len(robot.rev)
           
            
          #  p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL, forces=np.zeros(len(robot.rev)))
           
            #p.setJointMotorControlArray(robot.robot_id,robot.rev,p.POSITION_CONTROL)


            #tau = p.calculateInverseDynamics(robot.robot_id, list(self.thpos), list(self.thdvel), list(abar), flags =1)
            tau = p.calculateInverseDynamics(robot.robot_id, list(self.thdes), list(self.dthdes), list(abar), flags =1)

            tau = np.reshape(tau,(19,))

            #self.jtorques = tau[7:15]

            jointstates = p.getJointStates(robot.robot_id,robot.rev)
            
            self.jtorques = robot.applied_torques



           # jointstates = p.getJointStates(robot.robot_id,robot.rev)
            
            #self.jtorques = robot.applied_torques


            #if t<self.t_impact+0.3:

            # print("self.thdes",self.thdes)
            # print("self.dthdes_new",self.dthdes_new)
            # print("self.ddthdes",self.ddthdes)
            
          #  print("tau", tau)

          #  print("self.jtorques", self.jtorques)

            jtorques_filt_prev = self.jtorques_filt

            wc = 1 #cut-off frequency, can not be greater than 2/self.t_step2 
            ws = 1/self.t_step2 

            wr = wc/ws

            sf = 1 /(1+wr)
            

            for i in range(12):
                self.jtorques_filt[i] = (1-sf)*jtorques_filt_prev[i] + sf*self.jtorques[i]
               #self.jtorques_filt[1] = (1-sf)*jtorques_filt_prev[1] + sf*self.jtorques[1]


            
            if t<5:
                p.setJointMotorControlArray(robot.robot_id, robot.rev,
                                        controlMode = p.TORQUE_CONTROL, 
                                        forces = self.jtorques_filt)
            
            
            ##ankle torque control
            mgc1 = (self.c_cm[1]-robot.urdf_pos[17][1])*M*9.81
            mgc2 = (self.c_cm[1]-robot.urdf_pos[40][1])*M*9.81

            if t<7:

                p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[4],
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  0*self.jtorques[4] - ankle_torque_2+0.5*mgc1)

                p.setJointMotorControl2(robot.robot_id, jointIndex = robot.rev[10],
                                        controlMode = p.TORQUE_CONTROL, 
                                        force =  0*self.jtorques[10] - ankle_torque_1+0.5*mgc2) 
            
            """
            phimax = 0.1
            T1 =2
            T2 = 10
            
            if area<1e-2:

                if t<T1:

                    phi = -2*phimax*t*t*t*(1/(T1*T1*T1)) + 3*phimax*t*t*(1/(T1*T1))

                else:

                    phi = 2*phimax*t*t*t*(1/(T2*T2*T2)) - 3*phimax*t*t*(1/(T2*T2)) + phimax

                    phi = 0*phi

                    p.setJointMotorControl2(robot.robot_id,robot.rev[0],p.POSITION_CONTROL,targetPosition = phi)

                    p.setJointMotorControl2(robot.robot_id,robot.rev[6],p.POSITION_CONTROL,targetPosition = phi)
            """
            phimax = 0.1
            T1 =2
            T2 = 50
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
                
        

        
            

            dth=np.array([robot.qdot])
            dthT = dth.T

           


            self.actualmomentumfromh = self.h@dthT+Mvrhst

            

            self.actualmomentumfromh = np.reshape(self.actualmomentumfromh ,(3,))

            
            
            
            self.write_data()



        # time.sleep(1/100)
    
    def checkH(self):

        thd=p.getJointStates(robot.robot_id,robot.rev)
      

      #  thdfixed = p.getJointStates(robot.robot_id,[0,2,4,8,10,12])
        
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
        
       
        self.dthdes  = np.real(self.dthdes)

        self.dthdes = np.reshape(self.dthdes,(12,))

        self.thdes = np.reshape(self.thdes,(12,))

        self.jointvelerr = self.dthdes.T-self.thdvel

        self.jointposerr = self.thdes.T-self.thpos

       

        robot.getCOMPos()
       
        self.M = np.sum(robot.setLinkM())
       
        self.v_torso= np.array(p.getBaseVelocity(robot.robot_id)[0])


        """
        self.v_base= (robot.m_array[0]*robot.link_vel[0]+robot.m_array[8]*robot.link_vel[8]+robot.m_array[16]*self.v_torso)/(robot.m_array[0]+robot.m_array[8]+robot.m_array[16])
        
       
        dth=np.array(robot.qdot)
       

        qnorm=np.linalg.norm(robot.urdf_orien[0])
        
        dthall =  np.array(robot.qdotall)
       
        

       

        #vbase_r= robot.base_vel+self.skewM(robot.base_angvel)@(robot.urdf_pos[0]--robot.com_base_pos)
        vbase_r= robot.link_vel[0]+self.skewM(robot.link_omega[0])@(robot.urdf_pos[1]-robot.com_pos[0])
        vbase_l= robot.link_vel[8]+self.skewM(robot.link_omega[8])@(robot.urdf_pos[9]-robot.com_pos[8])
        Mr = np.sum(robot.setLinkM()[1:8])
        Ml = np.sum(robot.setLinkM()[9:16])
        M_base= robot.setLinkM()[16]+robot.setLinkM()[0]+robot.setLinkM()[8]
        
        modi_link_mass = robot.setLinkM()
        modi_link_vel = np.append(robot.link_vel,[self.v_torso],axis=0)
        vcmdata=robot.getrobotCOMvelocity(modi_link_mass, modi_link_vel)

                
        #link_vel1= robot.base_vel+self.skewM(robot.base_angvel)@(robot.com_pos[0]-robot.com_base_pos)
        link_vel1= robot.base_vel+np.matmul(self.skewM(robot.base_angvel),(robot.com_pos[0]-robot.com_base_pos))

        #link_vel= robot.link_vel[0]+self.skewM(robot.link_omega[0])@(robot.urdf_pos[1]-robot.com_pos[0])+self.skewM(robot.link_omega[1])@(-robot.urdf_pos[1]+robot.com_pos[1])
        link_vel= robot.link_vel[2]+self.skewM(robot.link_omega[2])@(robot.urdf_pos[3]-robot.com_pos[2])+self.skewM(robot.link_omega[3])@(-robot.urdf_pos[3]+robot.com_pos[3])
        
        """
        
        link_vell = np.append(robot.link_vel,[self.v_torso],axis=0)

        link_pos = np.append(robot.com_pos,[robot.com_base_pos],axis=0)
      
        
        mv=np.zeros((50,3))

        mc = np.zeros((50,3))
        
        #print("len(robot.m_array)", len(robot.m_array))

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
       
       

      
        self.v_cm = m_vcm/self.M

        self.c_cm = m_ccm/self.M

       # print("self.c_cm", self.c_cm)
        
     
          
          

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
            #writer.writerow(self.omega_err) 
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


if __name__ == "__main__":
    robot = Biped()
   # traj = GenTraj()
    s1 = BipedSim()
   # p1 = np.array([-0.17,0.17,-0.05])
   ## p2 = np.array([ 0.07,-0.07,0.1])
   # p3 = np.array([-0.17,0.17,-0.05])
    #traj.setTrajPt(**{"p1":p1,"p2": p2,"p3": p3})
    s1.runSim()
    

