from re import X
import casadi as ca
import numpy as np

def runOptimization(h,thd,Mvrhs):

  Mvrhs = np.reshape(Mvrhs,(3,)) 
    
  h1 =  np.eye(2,2)
  H = 2*ca.DM(h1)
#     # print(H)
#     # print(M*a_torso)

  #constraint = hdot@V
  Y=np.matrix([[1, 0, 0], [0, 1, 0]])
  

#     # print(constraint)
  #gr = np.matrix([[0],[0],[-10]])
  #A = ca.DM(Y@hdot@V)
  A= ca.DM(np.zeros((2,2)))
  
  
  #hthd = h@thd
  hthd = h@thd
  Yhthd = Y@hthd  #To extract x and y components
  g = -2*ca.DM(Yhthd.T)

  #print('hthd', hthd.shape)
  #print('Mvrhs',Mvrhs.shape)
  #g=ca.DM(f)
  
  #uba = ca.DM(np.array([np.inf,np.inf,0])+ h@qddot - M*a_torso- M*gr)
  


  
  #print("lba", lba)

  #lba = -M*10*np.array([mu/np.sqrt(2), mu/np.sqrt(2)]) + Y@( h@qddot - M*a_torso - M*gr)
  #lba = -ca.DM((N1+N2)*np.array([mu/np.sqrt(2), mu/np.sqrt(2), 0]) +  h@qddot - M*a_torso- M*gr)
  #lbaa= -ca.DM((N1+N2)*np.array([mu/np.sqrt(2), mu/np.sqrt(2)]) + Y@( h@qddot - M*a_torso- M*gr))
  #lba= ca.DM(np.array([lbaa, 0]))
  #uba = ca.DM((N1+N2)*np.array([mu/np.sqrt(1), mu/np.sqrt(1), np.inf]) - np.array([ h@qddot + M*a_torso+M*gr]))
  #uba = ca.DM(np.array([100,100,0]))
  #uba = -ca.DM((N1+N2)*np.array([mu/np.sqrt(1), mu/np.sqrt(1), -np.inf]) - np.array([ h@qddot + M*a_torso+M*gr]))

  

 # lbx = ca.DM(np.matrix([[-0.1],[-0.1]]))
 # ubx = ca.DM(np.matrix([[0.1],[0.1]]))

 
  lbx = ca.DM(np.array([-0.2,-0.2]))
  ubx = ca.DM(np.array([0.2,0.2]))


  #lbx = ca.DM(np.array([0.1,0.1,0.1,0.1,0.1]))
  #lbx = ca.DM(np.array([0.05,0.05,0.05,0.05,0.05]))

  #ubx = ca.DM(np.array([6000,6000,6000, 6000, 6000]))

  qp = {
       'h': H.sparsity(),
       'a': A.sparsity(),
       }

  #opts = {'print_out':0, 'print_time':0}

  S = ca.conic('S', 'qpoases', qp)

  solu = S(h=H,a=A,g = g ,lbx= lbx, ubx = ubx , x0 = np.matrix([[0],[0]]))

  X = solu['x']

  return X
