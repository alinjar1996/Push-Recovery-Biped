import csv
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
#from hvelcontrolmodixyoptimized_notippingmodi import BipedSim
#from hvelcontrolmodixyoptimizeddifflvl_notipping3 import BipedSim
#from walking_code2 import BipedSim
#from hvelcontrolmodixyoptimizeddynadifflvl_notipping import BipedSim
#from hvelcontrolmodixyoptimizeddifflvl_notipping3 import BipedSim
#from hvelcontrolmodixyoptimizeddynasinglesupport_notipping2 import BipedSim
from filterfunc import Filter
#from hvelcontrolmodifrictionmodi2 import BipedSim
#from hvelsinglesupportorg import BipedSim
#from hvelcontrolsameplanedoublesupportorg3_CTC import BipedSim
#from hvelcontrolsameplanedoublesupportorg02082023_new import BipedSim

#folder = "/home/udayan/Downloads/pybullet-kinematics-dynamics-control-master(1)/pybullet-kinematics-dynamics-control-master/examples"
folder = "/home/udayan/Desktop/Alinjar/Biped Pybullet/bipedal_03082023_12DOF_and_6_DOF/bipedal_12_01/"
#folder = "bipedal/"
#file = folder+"cm.csv"
#file = folder+"jvel.csv"
file = folder+"jtorques.csv"
#file = folder+"omegaerr.csv"
#file = folder+"linvelerr.csv"
#file = folder + "jointvelerror.csv"
#file = folder + "jointposerror.csv"
#file = folder + "jointposition.csv"
#file = folder + "jointvelocity.csv"
#file = folder+"vcm.csv"
#file = folder + "X.csv"
#file = folder + "ReactionForce.csv"
#file = folder + "Force_check.csv"
#file =  folder +"Margincheck.csv"
#file =  folder +"ZMP.csv"
#file = folder+"jointposerror.csv"

rms = False

# m = 000
fields = []
plotting_data = []
time_data = []

with open(file, 'r') as csvfile:
    csvreader = csv.reader(csvfile)

    fields = next(csvreader)

    for row in csvreader:
        plotting_data.append(np.array(row))


plotting_data = np.array(np.float32(plotting_data)).T

n = 0

#n = np.size(plotting_data[0])

m = np.size(plotting_data[0])



#m1 = len(plotting_data)

#m =1000

#print('m1', m1)

#timearray = np.linspace(0,np.ceil(np.size(plotting_data[0])/240),m)

#print(getattr(BipedSim, "t_stop"))

#Tstop = BipedSim().timeee()

with open(folder+"Time.csv", 'r') as csvfile:

    csvreader = csv.reader(csvfile)

    fields = next(csvreader)

    for row in csvreader:

        time_data.append(np.array(row))


time_data = np.array(np.float32(time_data)).T

timearray = time_data[0]

#print("timearray", time_data)

"""

Tstop = BipedSim().time

print('Tstop', Tstop)

timearray = np.linspace(0,Tstop,m)

"""
#print('timearray', timearray)

SMALL_SIZE = 8
MEDIUM_SIZE = 10
LARGE_SIZE =  12

#efont = {'fontname':'Euclid'}


if file == folder+'vcm.csv' and rms == False:
    
    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    #print("plotting_data", plotting_data.shape)

    plt.rc('font', size=LARGE_SIZE)

    print("m", m)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="X",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="Y",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="Z",linewidth=2)
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel("VCM (metre/sec)")
    plt.legend(title= 'COM velocity')
    plt.title("COM velocity with time")
    plt.show()

elif file == folder+'cm.csv' and rms == False:


    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="X",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="Y",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="Z",linewidth=2)
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel("CM")
    plt.legend(title= 'COM position')
    plt.title("COM position with time")
    plt.show()

elif file == folder+'h_err.csv':

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="x",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="y",linewidth=2,color='black')
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="z",linewidth=2)
    plt.xlabel("Time (sec)")
    plt.ylabel("Error")
    plt.legend(title= 'error for x,y,z')
    plt.title("Error with time")
    plt.show()

elif file == folder+'jtorques.csv':

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)
    
    plt.plot(timearray[n:m],plotting_data[0][n:m],label="1",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="2",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="3",linewidth=2)

    plt.plot(timearray[n:m],plotting_data[3][n:m],label="4",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[4][n:m],label="5",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[5][n:m],label="6",linewidth=2)
   
    
    plt.xlabel("Time (sec)")
    plt.ylabel("Torque (Newton- Metre)")
    plt.legend(title= 'Joint number')
    plt.title("Joint Torque with time")
    plt.show()

elif file == folder+'jvel.csv':

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 
    
    plt.rc('font', size=LARGE_SIZE)
    
    plt.plot(timearray[n:m],plotting_data[0][n:m],label="1",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="2",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="3",linewidth=2)

    plt.plot(timearray[n:m],plotting_data[3][n:m],label="4",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[4][n:m],label="5",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[5][n:m],label="6",linewidth=2)

    plt.xlabel("Time (sec)")
    plt.ylabel("Joint velocity (Radian/sec)")
    plt.legend(title= 'Joint number')
    plt.title("Joint velocity with time")
    plt.show()

elif file == folder+'jointvelerror.csv':

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)
    plt.plot(timearray[n:m],plotting_data[0][n:m],label="1",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="2",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="3",linewidth=2)

    plt.plot(timearray[n:m],plotting_data[3][n:m],label="4",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[4][n:m],label="5",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[5][n:m],label="6",linewidth=2)

    plt.xlabel("Time (sec)")
    plt.ylabel("Joint velocity error")
    plt.legend(title= 'Joint number')
    plt.title("Joint velocity error with time")
    plt.show()    


elif file == folder+'jointposerror.csv':

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="1",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="2",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="3",linewidth=2)

    plt.plot(timearray[n:m],plotting_data[3][n:m],label="4",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[4][n:m],label="5",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[5][n:m],label="6",linewidth=2)
    plt.xlabel("Time (sec)")
    plt.ylabel("Joint position error")
    plt.legend(title= 'Joint number')
    plt.title("Joint position error with time")
    plt.show()    

elif file == folder+'jointposition.csv':

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)
    plt.plot(timearray[n:m],plotting_data[0][n:m],label="1",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="2",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="3",linewidth=2)

    plt.plot(timearray[n:m],plotting_data[3][n:m],label="4",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[4][n:m],label="5",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[5][n:m],label="6",linewidth=2)
    plt.xlabel("Time (sec)")
    plt.ylabel("Joint Position (Rad)")
    plt.legend(title= 'Joint number')
    plt.title("Joint  Position with time")
    plt.show()     

elif file == folder+'jointvelocity.csv':

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)
    
    
    plt.plot(timearray[n:m],plotting_data[0][n:m],label="1",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="2",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="3",linewidth=2)

    plt.plot(timearray[n:m],plotting_data[3][n:m],label="4",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[4][n:m],label="5",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[5][n:m],label="6",linewidth=2)
    
    plt.xlabel("Time (sec)")
    plt.ylabel("Joint Velocity (Rad/Sec)")
    plt.legend(title= 'Joint number')
    plt.title("Joint  Velocity with time")
    plt.show()        


elif file == folder+'omegaerr.csv' and rms == False:

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="wX",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="wY",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="wZ",linewidth=2)
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel("omega")
    plt.legend(title= 'omega')
    plt.title("omega error with time")
    plt.show()

elif file == folder+'linvelerr.csv' and rms == False:
    
    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="vX",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="vY",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="vZ",linewidth=2)
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel("lin vel error")
    plt.legend(title= 'lin vel error')
    plt.title("lin vel error with time")
    plt.show() 

elif file == folder+'X.csv' and rms == False:

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="X1",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="X2",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="X3",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[3][n:m],label="X4",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[4][n:m],label="X5",linewidth=2)
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel("X")
    plt.legend(title= 'X')
    plt.title("X with time")
    plt.show() 

elif file == folder+'ReactionForce.csv' and rms == False:

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 
    
    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="RX",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="RY",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="RZ",linewidth=2)
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel("Normal Force")
    plt.legend(title= 'Reaction Force')
    plt.title("Reaction Force with time")
    plt.show()       



elif file == folder+'Force_check.csv' and rms == False:

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="RX",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="RY",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="RZ",linewidth=2)
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel(" Force")
    plt.legend(title= ' Force check')
    plt.title(" Force check with time")
    plt.show() 

elif file == folder+'ZMP.csv' and rms == False:

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    plt.plot(timearray[n:m],plotting_data[0][n:m],label="ZMP_X",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="ZMP_Y",linewidth=2)
#    plt.plot(timearray[n:m],plotting_data[2][n:m],label="ZMP_Z",linewidth=2)
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel("Zero MomentPoint (ZMP)")
    plt.legend(title= ' ZMP')
    plt.title(" ZMP with time")
    plt.show()     

elif file == folder+'Margincheck.csv' and rms == False:

    plotting_data_  = Filter.filter(plotting_data,timearray,n,m)

    plotting_data = plotting_data_.T 

    plt.rc('font', size=LARGE_SIZE)

    """
    plt.plot(timearray[n:m],plotting_data[0][n:m],label="1",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[3][n:m],label="1l",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[6][n:m],label="1u",linewidth=2)
    """
    plt.plot(timearray[n:m],plotting_data[1][n:m],label="2",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[4][n:m],label="2l",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[7][n:m],label="2u",linewidth=2)
    """
    plt.plot(timearray[n:m],plotting_data[2][n:m],label="3",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[5][n:m],label="3l",linewidth=2)
    plt.plot(timearray[n:m],plotting_data[8][n:m],label="3u",linewidth=2)
    """
    
    
    
    
    
    
    
    # plt.ylim((-0.02,0.02))
    # plt.ylim((-.25,.25))
    plt.xlabel("Time (sec)")
    plt.ylabel(" Margincheck")
    plt.legend(title= ' Margincheck')
    plt.title(" Margincheck with time")
    plt.show()            
