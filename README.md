# Push-Recovery-Biped

This repository contains Python codes for an algorithm developed for push recovery of a biped from different stance scenarios. The stance scenarios are Double contact at the same level ground, Double contact at Different level ground, and initial Single contact.

Please follow the following steps to run the necessary codes.

1. git clone https://github.com/alinjar1996/Push-Recovery-Biped.git -b Push_Recovery_6_DoF

2. For the stance scenario of double contact with the same plane and Forward push, run "PushRecovery_6DOF_same_Plane_Double_Support_real_time_04102023_CTC.py".  For the same stance scenario, but for the backward push, run "PushRecovery_6DOF_same_Plane_Double_Support_real_time_04102023_opp_dir_CTC.py". The exact value of forces and their durations are mentioned inside the respective codes.

3.  For the stance scenario of double contact offset plane and Forward push, run "PushRecovery_6DOF_diffplaneorg_real_time_04102023_CTC.py". For the same stance scenario but for the backward push, run "PushRecovery_6DOF_diffplaneorg_real_time_04102023_opp_dir_CTC". The exact value of forces and their durations are mentioned inside the respective codes.

4.  For the stance scenario of initial single support and Forward push (along with lateral), run "PushRecoverysinglesupportorg_6DOF_real_time_04102023_CTC.py". For the same stance scenario, but for the backward push (along with lateral), run "PushRecoverysinglesupportorg_6DOF_real_time_opp_dir_04102023_CTC.py". The exact value of forces and their durations are mentioned inside the respective codes.

5. The class dependencies can be understood from the classes imported at the beginning of each .py file.

6. Run "plotter_6DOF.py" to see the plots.

7. For obtaining ZMP, uncomment the lines with self.plott(np.array(CP),hull,zmp, t) or self.plott(np.array(VHP),hull,zmp, t) (for double support offset).
  
8. Please note that the URDF files are updated so that the mass and inertia properties closely match the hardware developed at IIT Delhi by the author and his team. 





