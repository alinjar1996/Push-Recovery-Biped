# Push-Recovery-Biped

This repository contains Python codes for an algorithm developed for push recovery of a biped from different stance scenarios. The stance scenarios are Double contact at the same level ground, Double contact at Different level ground, and initial Single contact.

Please follow the following steps to run the necessary codes.

1. git clone https://github.com/alinjar1996/Push-Recovery-Biped.git -b main

2. For the stance scenario of double contact with the same plane and Forward push (along with lateral), run "Pushrecoverysameplanedoublesupportorg_201223_CTC.py".  For the same stance scenario, but for the backward push (along with lateral), run "Pushrecoverysameplanedoublesupportorg_041023_opp_dir_CTC.py". The exact value of forces and their durations are mentioned inside the respective codes.

3.  For the stance scenario of double contact offset plane and Forward push (along with lateral), run "Pushrecoverydiffplaneorg_real_time_201223_CTC.py". For the same stance scenario, but for the backward push (along with lateral), run "Pushrecoverydiffplaneorg_real_time_04102023_opp_dir_CTC.py" . The exact value of forces and their durations are mentioned inside the respective codes.

4.  For the stance scenario of initial single support and Forward push (along with lateral), run "Pushrecoverysinglesupportorg_real_time_201223_CTC.py". For the same stance scenario, but for the backward push (along with lateral), run "Pushrecoverysinglesupportorg_real_time_04102023_opp_dir_CTC.py". The exact value of forces and their durations are mentioned inside the respective codes.

5. The class dependencies can be understood from the classes imported at the beginning of each .py file.

6. Run "plotter_12DOF.py" to see the plots.

7. For obtaining ZMP, uncomment the lines with self.plott(np.array(CP),hull,zmp, t) or self.plott(np.array(VHP),hull,zmp, t) (for double support offset).

8. It is worth mentioning that all the necessary comments to understand the meaning of each line are given in "Pushrecoverysameplanedoublesupportorg_201223_CTC.py". For other codes, the lines are the same; hence, the same comments denote the lines' mathematical significance are not repeated.
   
9. Please note that the URDF files are updated so that the mass and inertia properties closely match the hardware developed at IIT Delhi by the author and his team. 

10. Please also note that there are still some redundant files in the repository. It is time-consuming to detect and remove all of them. However, in future, they will be selected and removed.
