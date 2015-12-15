Razer Hydra (Sixence) Driver to Linux (C++) with Python Interface using YARP communication.
by Dannier Castro.

Instructions:

1) Download the Razer Hydra SDK Linux distribution from Sixence Official Web Page: http://sixense.com/linuxsdkdownload

2) Follow the instructions from "README.txt" file in SDK root directory to install the dependences so the program works on Linux OS.

3) Install YARP following this instructions https://wiki.arcoslab.eie.ucr.ac.cr/doku.php/installing_yarp_in_debian

4) Important Steps:
  4.a) Copy razer_linux_driver.cpp in directory ~/src/sixence_simple3d/progs/demos/sixense_simple3d
  4.b) Copy the 4 files in ~/src/sixence_simple3d/Eclipse/sixence_simple_3d_Release_x64
  
5) $ cd ~/src/sixence_simple3d/Eclipse/sixence_simple_3d_Release_x64
   $ make clean all
   
It may create a binary file called "razer_linux_driver" and enjoy to GAMES PROJECTS.
