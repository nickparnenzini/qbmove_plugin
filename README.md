<b>Fixed and tested with Gazebo7.</b> 

This is an open-source qbmove plugin in Gazebo. For further information,
please visit: http://www.qbrobotics.com/

If you want to use this software in Gazebo:

1) Download a Gazebo version from http://gazebosim.org/

2) Download this folder and copy it in ~/.gazebo/models

3) open a new terminal and digit the following commands: 
 ```
	  $ cd ~/.gazebo/models/qbmove_plugin_v1.0/
          $ mkdir build
          $ cd build
          $ cmake ../
          $ make
 ```
4) Now you should have an executable plugin for this model. 

5) you have to register this plugin by digiting the following command: 
          `$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/.gazebo/models/qbmove_plugin_v1.0/build`
           
6) before starting Gazebo, you must be in the folder: ~/.gazebo/models/qbmove_plugin_v1.0
   to let Gazebo load meshes of this model. 
   
7) digit: $ gazebo -u
   and a new Gazebo execution will start. Select 'qbmove' model from GUI. Then you can start simulation,
   but nothing will happen until you set values for your qbmove model. This will be done in the next step.
   
8) Open a new terminal, and digit the following commands: 
```      $ cd  ~/.gazebo/models/qbmove_plugin_v1.0/build
      $ ./talker position_value stiffness_value```
   where position_value and stiffness_value are values that you can choose for your qbmove model. 
   These values are in degrees. For instance, you could choose 45 and 0 for position and stiffness, respectively.
   You will see your qbmove model moving.
