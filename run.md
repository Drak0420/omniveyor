Follow these steps to run omniveyor (Comments are in ()'s and are not required.)

1. Power Up AGV
	1a. Plug battery into AGV
		Plug in PSU and ensure charge is okay 
			42.2V is max but should not be below 40V as recommended minimum is around 35V
	1b. Press green ON button on side of AGV
	1c. Depress E-stop and press the RESET button below the E-stop
	1d. Position AGV centered on center of the concrete corner with the blue tape on floor labeled O
		Face AGV (front is the side w/ two cameras) towards the CNCs
	
	
2. Start AGV Main Node
	2a. Open a new terminal window and type in (om is shortcut set in ~/.ssh/config)
		ssh om
			Note: If this fails, wait a few seconds for the AGV computer to power up
	2b. When ssh'ed in, type in (These are aliases set in ~/.bashrc)
		om && om_run 
		
		
3. Start other nodes
	3a. Joystick tele-operation
		3a1. Take out wireless controller and connect it to the computer
			No Bluetooth so can not connect it :(
			Will be true later: Black PS4 controller is already linked, just press the PS4 logo
		3a2. Open a new terminal window and type in 
			om
			roslaunch pcv_base joystick_teleop.launch
		3a3. Press PS logo button for help menu (shows in terminal where you launched node)
		
	3b. Keyboard tele-operation
		3b1. Open a new terminal window and type in 
			om
			roslaunch pcv_base keyboard_teleop.launch
			
	3c. Run trajectory
		(can be done when ssh'ed in [recommended] or on local machine)
		3c1. Open a new terminal window and type in (om is shortcut set in ~/.ssh/config)
			ssh om
				Note: If this fails, wait a few seconds for the AGV computer to power up
		3c2. When ssh'ed in, type in
			om
			roslaunch omniveyor_mobility run_trajectory.launch
		3c3. Running goals
			Utilize joystick teleop instructions (3a)
		
			
	3d. Create trajectory 
		(can be done when ssh'ed in [recommended] or on local machine)
		3d1. Open a new terminal window and type in (om is shortcut set in ~/.ssh/config)
			ssh om
				Note: If this fails, wait a few seconds for the AGV computer to power up
		3d2. When ssh'ed in, type in
			om
			roslaunch omniveyor_mobility create_trajectory.launch
				Note: Wipes old trajectory - rename file in {machine-specific folder}/src/omniveyor/omniveyor_mobility/resources if you want to keep it
		3d3. Create goals
			3e3a. If using controller, goal buttons per help menu correspond to their respective goals
			3e3b. Otherwise, input goals on the rviz GUI (per 3f)
				Click "2D Nav Goal" in the upper toolbar, then click and drag for desired position and direction facing.
				Note: this will generate in order from 0->1->2, etc. and can not go back in terms of goal #s
			
	3e. Build map 
		(should be done if major change in layout happens because AGV relies on this map to know where it is)
		3e1. Open a new terminal window and type in (om is shortcut set in ~/.ssh/config)
			ssh om
				Note: If this fails, wait a few seconds for the AGV computer to power up
		3e2. When ssh'ed in, type in
			om
			roslaunch omniveyor_mobility build_map.launch
				NOTE: Wipes old map - move old maps if desired (files named IMI_001 on the AGVs in omniveyor_mobility/resources/maps)
				
	3f. Visualization (rviz)
		3c1. Open a new terminal window and type in
			om
			rviz -d /home/cartman/Dev/ros_ws/src/omniveyor/omniveyor_mobility/resources/movebase.rviz
			
	3g. Alternative visualization panel (rqt)
		3d1. Open a new terminal window and type in 
			om
			rqt --perspective-file /home/cartman/Dev/ros_ws/src/omniveyor/omniveyor_mobility/resources/omniveyor.perspective
			
			
4. Shutdown
	4a. Press Off button on side of AGV
	4b. Remove battery 
		should be stored around 38V if stored long-term to prevent battery damage
