# Path Planning with ROS2 and Python

## The project is consist of 4 parts:

- Detecting the robot in the given Gazebo environment
  
- Mapping --> Converting Gazebo environment into a topological map

- Path Planning --> First finding any available path inside the environment and then apply Dijisktra and A* algorithms to find the shortest path 

- Motion Planning --> Implementing obstacle detection and moving the robot towards the exit.

Everything is implemented inside a final script which is called `final_script.py` and once this is executed, GUI will show up along with a live Gazebo image.

## Running the project

In terminal 1:

    cd path_planning
    colcon build
    source install/setup.bash
    ros2 launch path_plan env_w_camera.launch.py
    
`env_w_camera.launch.py` has the main Gazebo environment that is created by `lab.py`.  Can run ` ros2 launch path_plan env_2_camera.launch.py` to see a simpler Gazebo world for the simulation; here the environment has diagonal pathways which shows that path planning algorithms also work for complex paths. 

In terminal 2:

   
    source install/setup.bash
    ros2 run path_plan bot_move
    
This shows a GUI and it is possible to see the outcomes of 4 parts by navigating the slides. 
