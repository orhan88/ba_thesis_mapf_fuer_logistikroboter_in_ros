# README

This project contains the simulation developed during the bachelor's thesis *"Multi-Agent Path Planning für Logistikroboter in ROS" (by Orchan Heupel)*

The repository contains the following five ROS2-packages:
* **mapf_simulation** (Graphical user interface and the visualization of the simulation)
* **mapf_controller** (Interface between the simulation, pathplanner and evaluator)
* **mapf_evaluator** (evaluates and compares the different algorithms by plotting certain data)
* and two path planners 
    * **mapf_pathplanner_pbs** (Priority Based Search)
    * **mapf_pathplanner_cbs** (Conflict Based Search)

## Documentation

The documentation of the whole simulation you can find in the following paper (in german):   
*./docu/Bachelorarbeit_Multi_Agent_Paht_Planning_fuer_Logistikroboter_in_ROS.pdf*

The documentation of the packages **mapf_simulation**, **mapf_controller** and **mapf_evaluator** you can find in the folder *./docu*. The documentation was created with pdoc. 

The documentation of the Priority Based Search you can find here:  
https://github.com/Jiaoyang-Li/PBS  
or in the following paper http://idm-lab.org/bib/abstracts/papers/aaai19b.pdf  

The documentation of the Conflict Based Search you can find here:  
https://github.com/Jiaoyang-Li/CBSH2-RTC  
or in the following paper http://idm-lab.org/bib/abstracts/papers/ijcai19a.pdf

## Requierements

### mapf_simulation / mapf_controller / mapf_evaluator

* Python interpreter version 3.x
* PyQt5, version 5.6 or later
* python3-matplotlib 
* ROS2 version Foxy Fitzroy


### mapf_pathplanner_pbs / mapf_pathplanner_cbs

* ROS2 version Foxy Fitzroy
* libboost-all-dev

    The code requires the external library [boost](https://www.boost.org/). 
    If you are using Ubuntu, you can install it simply by:
    ```shell script
    sudo apt install libboost-all-dev
    ``` 

## Installation 

1. **Install the requiered packages**
    * Python interpreter version 3.x
    * PyQt5
    * python3-matplotlib
    * ROS2
    * libboost-all-dev
    
    &nbsp;  

2. **Copy the following packages from** ./src **into the source folder of your ROS2-workspace:** 
    * mapf_interfaces
    * mapf_simulation
    * mapf_controller
    * mapf_evaluator
    * mapf_pathplanner_pbs 
    * mapf_pathplanner_cbs 
    
    &nbsp;  


3. **Compile and source the packages from the root of your workspace**

    ```colcon build --packages-select mapf_interfaces ```  
    
    ```source install/setup.bash ```
    
    
    ```colcon build --packages-select mapf_simulation```
    
    ```colcon build --packages-select mapf_controller ```  
    
    ```colcon build --packages-select mapf_evaluator ```  
    
    ```colcon build --packages-select mapf_pathplanner_pbs ```  
    
    ```colcon build --packages-select mapf_pathplanner_cbs ```   
    
    ```source install/setup.bash ```  
    
    &nbsp;  

## Usage 

1.  **Start the nodes from the terminal by running the following commands:**

    ```ros2 run mapf_simulation start_gui ```  

    ```ros2 run mapf_controller controller ```
    
    ```ros2 run mapf_pathplanner_pbs pathplanner  ```
    
    ```[optional] ros2 run mapf_evaluator evaluator ```
    
    &nbsp;  
    
2.  **Load the .map file and the .scen files (GUI)** (Examples you can find in the folder ./data) 
    &nbsp;   

3.  **Start the simulation by pressing the button "Start Sim" (GUI)**



