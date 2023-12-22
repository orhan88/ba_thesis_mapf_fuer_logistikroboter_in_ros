#! /usr/bin/env python3
import sys
import time
from datetime import datetime
from typing import List

from mapf_interfaces.msg import RobotAction, RobotInfo, MapInfo, MAPFInstance, MAPFSolution, StatisticInfo

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter


# Different possible states of Controller
MODE_INIT = 0
'''= state of the module CONTROLLER at the begin of every cycle till a "mapf_interfaces/MapInfo.msg" has been received.'''
MODE_WAIT_SIM = 1
'''= state of CONTROLLER after a "mapf_interfaces/MapInfo.msg" (for the current cycle) has been received. 
     
     CONTROLLER checks whether the number of the robots in the 
     simulation has changed and (if necessary) creates additional subscribers and publishers for new robots. 
     After CONTROLLER is waiting until it has received a message of type 
     "mapf_interfaces/RobotInfo.msg" from every robot in the simulation.'''
MODE_PUB_INSTANCE = 2
'''= state of CONTROLLER after it has received all necessary information from the simulation for the current cycle. 
     
     CONTROLLER fuses all received information to create an Instance ("mapf_interfaces/MAPFInstance.msg") which can be solved by a pathplanner'''
MODE_WAIT_SOLVER = 3
'''= state of CONTROLLER after it has published the generated Instance and waits for the solution from the pathplanner
'''
MODE_PUB_SIM = 4
'''= state of CONTROLLER after it has received a solution from the pathplanner. 
     
     CONTROLLER generates the next action for every robot so they can follow the defined paths by the pathplanner.
     In addition CONTROLLER generates a "mapf_interfaces/StatisticInfo.msg" for every received solution for the modul EVALUATOR
'''
        

class Controller(Node):
    '''
    This class represents the following ros2 node: **mapf_controller_node**.  

    The node mapf_controller_node subscribes to the folllowing topics:  
    **/mapf_map_info** *[mapf_interfaces/MapInfo.msg]*, callback_mapinfo(self, msg)  
    **/mapf_instance_solution** *[mapf_interfaces/MAPFSolution.msg]*, callback_solution(self, msg)

    The node mapf_controller_node publishes to the folllowing topics:  
    **/mapf_instance_info** *[mapf_interfaces/MAPFInstance.msg]*  
    **/mapf_statistics_info** *[mapf_interfaces/StatisticInfo.msg]*  

    For every robot in the simulation an additional subscriber and publisher will be created
    (the ID in the topic name will be replaced by the ID of the robot):  
    **/mapf_robot_info_ID** *[mapf_interfaces/RobotInfo.msg]*, callback_robotinfo(self, msg)  
    **/mapf_robot_action_ID***[mapf_interfaces/RobotAction.msg]*
    '''
    def __init__(self):
        '''
        Creates an instance of the controller
        '''
        super().__init__('mapf_controller_node')

        self.ctrl_state = MODE_INIT 
        '''**int** - defines the current state of the controller. Possible values are: 
        >MODE_INIT = 0  
        >MODE_WAIT_SIM = 1  
        >MODE_PUB_INSTANCE = 2  
        >MODE_WAIT_SOLVER = 3   
        >MODE_PUB_SIM = 4
        '''
        
        self.current_id_cycle = -1
        '''**int** - defines the current cylce. Each cycle has a unique id which is incrementing with every new cycle. '''
        self.number_robots = 0
        '''**int** - defines the number of robots for which the connection between the CONTROLLER and SIMULATION is created'''
        
        #subscribe to "mapf_map_info" [MapInfo.msg]
        self.subscriber_map = self.create_subscription(MapInfo, 'mapf_map_info' , self.callback_mapinfo, 1)
        '''**rclpy.subscription.Subscription** - ros2 subscriber to the topic */mapf_map_info*'''
        
        #list of subscribers to "mapf_robot_info_ID" [RobotInfo.msg]
        self.subscribers_robot = []
        '''list of objects of type **rclpy.subscription.Subscription** - subscribers to the topics: /mapf_robot_info_"ID"'''
        
        #list of publisher to "mapf_robot_action_id" [RobotAction.msg]
        self.publishers_robot = []
        '''list of objects of type **rclpy.publisher.Publisher** - publishers to the topics:   
        /mapf_robot_action_"ID"'''
        
        #publish Instance to "mapf_instance_info" [MAPFInstance.msg]
        self.publisher_instance = self.create_publisher(MAPFInstance, 'mapf_instance_info', 1)
        '''**rclpy.publisher.Publisher** - ros2 publisher to the topic */mapf_instance_info*'''
        
        #subscribe to "mapf_instance_solution" [MAPFSolution.msg]
        self.subscriber_instance = self.create_subscription(MAPFSolution, 'mapf_instance_solution' , self.callback_solution, 1)
        '''**rclpy.subscription.Subscription** - ros2 subscriber to the topic */mapf_instance_solution*'''
        
        #publish  to "mapf_statistics_info" [StatisticInfo.msg]
        self.publisher_statistics = self.create_publisher(StatisticInfo, 'mapf_statistics_info', 1)
        '''**rclpy.publisher.Publisher** - ros2 publisher to the topic */mapf_statistics_info*'''
        
        #save incoming msg here  
        self._map_msg = None    #MapInfo.msg
        self._robot_msgs = []   #list of RobotInfo.msg
        self._instance_msg = None
        self._solution_msgs = []#list of MAPFSolution.msg

        #flag if MapInfo.msg already received, so Map is initialised
        self._map_received = False
        #list of flags for every robot
        #if RobotInfo message received for the robot with ID, then list[ID-1]=True
        #self._robots_received = [] #Notice: ID starts with 1 while list with 0!        
        #self._solutions_received = []
        
    def callback_mapinfo(self, msg):
        '''
        This function will be called every time a message on the topic is received: */mapf_map_info*

        If the number of the robots has not changed since the last time this function has been called 
        then:
        >1) set the init flags for every robot to FALSE (meaning the robot is not ready yet)   
        >2) delete all old messages received for the robots on the following topics: */mapf_robot_info_"ID"*
        
        If the number of the robots has changed since the last time this function has been called then:   
        (this is although TRUE if this function is beeing called for the first time)
        >1) delete old Subscribers for the topics: */mapf_robot_info_"ID"*  
            delete old Publishers for the topics: */mapf_robot_action_"ID"*  
            
        >2) for every robot create a Publisher and Subscriber  
        (Publisher for */mapf_robot_info_"ID"* and Subscriber for */mapf_robot_action_"ID"*)
        
        >3) set the init flags for every robot to FALSE (meaning the robot is not ready yet)   
        
        >4) delete all old messages received for the robots on the following topics:   
           >> */mapf_robot_info_"ID"*  
           >> */mapf_instance_info_"ID"*  
           >> */mapf_instance_solution_"ID"*  
        
        :param msg: mapf_interfaces/MapInfo.msg - message from the topic /mapf_map_info
        '''

        self.ctrl_state = MODE_INIT
        self.current_id_cycle = msg.id
        
        #save the received msg
        self._map_msg = msg
        #mark that map message has already been received
        self._map_received = True
        
        #if the number of robots from last time hasn't changed then
        if not self.number_robots_changed(msg):
            self._robot_init = [False]*len(self.subscribers_robot)
            self._robot_msgs = [None]*len(self.subscribers_robot)
            return
        
        self.clear_robots()
        self.number_robots = msg.number_robots
        
        #create publisher[RobotAction.msg] and subscriber[RobotInfo.msg] for every robot
        for i in range (0, msg.number_robots):
            #create subscriber
            subscriber = self.create_subscription(RobotInfo, 'mapf_robot_info_' + str(i+1),
                                                    self.callback_robotinfo, 1)
            self.subscribers_robot.append(subscriber)
            #create publisher
            publisher = self.create_publisher(RobotAction, 'mapf_robot_action_' + str(i+1), 1)
            self.publishers_robot.append(publisher)

        self._robot_init = [False]*len(self.subscribers_robot)
        self._robot_msgs = [None]*len(self.subscribers_robot)

    def callback_robotinfo(self, msg):    
        '''
        This function will be called every time a message on the following topics is received: */mapf_robot_info_"ID"*  

        CONTROLLER checks whether the ID in the message is equal to the ID of the current cycle.   
        If the numbers are not equal, then the information in the message is old and will be ignored.  
        If the numbers are equal then, the CONTROLLER saves the message and sets the INIT-flag for the corresponding robot to TRUE!  

        :param msg: mapf_interfaces/RobotInfo.msg - message from the topic /mapf_robot_info_ID
        '''

        if msg.id != self.current_id_cycle:
            self.get_logger().info("RobotInfo.msg received, but with an old ID = %d. Skip the message" % msg.id)
            self.get_logger().info("msg.id = %d" % msg.id)
            self.get_logger().info("self.current_id_cycle = %d" % self.current_id_cycle)
            return    
        self._robot_init[msg.robot_id-1] = True
        self._robot_msgs[msg.robot_id-1] = msg
        print("Initialising ROBOT with ID: ", msg.robot_id)    

    def callback_solution(self, msg):
        '''
        This function will be called every time a message on the following topics is received: */mapf_instance_solution*  

        CONTROLLER checks whether the ID in the message is equal to the ID of the current cycle.   
        If the numbers are not equal, then the information in the message is old and will be ignored.  
        If the numbers are equal, then the CONTROLLER saves the message/solution.

        :param msg: mapf_interfaces/MAPFSolution.msg - message from the topic /mapf_instance_solution
        '''
        
        #skip solutions for old instances 
        if msg.id != self.current_id_cycle:
            self.get_logger().info("MAPFSolution.msg received, but with an old ID = %d. Skip the message" % msg.id)
            self.get_logger().info("msg.id = %d" % msg.id)
            self.get_logger().info("self.current_id_cycle = %d" % self.current_id_cycle)
            return
        self.get_logger().info("MAPFSolution.msg received, ID = %d, STEP: %d, algorithm: %s" % (msg.id, msg.step, msg.name_algorithm))                              
        #self._solution_msgs.append(msg)
        self._solution_msgs.insert(0, msg)
        
    def publish_instance_msg(self):
        '''
        This function generates a ros2 message of type [mapf_interfaces/MAPFInstance.msg] and publishes it to the following topic: */mapf_instance_info*  

        The message will be generated by fusing the information from the arrived messages on the following topics:  
        */mapf_map_info* [mapf_interfaces/MapInfo.msg] and   
        */mapf_robot_info_ID* [mapf_interfaces/RobotInfo.msg] for every robot 
        '''
        msg = MAPFInstance()
        msg.id = self.current_id_cycle
        msg.step = self._map_msg.step
        msg.map = self._map_msg
        for robot_msg in self._robot_msgs:
            msg.robots.append(robot_msg)
        self.publisher_instance.publish(msg)
        #save the published message for later to check with corresponding answer
        self._instance_msg = msg
        #delete all solutions for an older instance!!!!
        self._solution_msgs = []

    def publish_action_msg(self, solution_number=0):         
        '''
        This function generates a ros2 message of type [mapf_interfaces/RobotAction.msg] and publishes it to the following topic: */mapf_robot_action_ID*  
        
        The message will be generated from the received solution by the pathplanner: */mapf_solution_info* [mapf_interfaces/MAPFSolution.msg].  
        The solution defines the paths for every robot. Every path is defined by a set of points. The action for every robot will be defined by the difference 
        between the first two points from their path.
        The first point in the path describes the actual position of the robot, the second point the next target point from the path.
        
        :param int: solution_number - defines which solution should be used for generating the actions for the robot  
        (at the moment the CONTROLLER can save only one solution at the same time, but in future it should be possible to save more than one 
        solution and choose the best one of those. For that matter it should be possible to define which solution should be used here)
        '''
        for pathInfoMsg in self._solution_msgs[solution_number].paths:
        #for pathInfoMsg in self.solution_msg.paths:
            msg = RobotAction()
            msg.id = self.current_id_cycle
            msg.step = pathInfoMsg.step
            msg.robot_id = pathInfoMsg.robot_id
            if len(pathInfoMsg.path) < 2:
                msg.move_x = 0
                msg.move_y = 0
                msg.rotate = 0.0
            else:
                msg.move_x = pathInfoMsg.path[1].pos_x - pathInfoMsg.path[0].pos_x
                msg.move_y = pathInfoMsg.path[1].pos_y - pathInfoMsg.path[0].pos_y
                msg.rotate = pathInfoMsg.path[1].theta - pathInfoMsg.path[0].theta
                msg.path = pathInfoMsg.path
            self.publishers_robot[msg.robot_id].publish(msg)
            time.sleep(0.001)
                      
    def publish_statistics_msg(self, solution_number=0):
        '''
        This function generates a ros2 message of type [mapf_interfaces/StatisticInfo.msg]
        and publishes it to the following topic: */mapf_statistics_info*  
        
        The content for the message can be extracted directly from the saved solution (MAPFSolution.msg).

        :param int: solution_number - defines which solution should be used for generating the message  
        (at the moment the CONTROLLER can save only one solution at the same time, but in future it should be possible to save more than one 
        solution and choose the best one of those. For that matter it should be possible to define here which solution should be used)
        '''
        #msg = self._solution_msgs[solution_number].statistics
        self.publisher_statistics.publish(self._solution_msgs[solution_number].statistics)

    def check_init_robots(self):   
        '''
        This function checks whether the flag for every robot is set to TRUE. 
        
        This function is used to check whether the information about all the robots
        has been received by the CONTROLLER from the SIMULATION.

        :return: **True**, if the flag for every robot is set to TRUE in the CONTROLLER  
        **False**, otherwise

        '''
        if self._robot_init is None:
            return False
        for robot in self._robot_init:
            if robot != True:
                return False
        return True

    def all_agents_at_target(self):
        '''
        This function checks if all robots have visited all their checkpoints. If it is the case then all robots have reached their targets!
        :return: **True**, if all robots have visited all their checkpoints  
        **False**, otherwise
        '''
        for robot_msg in self._robot_msgs:
           for checkpoint_msg in robot_msg.checkpoints:
                if not checkpoint_msg.visited:
                   #print("CHECKPOINT NOT VISITED")
                   #print(checkpoint_msg.position.pos_x)
                   #print(checkpoint_msg.position.pos_y)
                   return False
        return True
             
    def number_robots_changed(self, msg):
        '''
        This function checks whether the number of the robots has changed from the last time (last time CONTROLLER received the MapInfo.msg). 
        
        :param msg: mapf_interfaces/MapInfo.msg - message from the topic /mapf_map_info

        :return: **True**, if the number of the robots has changed or if the CONTROLLER has not received any MapInfo.msg so far  
        **False**, if the number of the robots has not changed
        '''
        if not self._map_received:
            return True
        if self.number_robots != msg.number_robots:
            return True
        return False

    def clear_robots(self):
        '''
        This function is executing the following procedures:
        >   1) deleting all Subscribers to and received messages on the following topics: /mapf_robot_info_"ID"  
            2) deleting all Publishers to the following topics: /mapf_robot_action_"ID"  
            3) deleting all created Instances for the pathplanners and all received solutions from the pathplanners  
            4) at the end resets the STATE of the CONTROLLER to MODE_INIT 

        This function is called when the number of the robots has changed. In that case the old data will be deleted 
        and a new initialisation of the CONTROLLER with the new number of robots will be carried out.
        '''
        self.publishers_robot = []
        self.subscribers_robot = []
        self._robot_msgs = []   #list of RobotInfo.msg
        self._instance_msg = None
        self._solution_msgs = []#list of MAPFSolution.msg
        
        #list of flags for every robot
        #if RobotInfo message received for the robot with ID, then list[ID-1]=True
        #self._robots_received = [] #Notice: ID starts with 1 while list with 0!        
        #self._solutions_received = []
        self.ctrl_state = MODE_INIT

    def print_missing_robot_info(self):
        '''
        This function prints the ID-s of those robots of which the CONTROLLER is missing the information about
        (have not received a RobotInfo.msg for these robots with the id of the current cycle).  

        This function is used as a help-function for diagnostic purposes.
        '''
        print("Missing RobotInfo.msg for the robots with the following IDs: [", end="")
        id = 1
        for robot in self._robot_init:
            if robot != True:
                print(id, end="")
                print("; ", end="")
            id +=1
        print("]")

def run():

    '''
        **Main function**: creates the CONTROLLER and runs the ros2 node in a while loop. 

        While loop is runnning as long as the ros2 node **mapf_evaluator_node** exists.  
        Every loop cycle the ros2 node is spinned for a defined period (default: 0.1sec) so it can process all data.  

        >   1) At the beginning of every cycle CONTROLLER finds itself in the state MODE_INIT.  

        >    2) CONTROLLER changes it's state to MODE_WAIT_SIM after a MapInfo.msg is arrived   

        >    3) CONTROLLER moves to the state MODE_PUB_INSTANCE after receiving all needed data
        (RobotInfo.msg for every robot in the map).    

        >    4) After generating and publishing the instance (which can be solved by a pathplanner) 
            CONTROLLER waits for the solution (state: MODE_WAIT_SOLVER).      

        >    5) After receiving a solution from a pathplanner CONTROLLER generates actions for the robots
            (state: MODE_PUB_SIM). After sending the actions to the SIMULATION the CONTROLLER 
            goes back to the initial state MODE_INIT.

    '''

    rclpy.init()
    controller = Controller()

    #spin ros for 0.1seconds
    #time_spin = 2.0
    time_spin = 0.1
    #after publishing the MAPFInstance message to the solver wait for the solution
    #if after the given timeout no solution received from the solver then publish the MAPFInstance messsage again
    timeout_pub_instance = 60.0  
    
    print("STATE: Waiting for the MapInfo.msg to the topic /mapf_map_info")

    while(rclpy.ok()):            

        print()
        print("Time: " + str(datetime.utcnow()) + "; STATE of the CONTROLLER: " + str(controller.ctrl_state))

        #spin ros2 for a certain time to get all the messages
        timeout_spin = time.time() + time_spin
        while time.time()<timeout_spin:
            rclpy.spin_once(controller, timeout_sec=0)
        
        rclpy.spin_once(controller, timeout_sec=0)
        
        #STATE 0: wait till a MapInfo.msg received    
        if controller.ctrl_state == MODE_INIT:
            print("Details: MODE_INIT - Waiting for MapInfo.msg to the topic /mapf_map_info")
            if controller._map_received:
                controller.ctrl_state = MODE_WAIT_SIM
            else:
                continue
    
        #STATE 1: wait till all RobotInfo.msg received    
        if controller.ctrl_state == MODE_WAIT_SIM:
            print("Details: MODE_WAIT_SIM - Waiting for all RobotInfo.msg to the topic /mapf_robot_info_X where X = ID of the robot")
            if controller.check_init_robots():
                if not controller.all_agents_at_target():
                    controller.ctrl_state = MODE_PUB_INSTANCE
                else:
                    print("Details: All robots have already reached their target, return to STATE 0")
                    continue
            else:
                controller.print_missing_robot_info()
                continue
            
        #STATE 2: create and publish a MAPFInstance.msg (composed of MapInfo.msg and all RobotInfo.msg)
        if controller.ctrl_state == MODE_PUB_INSTANCE:
            print("Details: MODE_PUB_INSTANCE - Create and publish a MAPFInstance.msg to the topic /mapf_instance_info")
            controller.publish_instance_msg()                
            controller.ctrl_state = MODE_WAIT_SOLVER
            time_repeat = time.time() + timeout_pub_instance
        #STATE 3: wait till a MAPFSolution.msg received)
        if controller.ctrl_state == MODE_WAIT_SOLVER:
            print("Details: MODE_WAIT_SOLVER - Waiting for solution from pathplanner")
            
            if len(controller._solution_msgs) != 0:
                controller.ctrl_state = MODE_PUB_SIM
            #if after 30sec no answer received from solver then go back to STATE 2 (pub MAPFInstance.msg again)
            if time.time()>time_repeat:
                controller.ctrl_state = MODE_PUB_INSTANCE
        #STATE 4: create and publish RobotAction.msg for the simulation
        if controller.ctrl_state == MODE_PUB_SIM:
            print("Details: MODE_PUB_SIM - Create and publish RobotAction.msg for every robot")
            controller.publish_action_msg(solution_number=0) 
            controller.publish_statistics_msg(solution_number=0) 
            controller._solution_msgs = []               
            controller._map_received = False
            controller.ctrl_state = MODE_INIT
            
    #end while loop


if __name__ == '__main__':
    sys.exit(run())
    

    ''''
    def msg_changed_map(self, msg):
        if self._map_msg != msg:
            return True
        return False        
    '''

    '''
    def clear(self):
        self.publishers_robot = []
        self.subscribers_robot = []
        self._map_msg = None    #MapInfo.msg
        self._robot_msgs = []   #list of RobotInfo.msg
        self._instance_msg = None
        self._solution_msgs = []#list of MAPFSolution.msg
        #flag if MapInfo.msg already received, so Map is initialised
        self._map_received = False
        #list of flags for every robot
        #if RobotInfo message received for the robot with ID, then list[ID-1]=True
        #self._robots_received = [] #Notice: ID starts with 1 while list with 0!        
        #self._solutions_received = []
        self.ctrl_state = MODE_INIT
    '''

    '''
        def callback_mapinfo(self, msg):
    
        #if already received a MapInfo.msg and the new message is not different from the old one, then return
        if self._map_received and not self.msg_changed_map(msg):
            return        
        #reset previous data 
        self.clear()
        self._map_msg = msg
        for i in range (0, msg.number_robots):
            subscriber = self.create_subscription(RobotInfo, 'mapf_robot_info_' + str(i+1),
                                                    self.callback_robotinfo, 1)
            self.subscribers_robot.append(subscriber)
            publisher = self.create_publisher(RobotAction, 'mapf_robot_action_' + str(i+1), 1)
            self.publishers_robot.append(publisher)

        self._robot_init = [False]*len(self.subscribers_robot)
        self._robot_msgs = [None]*len(self.subscribers_robot)
        self._map_received = True
        '''
