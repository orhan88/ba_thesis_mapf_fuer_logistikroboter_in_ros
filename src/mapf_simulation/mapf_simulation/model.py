from mapf_simulation.visualizerItem import *
from mapf_simulation.visualizerGraphicItem import *
from mapf_simulation.modelView import ModelView

import re
import time
from mapf_simulation.action import Action

import rclpy
from rclpy.node import Node
from mapf_interfaces.msg import MapInfo

class Model(Node):
    '''
    This class stores the information about the elements in the simulation: robots, tasks and checkpoints.  
    It is also responsible for the communication between the CONTROLLER and the simulation through the ROS2-interfaces.  
    The visualization of the simulation is done by *modelView*.  
    The computation of the simulation is done by *Model*.
    '''
    def __init__(self):
        """"""
        #------------------------------INIT ROS2 NODE --------------------------------------

        #Init ROS2 Node with the name mapf_robot_ID, here ID is the ID of the robot
        Node.__init__(self, 'mapf_model_node')

        #Create a publisher for the topic "mapf_robot_info_ID" to publish the information 
        #of the robot with the custom message type "RobotInfo.msg"
        self.publisher = self.create_publisher(MapInfo, 'mapf_map_info', 10)
        '''**rclpy.publisher.Publisher** - ros2 publisher to the topic */mapf_map_info*'''
        self.msg_id = -1
        '''
        This id describes the number of the current cycle in the simulation.   
        By the start of every new cycle the id will be incremented.  
        A new cycle begins by publishing a message to the topic */mapf_map_info* [mapf_interfaces/MapInfo]
        A cycle is finished when an action has been executed for every item in the model for the current step.  
        (sim->controller->planner->controller->sim)
        '''

        #-----------------------------------------------------------------------------------
        self._windows = []
        
        self._items = {}
        self._graphic_items = {}
        self._new_items = {}
        self._editable = True

        self._grid_size = (1, 1)
        
        self._nodes = []                #pairs of x and y
        self._nodes_start = []          #pairs of x and y
        
        self._blocked_nodes = [(1,1)]   #pairs of x and y
        self._blocked_nodes_start = self._blocked_nodes.copy()
        
        self._node_ids = {}
        
        self._init_step = True
        self._num_steps = 0
        self._current_step = 0
        
        #----------------------------------------------------------------------------------
        #DATA FROM USER INPUT
        #input map file - name
        self._mapf_filename_map = ""
        #input scene file - name
        self._mapf_filename_scene = ""
        #input buckets from GUI
        self._buckets_list = []
        #input number robots from GUI
        self._mapf_number_robots = 0
        
        #----------------------------------------------------------------------------------
        #DATA FROM INPUT FILES
        #map size (from .MAP file)
        self._mapf_map_size_x = None
        self._mapf_map_size_y = None
        #map data as array (from .MAP file)
        self._mapf_map_data = []
        #max number of robots (from .SCEN file)
        self._mapf_max_number_robots = None
        #robots start positions (from .SCEN file)
        self._mapf_robot_start = []   #pairs of x and y   ->  [(1,1)]
        #robots positions (from .SCEN file)
        self._mapf_robot_pos = []     #pairs of x and y   
        #robots goal positions (from .SCEN file)
        self._mapf_robot_goals = []   #pairs of x and y
        #robots buckets numbers (from .SCEN file)
        self._mapf_robot_buckets = [] 
        #robots heuristics (from .SCEN file)
        self._mapf_robot_heuristics = []
        #----------------------------------------------------------------------------------

    #CLEAR
    def clear(self):
        """
        This function clears the model. All stored data will be deleted. This function is used only when a new map is loaded! 
        """
        for window in self._windows:
            if isinstance(window, ModelView):
                window.clear()

        #destroy all registered ROS2 Nodes
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                if isinstance(item, Node):
                    item.destroy_node()
                    pass

        self._items = {}
        self._graphic_items = {}
        self._new_items = {}
        self._editable = True

        self._grid_size = (1, 1)
        self._nodes = []                
        self._blocked_nodes = [(1,1)]   
        
        self._init_step = True
        self._num_steps = 0
        self._current_step = 0
        
        self._mapf_filename_map = ""
        self._mapf_filename_scene = ""
        self._buckets_list = []

        
        self._mapf_map_size_x = None
        self._mapf_map_size_y = None
        self._mapf_map_data = []
        self._mapf_number_robots = 0
        self._mapf_max_number_robots = None

        self._mapf_robot_start = []   
        self._mapf_robot_pos = []   
        self._mapf_robot_goals = []  
        self._mapf_robot_buckets = []
        self._mapf_robot_heuristics = []

        self.update_windows()

    def clear_scene(self):
        """
        This function clears the scene of the model. 
        This function is used only when a new scene file is loaded into the model! 
        Following actions will be performed:
        -   all registered ROS2-nodes for robots destroyed
        -   all items of the following kinds removed: RobotROS, CheckpointROS and TaskROS
        -   all previously saved data from the .SCEN file also deleted
        """
        
        #destroy all registered ROS2 Nodes
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                if isinstance(item, Node):
                    item.destroy_node()
                    pass
        
        self._items = {}
        self._graphic_items = {}
        self._new_items = {}
        
        self._init_step = True
        self._num_steps = 0
        self._current_step = 0
        
        #new added
        self._mapf_filename_scene = ""
        self._buckets_list = []
        
        self._mapf_number_robots = 0
        self._mapf_max_number_robots = None

        self._mapf_robot_start = []  
        self._mapf_robot_pos = []   
        self._mapf_robot_goals = []  
        self._mapf_robot_buckets = []
        self._mapf_robot_heuristics = []

        self.update_windows()

    '''
    def clear_robots(self):
        """
        This function is used when the number of robots in the simulation has been changed by the user. 
        In this case old data will be deleted and new data generated. 
        Following actions will be performed:
        -   all registered ROS2-nodes for robots destroyed
        -   all items of the following kinds removed: RobotROS, CheckpointROS and TaskROS
        """
        #destroy all registered ROS2 Nodes
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                if isinstance(item, Node):
                    item.destroy_node()
                    pass
        
        self._items = {}
        self._graphic_items = {}
        self._new_items = {}
        
        self.update_windows()
    '''

    def clear_actions(self):
        """
        This function deletes all actions for all objects (robots, tasks, checkpoints) in the simulation.
        """
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                item.clear_actions()
       
    def clear_older_actions(self):
        """
        This function deletes those actions for all objects (robots, tasks, checkpoints) in the simulation 
        which are no longer relevant.   

        This is usually the case when the user pauses the simulation, navigates a few steps back 
        and then starts the simulation again. Then all actions for steps higher than the current step
        are no longer relevant and should be deleted.
        """
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                item.clear_older_actions(self._num_steps)
        
    def reset(self):
        """
        This function deletes all actions for all objects (robots, tasks, checkpoints) in the simulation and 
        resets the number of steps to zero (meaning no steps have been executed in the simulation so far).

        This function is used for Drag&Drop. 
        After moving an object in the window the number of steps should be set to zero 
        and all prevous actions discarded. 
        """
        self.clear_actions()
        self._num_steps = 0
        self._current_step = 0

        self.update_windows()

    #UPDATE
    def update_windows(self):
        """
        This function performs an update of all windows saved in a speficic list in the model. 
        This is used to update the data in all windows after some information in the model has changed.
        """
        for window in self._windows:
            window.update()

    def add_window(self, window):
        """
        This function adds a widget to the list of windows. Then every window from this list can be updated at the same time. 
        The update will be executed by the function *update_windows()* when some information in the model has changed.
    
        :param window:   QtWidgets.QWidget -   window which should be updated when some data in the model has changed
        """
        if window not in self._windows:
            self._windows.append(window)

    def remove_window(self, window):
        """
        This function removes a specific widget from the list of windows which have to be updated every time the model has changed.

        :param window:   QtWidgets.QWidget -   window which should be removed from the list
        """
        if window in self._windows:
            self._windows.remove(window)

    def update(self, update_windows = True):
        """
        This function performs a step forward for every item in the model

        :param update_windows:   bool, optional -  defines if the windows / visualization should be updated after changing the data in the model
        :return: **int** -  the current step which is increased by 1 after every update (step forward)
        """
        for items_dic in self._items.values():
            for item in items_dic.values():
                item.on_step_update(self._current_step)
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                item.do_action(self._current_step)
        
        self._current_step += 1
        if self._current_step > self._num_steps:
            self._num_steps = self._current_step

        if(update_windows):
            self.update_windows()

        return self._current_step

    #<
    def undo(self):
        """
        This function performs a step backwards for every item in the model and then updates the windows.  
        This function is used after clicking the BUTTON "<" in the gui

        :return: **int** -  the current step which is decreased by 1 after every step backwards
        """
        if self._current_step == 0 or self._current_step == -1:
            return self._current_step
        self._current_step -= 1
        for items_dic in self._items.values():
            for item in items_dic.values():
                item.on_step_undo(self._current_step)
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                item.undo_action(self._current_step)
        
        self.update_windows()
        
        print("Current step: " + str(self._current_step))
        return self._current_step
    
    #>
    def redo(self):
        """
        This function performs a step forward for every item in the model and then updates the windows.  
        This function is used after clicking the BUTTON ">" in the gui

        :return: **int** -  the current step which is increased by 1 after every step forward
        """
        if self._current_step == self._num_steps:
            return self._current_step

        for items_dic in self._items.values():
            for item in items_dic.values():
                item.on_step_update(self._current_step)

        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                item.do_action(self._current_step)  

        self._current_step += 1
        self.update_windows()

        print("Current step: " + str(self._current_step))
        return self._current_step
    
    #<<
    def restart(self):
        """
        This function performs a step backwards for every item in the model until it reaches the step 0
        and then updates the windows.  
        This function is used after clicking the BUTTON "|<" in the gui
        """
        if self._current_step == 0 or self._current_step == -1:
                return self._current_step
            
        while(self._current_step > 0):
            
            self._current_step -= 1
            for items_dic in self._items.values():
                for item in items_dic.values():
                    item.on_step_undo(self._current_step)
            for items_dic in self._graphic_items.values():
                for item in items_dic.values():
                    item.undo_action(self._current_step)
            
        print("Current step: " + str(self._current_step))    
        self.update_windows()

    #>>
    def skip_to_end(self):
        """
        This function performs a step forward for every item in the model until it reaches the last simulated step. 
        and then updates the windows.  
        This function is used after clicking the BUTTON ">|" in the gui
        """
        while(self._current_step < self._num_steps):
            for items_dic in self._items.values():
                for item in items_dic.values():
                    item.on_step_update(self._current_step)

            for items_dic in self._graphic_items.values():
                for item in items_dic.values():
                    item.do_action(self._current_step)  
            self._current_step += 1 
        print("Current step: " + str(self._current_step))
        self.update_windows()


    #MAP: MODIFY / ADD / REMOVE NODES
    def add_node(self, x, y, node_id = None):
        """
        This function adds a node to the map. Every node is described by x and y coordinates. 
        Every robot or checkpoint can be placed at a specific node. 

        :param x:       int             - The x coordinate on the grid
        :param y:       int             - The y coordinate on the grid
        :param node_id: int, optional   - A specific ID for the node (the id is not used at the moment)
        """
        if (x,y) in self._nodes:
            return

        self._nodes.append((x, y))
        self._node_ids[(x,y)] = node_id
        if (x,y) in self._blocked_nodes:
            self._blocked_nodes.remove((x,y))
        if x > self._grid_size[0] or y > self._grid_size[1]:
            self.set_grid_size(max(x, self._grid_size[0]), max(y, self._grid_size[1]))

    def remove_node(self, x, y):
        """
        This function removes a node from the map and so blocks the grid cell. 
        The grid cell can no longer be occupied by a robot or checkpoint.

        :param x:       int             - The x coordinate on the grid
        :param y:       int             - The y coordinate on the grid
        """
        if (x,y) not in self._nodes:
            return

        self._nodes.remove((x, y))
        if (x,y) not in self._blocked_nodes:
            self._blocked_nodes.append((x,y))

    def set_grid_size(self, X, Y, enable_nodes = False):
        """
        This function creates a map with size X and Y. 

        :param X:            int            - The horizontal size of the map
        :param Y:            int            - The vertical size of the map
        :param enable_nodes: bool, optional - Defines whether for every grid cells a node should be created 
                                              or the grid cells should be all blocked
        """
        if X < 1:
            X = 1
        if Y < 1:
            Y = 1

        to_remove = []
        for node in self._nodes:        #remove old nodes
            if node[0] > X:
                to_remove.append(node)
            elif node[1] > Y:
                to_remove.append(node)
        for node in to_remove:
            self._nodes.remove(node)
            self._blocked_nodes.remove(node)

        if enable_nodes:
            for x in range(self._grid_size[0] + 1, X + 1):
                for y in range(1, Y + 1):
                    self._nodes.append((x,y))

            for x in range(1, self._grid_size[0] + 1):
                for y in range(self._grid_size[1] + 1, Y + 1):
                    self._nodes.append((x,y))

        else:
            self._blocked_nodes = []
            for x in range(1, X+1):
                for y in range(1, Y+1):
                    self._blocked_nodes.append((x,y))
            for node in self._nodes:
                self._blocked_nodes.remove(node)
        self._grid_size = (X, Y)

    def is_node(self, x, y):
        """
        Returns whether the cell x,y is a node (free) or is blocked
        
        :param x:       int - The x coordinate on the grid
        :param y:       int - The y coordinate on the grid

        :return: **True**   -  if the cell is a free node
                 **False**  -  if the cell is a blocked node
        """
        return (x, y) in self._nodes

    def is_node_free_of_robots(self, x, y):
        """
        Returns whether the node x,y is free of robots
        
        :param x:       int - The x coordinate on the grid
        :param y:       int - The y coordinate on the grid
        
        :return: **False**  -  if there is a robot at the node x,y  
                 **True**   -  otherwise
                 
        """
        robots = self._graphic_items['robot_ros'].items()
        for key, robot in robots:
            if robot.get_position()[0] == x and robot.get_position()[1]==y:
                print("Node not free!")
                return False
        return True
        '''
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                -----
        '''
    

    #ITEMS: ADD / REMOVE - scene
    def create_item(self, item_kind, ID = None, add_immediately = False):
        """
        This function creates an item and saves it in the model.

        :param item_kind:       str             - The name if the item ("robot_ros", "checkpoint_ros", "task_ros")
        :param ID:              int             - The ID for the item (usually the ID will be incremented for every new item of the same kind)
        :param add_immediately: bool, optional  - Not used (always True)
        """
        item = None
        if ID is None:
            dic = None

            if item_kind == 'robot_ros':
                if 'robot_ros' not in self._graphic_items:
                    self._graphic_items['robot_ros'] = {}
                dic = self._graphic_items['robot_ros']
            elif item_kind == 'checkpoint_ros':
                if 'checkpoint_ros' not in self._graphic_items:
                    self._graphic_items['checkpoint_ros'] = {}
                dic = self._graphic_items['checkpoint_ros']
            elif item_kind == 'task_ros':
                if 'task_ros' not in self._graphic_items:
                    self._graphic_items['task_ros'] = {}
                dic = self._graphic_items['task_ros']
        
            """
            if item_kind == 'shelf':
                if 'shelf' not in self._graphic_items:
                    self._graphic_items['shelf'] = {}
                dic = self._graphic_items['shelf']
            elif item_kind == 'pickingStation':
                if 'pickingStation' not in self._graphic_items:
                    self._graphic_items['pickingStation'] = {}
                dic = self._graphic_items['pickingStation']
            elif item_kind == 'chargingStation':
                if 'chargingStation' not in self._graphic_items:
                    self._graphic_items['chargingStation'] = {}
                dic = self._graphic_items['chargingStation']
            elif item_kind == 'robot':
                if 'robot' not in self._graphic_items:
                    self._graphic_items['robot'] = {}
                dic = self._graphic_items['robot']
            elif item_kind == 'order':
                if 'order' not in self._graphic_items:
                    self._graphic_items['order'] = {}
                dic = self._graphic_items['order']
            elif item_kind == 'checkpoint':
                if 'checkpoint' not in self._graphic_items:
                    self._graphic_items['checkpoint'] = {}
                dic = self._graphic_items['checkpoint']
            elif item_kind == 'task':
                if 'task' not in self._graphic_items:
                    self._graphic_items['task'] = {}
                dic = self._graphic_items['task']
            """

            ID = 1
            break_loop = False
            while not break_loop:
                break_loop = True
                for key in dic:
                    if str(key) == str(ID): 
                        ID += 1
                        break_loop = False
                        break

        ID = str(ID)
        if item_kind == 'robot_ros':
            item = RobotROS(ID)
        elif item_kind == 'checkpoint_ros':
            item = CheckpointROS(ID)
        elif item_kind == 'task_ros':
            item = TaskROS(ID)    
        '''
        elif item_kind == 'shelf':
            item = Shelf(ID)
        elif item_kind == 'robot':
            item = Robot(ID)
        elif item_kind == 'order':
            item = Order(ID)
        elif item_kind == 'checkpoint':
            item = Checkpoint(ID)
        elif item_kind == 'task':
            item = Task(ID)
        '''

        if item is not None:
            self.add_item(item, add_immediately)
        return item

    def _add_item2(self, item):
        """
        Saves the given item in the model.

        :param item: mapf_simulation.visualizerItem.VisualizerItem  - The item which should be saved in the model
        """
        if item is None:
            return
        dictionarie = self._map_item_to_dictionarie(item, True)
        if dictionarie is None:
            return
        if str(item.get_id()) in dictionarie:
            return
        key = (item.get_kind_name(), str(item.get_id()))
        if key in self._new_items: #and not ignore_duplicates:
            return
        item.set_model(self)
        if isinstance(item, VisualizerGraphicItem):
            item.enable_drag(self._editable)
        dictionarie[str(item.get_id())] = item

    def add_item(self, item, add_immediately = False):
        """
        Saves the given item in the model.

        :param item: mapf_simulation.visualizerItem.VisualizerItem  - The item which should be saved in the model
        :param add_immediately: bool, optional  - Not used (always True)
        """

        if add_immediately:
            return self._add_item2(item)
        if item is None:
            return
        key = (item.get_kind_name(), str(item.get_id()))
        if key in self._new_items:
            return
        self._new_items[key] = item

    """
    def accept_new_items(self, item_kinds = None):
        add_items = []
        if item_kinds == None:
            for item in self._new_items.values():
                add_items.append(item)
        else:
            for item_kind in item_kinds:
                for key in self._new_items:
                    if key[0] == item_kind:
                        add_items.append(self._new_items[key])
        self.discard_new_items(item_kinds)
        for item in add_items:
            self._add_item2(item)
        
    def discard_new_items(self, item_kinds = None):
        if item_kinds == None:
            self._new_items.clear()
            return
        delete_items = []
        for item_kind in item_kinds:
            for key in self._new_items:
                if key[0] == item_kind:
                    delete_items.append(key)
        for key in delete_items:
            del self._new_items[key]
    """

    def remove_item(self, item):
        """
        This function removes an item from the model.

        :param item: mapf_simulation.visualizerItem.VisualizerItem  - The item which should be removed from the model
        """
        if item is None:
            return
        key = (item.get_kind_name(), str(item.get_id()))
        item2 = self._new_items.pop(key, None)
        if item2 is not None:
            item2.set_model(None) 
            return

        dictionarie = self._map_item_to_dictionarie(item, True)
        if dictionarie is None:
            return
        if str(item.get_id()) not in dictionarie:
            return
        item.set_model(None)
        del dictionarie[str(item.get_id())]

    def filter_items(self, item_kind = None, 
                        ID = None, position = None, 
                        return_first = False,
                        return_non_buffered = True,
                        return_buffered = False):
        """
        This function searches the model for specific items and returns a list of items which match the criteria.

        :param item_kind:           str                 - The name of the item ("robot_ros", "checkpoint_ros", "task_ros")
        :param ID:                  int                 - The ID of the item 
        :param position:            (int, int, float)   - The position of the item (x,y,theta)
        :param return_first:        bool, optional      - if True then returns the first item that matches the given criteria
        :param return_non_buffered: bool, optional      - Not used (always True).
        :param return_buffered:     bool, optional      - Not used (always False)

        :return: one or a list of **mapf_simulation.visualizerItem.VisualizerItem**
        """
        result = []
        if ID is not None:
            ID = str(ID)
        if return_non_buffered:
            search_in = []
            if item_kind is None:
                for items_dic in self._graphic_items.values():
                    search_in.append(items_dic)
                if position is None:
                    for items_dic in self._items.values():
                        search_in.append(items_dic)
            else:
                if item_kind in self._graphic_items:
                    search_in.append(self._graphic_items[item_kind])
                if position is None:
                    if item_kind in self._items:
                        search_in.append(self._items[item_kind])
        
            for items_dic in search_in:
                if ID is not None:
                    if ID in items_dic:
                        item = items_dic[ID]
                        if position is None:
                            result.append(item)
                            if return_first:
                                return result
                        elif position == item.get_position():
                            result.append(item)
                            if return_first:
                                return result
                else:
                    for item in items_dic.values():
                        if position is None:
                            result.append(item)
                            if return_first:
                                return result
                        elif position == item.get_position():
                            result.append(item)
                            if return_first:
                                return result
            
        if return_buffered and position is None:
            for key in self._new_items:
                if ((key[0] == item_kind or item_kind is None)
                    and (key[1] == ID or ID is None)):
                    result.append(self._new_items[key])

        if return_first:
            return [None]
        return result

    def contains(self, item):
        """
        This function checks whether the model contains a specific item.

        :param item: mapf_simulation.visualizerItem.VisualizerItem  -   The item to search for

        :return: **True**   -  if the model contains the specific item  
                 **False**  -  otherwise
        """
        if item is None:
            return False
        if item.get_kind_name() in self._graphic_items:
            return item.get_id() in self._graphic_items[item.get_kind_name()]
        elif item.get_kind_name() in self._items:
            return item.get_id() in self._items[item.get_kind_name()]

    
    #SET FUNCTIONS 
    def set_editable(self, editable):
        """
        This function enables/disables the modification of the data (robots, checkouts) in the gui. 
        The data can be modified in the gui by drag&drop or rightclick->enable/disable node

        :param editable: bool  -   True (enable) or False(disable)
        """
        self._editable = editable
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                item.enable_drag(self._editable)
        
    def set_num_steps(self, num_steps):
        """
        This function sets the number of executed steps in the simulation.

        :param num_steps: int  -   The number of executed steps in the simulation
        """
        self._num_steps = num_steps

    def set_init_step(self, boolean):
        """
        This function defines whether the current cycle is the first cycle(here also called as "init step").   

        In the first cycle the SIMULATION has to build up the connection (ROS2) between the different robots and the CONTROLLER 
        which takes time. For that matterthe SIMULATION waits a little bit longer in the first cycle till the connection
        for all robots is established. 

        :param boolean: bool  -   True (INIT STEP) or False (NO INIT STEP)
        """
        self._init_step = boolean

    
    #GET FUNCTIONS
    def get_item(self, item_kind, ID, create = False, add_immediately = False):
        """
        This function searches the model for a specific item and returns it.

        :param item_kind:           str                 - The name of the item ("robot_ros", "checkpoint_ros", "task_ros")
        :param ID:                  int                 - The ID of the item 
        :param create:              bool, optional      - If True and the specific item doesn't exist then it will be created
        :param add_immediately:     bool, optional      - If False then the item will be added to a separate list 
        
        :return: **mapf_simulation.visualizerItem.VisualizerItem**  -   The item that matches the search criteria
        """
        items_dic = None
        if ID is not None:
            ID = str(ID)
        if item_kind in self._graphic_items:
            items_dic = self._graphic_items[item_kind]
        elif item_kind in self._items:
            items_dic = self._items[item_kind]
        elif (item_kind, str(ID)) in self._new_items:
            return self._new_items[(item_kind, str(ID))]
        elif create:
            return self.create_item(item_kind, ID, add_immediately)
        else:
            return None

        if ID in items_dic:
            return items_dic[ID]
        elif (item_kind, str(ID)) in self._new_items:
            return self._new_items[(item_kind, str(ID))]
        elif create:
            return self.create_item(item_kind, ID, add_immediately)
        else:
            return None

    def get_nodes(self):
        """
        Returns the nodes of the map which can be occupied by different items (robots, checkpoints..).
        
        :return: list of (int, int) -   List of pairs (x,y)
        """
        return self._nodes

    def get_node_id(self, node):
        """
        Returns the id of a specific node.
        
        :param node: pair of (int,int)  -   a node is defined by a pair of (x,y)
        :return: int -   The id of the node
        """
        
        if node in self._node_ids:
            return self._node_ids[node]
        return None

    def get_blocked_nodes(self):
        """
        Returns the blocked nodes of the map which can not be occupied by different items (robots, checkpoints..).
        
        :return: list of (int, int) -   List of pairs (x,y)
        """
        return self._blocked_nodes

    def get_grid_size(self):
        """
        Returns the size of the map.
        
        :return: (int, int) -   Size of the map (x, y)
        """
        return self._grid_size

    def get_max_number_robots(self): 
        """
        Returns the max number of robots in the map. 
        This number represents the number of robots which have been loaded from the .SCEN file.
        
        :return: int -   Max number of robots
        """
        if self._mapf_max_number_robots is None:
            return 1000000
        
        return self._mapf_max_number_robots

    def get_editable(self):
        """
        Returns whether the data of the model (robots, checkouts) can be changed in the gui. 
        The data can be modified in the gui by drag&drop or by right click->enable/disable node

        :return: **True**   -  Data can be changed in the gui  
                 **False**  -  Data can not be changed in the gui  
        """
        return self._editable

    def get_current_step(self):
        """
        Returns the current step in the simulation.
        
        :return: int -   The current step in the simulation
        """
        return self._current_step

    def get_num_steps(self):
        """
        Returns the number of executed steps in the simulation.
        
        :return: int -   The number of executed steps in the simulation
        """
        return self._num_steps


    #HELP FUNCTIONS
    def iterate_graphic_dictionaries(self):
        """
        This function can be used for iterating through the graphic dictionaries in the model.  
        
        :return: "robot_ros", "checkpoint_ros" and "task_ros"
        """
        for items_dic in self._graphic_items.values():
            yield items_dic

    def iterate_graphic_items(self):
        """
        This function can be used for iterating through the elements of every graphic dictionary in the model.  
        
        :return: every robot, every checkpoint and every task
        """
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                yield item

    def _map_kind_to_dictionarie(self, item_kind, 
                                    dictionaries, 
                                    create_dictionarie = False):
        
        if item_kind not in dictionaries:
            if not create_dictionarie:
                return None
            dictionaries[item_kind] = {}
        return dictionaries[item_kind]

    def _map_item_to_dictionarie(self, item, create_dictionarie = False):
        if item is None:
            return None
        dictionarie = None
        if isinstance(item, VisualizerGraphicItem):
            dictionarie = self._graphic_items
        elif isinstance(item, VisualizerItem):
            dictionarie = self._items
        else: 
            return None
        return self._map_kind_to_dictionarie(item.get_kind_name(), 
                                                dictionarie, 
                                                create_dictionarie)

    
    #------------------------------------------------------------------------------------------------------------------------
    #FUNCTIONS FOR GUI

    #INPUT MAP
    def mapf_update_map(self, filename, size_x, size_y, map_data):
        """
        This function loads a map in the model.

        :param filename:    string          - the complete path to the file including the name of the file
        :param size_x:      int             - the horizontal size of the map
        :param size_y:      int             - the vertical size of the map
        :param map_data:    list of str     - the relevant data from the file, every string represents a line from the file

        :return: **True**    -   If the update of the map was successful   
                 **False**   -   otherwise
        """
        self.clear()

        #    . - passable terrain
        #    G - passable terrain
        #    @ - out of bounds
        #    O - out of bounds
        #    T - trees (unpassable)
        #    S - swamp (passable from regular terrain)
        #    W - water (traversable, but not passable from terrain)

        #    passsable terrain:    . or G or S
        #    unpasssable terrain:  @ or O or T or W
        free_terrain = '.GS'

        self._mapf_filename_map = filename
        self._mapf_map_size_x = size_x          #width
        self._mapf_map_size_y = size_y          #height
        self._mapf_map_data = map_data          #2d Array
        
        #set the size for the map(x,y)
        try:            
            self.set_grid_size(self._mapf_map_size_x, self._mapf_map_size_y, True)
            #predefined function "set_grid_size" not perfect, node (1,1) still disabled, so correct it
            self._nodes.append((1,1))   
            
        except ValueError:
            print('x and y must be integer values')
            return False

        self._blocked_nodes = []
        #set the blocked nodes where terrain unpasssable
        y=1
        for line in self._mapf_map_data:
            x=1
            for char in line:
                if not bool(re.match('^['+free_terrain+']+$', char)):
                    #self._blocked_nodes.append((x,y))
                    self.remove_node(x,y)
                x=x+1
            y=y+1

        #save the start configuration 
        self._nodes_start = self._nodes.copy()
        self._blocked_nodes_start = self._blocked_nodes.copy()
        
        return True

    #INPUT SCENE
    def mapf_update_scene(self, filename, buckets, start, goals, heuristics, input_buckets):        
        """
        This function loads a scene in the model (robots, checkpoints, tasks ...).

        :param filename:        string              - the complete path to the file including the name of the file
        :param buckets:         list of int         - every entry represents the number of the bucket to which a robot belongs
        :param start:           list of (int, int)  - list of pairs (x,y), every pair represents a start position for a robot
        :param goals:           list of (int, int)  - list of pairs (x,y), every pair represents a goal position for a robot
        :param heuristics:      list of floats      - every entry represents the optimal length for every robot from his start to his goal
        :param input_buckets:   string              - the string represents the list of buckets to be loaded
        
        :return: **True**    -   If the update of the scene was successful   
                 **False**   -   otherwise
        """
        #check if map loaded?
        if self._mapf_filename_map == "":
            print("No map saved in the model. First you have to load the map and after update the scene!")
            return False

        self.clear_scene()

        self._mapf_filename_scene = filename
        #create list of buckets from input
        self._buckets_list = self.mapf_create_bucket_list(input_buckets)
        
        ID = 0
        #save all data from the input in the model if no buckets given
        if self._buckets_list == []:    
            self._mapf_robot_start = start          #list of pairs of x and y
            self._mapf_robot_pos   = start          #list of pairs of x and y
            self._mapf_robot_goals = goals          #list of pairs of x and y
            self._mapf_robot_buckets = buckets 
            self._mapf_robot_heuristics = heuristics 
            self._mapf_max_number_robots = len(self._mapf_robot_start)

        #save only the data corresponding to the right bucket number in the model
        else:
            for i in range(len(start)): 
                #print(ID)             
                if str(buckets[i]) in self._buckets_list:
                    self._mapf_robot_start.append(start[i])          
                    self._mapf_robot_pos.append(start[i])            
                    self._mapf_robot_goals.append(goals[i])          
                    self._mapf_robot_buckets.append(buckets[i])
                    self._mapf_robot_heuristics.append(heuristics[i])
                    ID += 1
            self._mapf_max_number_robots = ID
            
        self.update_windows()
        return True

    #ADD ROBOT
    def mapf_add_robot(self, ID, start = None, goal = None, bucket = None, heuristic = None):
        """
        This function adds a robot to the model.

        :param ID:          int         - The ID of the robot
        :param start:       (int, int)  - The start position of the robot
        :param goal:        (int, int)  - The target position of the robot
        :param bucket:      int         - The number of the bucket to which the robot belongs
        :param heuristics:  float       - The optimal length for the robot from his start position to his goal position
        
        :return: **mapf_simulation.visualizerGraphicItem.RobotROS**    -  The robot that has been created  
        """
        #Add Robot
        robot = self.create_item('robot_ros', ID, add_immediately = True)
        robot.set_starting_position(start[0], start[1])
        robot.set_position(start[0], start[1])
        robot.set_bucket(bucket)
        robot.set_heuristic(heuristic)
        
        #Create Checkpoint (goal)
        checkpoint = self.create_item('checkpoint_ros', ID, add_immediately = True)
        checkpoint.set_starting_position(goal[0], goal[1])
        checkpoint.set_position(goal[0], goal[1])

        #Create Task with only one Checkpoint as goal and set this task for the Robot
        task = self.create_item('task_ros', ID, add_immediately = True)
        task.add_checkpoint(checkpoint_id = checkpoint.get_id(), 
                            name = "ID: " + str(checkpoint.get_id()))
        '''
        task.add_checkpoint(checkpoint_id = checkpoint.get_id(), 
                            name = "ID: " + str(checkpoint.get_id()),
                            checkpoint_number = 0)
        '''
        task.set_robot(robot)
        
        return robot

    #INPUT NUMBER ROBOTS
    def mapf_update_number_robots(self, number_input):
        """
        This function changes the number of the robots which should be simulated.  
        This function is used when the user changes the number of robots through the gui.

        :param number_input:    int - The number of the robots which should be simulated
        
        :return: **True**    -   If the number of the robots in the simulation has been changed with success  
                 **False**   -   If the operarion failed for some reason
        """
        if self._mapf_filename_map == "":
            return False
        if self._mapf_filename_scene == "":
            return False
     
        ID = self._mapf_number_robots

        #check the number of robots is valid
        number_input = self.mapf_check_number_robots(number_input)

        #check how many new robots should be created if the difference > 0
        #or how many should be deleted if the difference < 0
        number_difference = number_input - self._mapf_number_robots

        if number_difference > 0:    
            if self._buckets_list != []:    
                for i in range(ID, len(self._mapf_robot_start)): #IGNORE: HIER FEHLER: ID +1
                    if str(self._mapf_robot_buckets[i]) in self._buckets_list:
                        ID = ID + 1
                        robot = self.mapf_add_robot(ID, self._mapf_robot_start[i], self._mapf_robot_goals[i], 
                                                    self._mapf_robot_buckets[i], self._mapf_robot_heuristics[i])
                        if ID >= number_input:
                            break
            else:
                for i in range(ID, len(self._mapf_robot_start)):                                   
                    ID = ID +1
                    robot = self.mapf_add_robot(ID, self._mapf_robot_start[i], self._mapf_robot_goals[i], 
                                                self._mapf_robot_buckets[i], self._mapf_robot_heuristics[i])
                    if ID >= number_input:
                        break
        
        elif number_difference < 0:   
            for i in range(abs(number_difference)): 
                robot = self._graphic_items['robot_ros'][str(ID)]
                
                tasks = robot.get_tasks()
                for task in tasks:
                    checkpoints = task.get_checkpoints()
                    for point in checkpoints:
                        self.remove_item(self._graphic_items['checkpoint_ros'][str(point[0].get_id())])
                        del point
                    self.remove_item(self._items['task_ros'][str(task.get_id())])
                    del task
                self.remove_item(self._graphic_items['robot_ros'][str(robot.get_id())])
                robot.destroy_node()
                del robot
                
                ID -= 1
                if ID < number_input:
                    break
        
        #save new number of robots
        self._mapf_number_robots = ID

        self.update_windows()
        return True

    def mapf_check_number_robots(self, number):
        """
        This function checks the input for the number of robots which should be simulated.   
        The input is not valid if it exceeds the max number of robots which have been loaded into the model from the scene file.
        
        :param number:    int   - The number of the robots which should be simulated
        
        :return: **int**        - The corrected number of the robots which should be simulated
        """
        if number is None:
            if self._mapf_robot_start is not None:
                return len(self._mapf_robot_start)
            else:
                return 0
        if number > self.get_max_number_robots(): 
            return self.get_max_number_robots()
        if number < 1:
            return 1
        return number
    
    #INPUT BUCKETS
    def mapf_create_bucket_list(self, input):        
        """
        This function creates a list of buckets from the input from the gui.  
        (only the robots with a number of bucket from this list should be loaded into the model from the scene file)
        
        :param input:       str      - The input string for the buckets from the gui 
        :return: list of **str**     - The list of buckets 
        """
        if input == "":
            return []
        groups = elements = buckets = []
        groups = input.split(';')
        for group in groups:
            elements = group.split('-')
            if len(elements)==1:
                if not elements[0] in buckets:
                    buckets.append(elements[0])
                continue
            elif len(elements) == 2:
                try:
                    min_ = int(elements[0])
                    max_ = int(elements[1])
                except:
                    continue
                for i in range(min_, max_+1):
                    if not str(i) in buckets:
                        buckets.append(str(i))
        return buckets


    #------------------------------------------------------------------------------------------------------------------------

    #CHECKBOXES: DRAW PATH
    def mapf_robots_enable_draw_start(self, draw_start):
        """
        This function enables / disables the drawing of starting points for all robots in the simulation.  
        This function is used when the user enables / disables the checkbox "DRIVEN PATH" in the gui.

        :param draw_path: bool  -   True or False
        """
        robots = self._graphic_items['robot_ros']
        for robot in robots.values():
            robot.set_draw_start(draw_start)

    def mapf_robots_enable_draw_path(self, draw_path):
        """
        This function enables / disables the drawing of the travelled path for all robots in the simulation.  
        This function is used when the user enables / disables the checkbox "DRIVEN PATH" in the gui.

        :param draw_path: bool  -   True or False
        """
        robots = self._graphic_items['robot_ros']
        for robot in robots.values():
            robot.set_draw_path(draw_path)

    def mapf_robots_enable_draw_future_path(self, draw_path):
        """
        This function enables / disables the drawing of the computed path for all robots in the simulation.  
        This function is used when the user enables / disables the checkbox "FUTURE PATH" in the gui.

        :param draw_path: bool  -   True or False
        """
        robots = self._graphic_items['robot_ros']
        for robot in robots.values():
            robot.set_draw_future_path(draw_path)
    
    
    #BUTTON RESET
    def mapf_reset_map_scene(self):
        """
        This function resets the simulation to it's initial state.   
        This function is used after pressing the button "RESET" in the gui by the user.
        """
        #reset map
        self._nodes = self._nodes_start.copy()
        self._blocked_nodes = self._blocked_nodes_start.copy()
        
        #reset scene
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                item.reset()
        
        self._num_steps = 0
        self._current_step = 0
        self.update_windows()

    #GET
    def mapf_get_map_height(self):
        """
        Returns the height of the map.
        
        :return: int -   The height of the map
        """
        return self._mapf_map_size_y

    def mapf_get_map_width(self):
        """
        Returns the width of the map.
        
        :return: int -   The width of the map
        """
        return self._mapf_map_size_x

    def mapf_get_filename_map(self):
        """
        Returns the name of the .MAP file (including the complete path)
        
        :return: str -   The name  of the .MAP file (including the complete path)
        """
        return self._mapf_filename_map

    def mapf_get_filename_scene(self):
        """
        Returns the name of the .SCEN file (including the complete path)
        
        :return: str -   The name of the .SCEN file (including the complete path)
        """
        return self._mapf_filename_scene

    def mapf_get_bucket_list(self):
        """
        Returns the list of the loaded buckets from the .SCEN file.  
        This function is used to adjust the title of the window   
        (Title of the window = "name of the .MAP file" + "name of the .SCEN file" + "list of loaded buckets")
        
        :return: str -   The list of loaded buckets from the .SCEN file
        """  
        if self._buckets_list == []:     
            return "All"
        
        buckets = ""
        for elem in self._buckets_list:
            buckets = buckets + str(elem) + ","
        return buckets[:-1]


    #------------------------------------------------------------------------------------------------------------------------
    #ROS2 CONNECTION
    
    #PUBLISH
    def mapf_init_next_step(self):
        """
        This function starts a new cycle by initialising all the robots and 
        publishing certain ROS2 messages.
        """
        #reset the state of all robots to "not ready"
        self.mapf_set_all_robots_ready(ready = False)
        #publish ROS Messages (RobotInfo.msg for every RobotROS Object, MapfInfo.msg)
        self.mapf_ros_publish_msg()

    def mapf_set_all_robots_ready(self, ready = False):
        """
        This function sets the state of all the robots to "READY" (True) or "NOT READY" (False).   
        This function is used to  initialize all the robots before a new cycle can be started. 
        
        :param ready: bool, optional   -   Sets the state of all the robots to "READY" (True) or "NOT READY" (False) 
        """
        robots = self._graphic_items['robot_ros']
        for robot in robots.values():
            robot.set_ready(ready)

    def mapf_ros_publish_msg(self,):    
        """
        This function creates and publishes a ROS2 message to the topic 
        */mapf_map_info* [mapf_interfaces/MapInfo].   
        Then it publishes a ROS2 message for every robot to the topic 
        */mapf_robot_info_"ID"* [mapf_interfaces/RobotInfo]. 
        """
        self.msg_id += 1
        
        #PUBLISH MAPINFO.MSG
        self.mapf_ros_publish_map_info(self._current_step)
        
        #WAIT A LITTLE BIT IF FIRST STEP
        if self._init_step:           
            #sleep small amount of time to give the controller(another node) the time 
            #to initialise all subscribers to the following publishers(robots), for 100 robots -> 0.3sec enough
            time.sleep(0.005*self._mapf_number_robots)
            self._init_step = False

        #PUBLISH ROBOTINFO.MSG FOR EVERY ROBOT
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                item.ros_publish_message_info(self.msg_id, self._current_step)
        
    def mapf_ros_publish_map_info(self, step):
        """
        This function creates and publishes a ROS2 message to the topic 
        */mapf_map_info* [mapf_interfaces/MapInfo].

        :param step: int   -   Sets the time step in the published messages (msg.step = time_step)
        """
        msg = MapInfo()
        msg.id = self.msg_id
        msg.step = step;
        msg.number_robots = self._mapf_number_robots
        msg.size_x = self._mapf_map_size_x
        msg.size_y = self._mapf_map_size_y
        for node in self._blocked_nodes:
            pos = Position()
            pos.pos_x = node[0]
            pos.pos_y = node[1]
            pos.theta = 0.0            
            msg.blocked.append(pos)
        self.publisher.publish(msg)        
        #return 0
       
    #SPIN
    def mapf_spin_once(self, timeout_end):
        """
        This function spins every ROS2-node in the simulation once.
        
        :param timeout_end: float   -   The next timeout in seconds 
        :return: (bool, bool)       -   First bool indicates if the simulation is ready to execute the next action for every robot.
                                        Second bool describes if the time has passed the given timeout.
        """
        #SPIN ALL ROBOTS ONCE (EVERY ROBOT IS A ROS2 NODE)
        robots = self._graphic_items['robot_ros']
        for robot in robots.values():
            robot.ros_spin_once()
        #SPIN MODEL ONCE(SELF IS A ROS2 NODE)
        rclpy.spin_once(self, timeout_sec=0)

        #CHECK IF SIMULATION RECEIVED ALL DATA FROM CONTROLLER FOR NEXT STEP
        simulation_ready = self.mapf_check_sim_ready()        
        timeout = False
        if time.time() > timeout_end: 
            timeout = True    
        return simulation_ready, timeout

    def mapf_check_sim_ready(self):
        """
        This function checks whether the simulation is ready to execute the next action for every robot.
        
        :return: **True**   -   The simulation is ready to execute the next action for every robot.  
                 **False**   -   The simulation is not ready yet to execute the next action for every robot.
        """
        robots = self._graphic_items['robot_ros']   
        #FOR EVERY ROBOT
        for robot in robots.values():
            #CHECK IF ROBOT READY (received RobotAction.msg for the next step)
            if not robot.is_ready():
                print("ROBOT WITH FOLLOWING ID IS NOT READY: ", end="", flush=True)
                print(robot.get_id())
                return False
            
            #CHECK IF ACTION VALID (next point is not blocked)
            x_pos = robot.get_position()[0]
            y_pos = robot.get_position()[1]
            action = robot.get_action(self.get_current_step())
            if action is not None:
                action_name = action.arguments[0].name
                action_value = action.arguments[1]
                if action_name == 'move':
                    action_x = action_value.arguments[0].number
                    action_y = action_value.arguments[1].number
                    node = (x_pos + action_x, y_pos + action_y)
                    if node in self._blocked_nodes:
                        print("WRONG ACTION for the ROBOT with ID: " +  str(robot.get_id()))
                        print("CANT MOVE TO THE FIELD: " +  str(node[0]) + ", " +  str(node[1]) + ", It is occupied")
                        return False        
        return True
        
    #RESET
    def mapf_reset_connection(self):
        """
        This function resets the connection between the SIMULATION and the CONTROLLER. 
        """
        self.msg_id += 1
        self._current_step = 0







'''
    SAVE TO FILE

    def to_init_str(self):
        s = []
        for node in self._nodes:
            s.append('init(object(node, '
                    + str(node[0] + (node[1]-1) * self._grid_size[0])
                    +  '), value(at, ('
                    + str(node[0]) + ', ' + str(node[1]) + '))).')
        for items_dic in self._graphic_items.values():
            for item in items_dic.values():
                s.append(item.to_init_str())
        for items_dic in self._items.values():
            for item in items_dic.values():
                s.append(item.to_init_str())
        return s
    

    def save_to_file(self, file_name):
        ofile = open(file_name, 'w')
        try:
            #head
            ofile.write('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
            ofile.write('\n%Grid size X: ' + str(self._grid_size[0]))
            ofile.write('\n%Grid size Y: ' + str(self._grid_size[1]))
            ofile.write('\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n')
            #body
            ofile.write('#program base.\n\n')

            ofile.write('%init\n')
            for ss in self.to_init_str():
                 ofile.write(str(ss.replace(".", ".\n")))
    
        except IOError:
            ofile.close()
            return
        ofile.close()

    def save_answer_to_file(self, file_name):
        ofile = open(file_name, 'w')
        try:
            for items_dic in self._graphic_items.values():
                for item in items_dic.values():
                    for action in item.to_occurs_str():
                        if action is not None:
                            ofile.write(action)

        except IOError:
            ofile.close()
            return
        ofile.close()

'''

'''
    INITS

    self._inits = []                #list of unhandled inits
    
    def add_init(self, init):
        self._inits.append(str(init) + '.')

    remove from def clear(), clear_scene(), clear_robots()
        
    remove from def to_init_str()
        for init in self._inits:
            s.append(str(init))
        

'''

'''

    HIGHWAYS

    self._highways = []             #pairs of x and y

    remove from def clear(), clear_scene(), clear_robots()

    def add_highway(self, x, y):
        self._highways.append((x, y))

    def is_highway(self, x, y):
        return (x, y) in self._highways

    def remove_highway(self, x, y):
        if (x,y) not in self._highways:
            return
        self._highways.remove((x,y))


    def get_highways(self):
        return self._highways

        
    remove from def to_init_str()
        for node in self._highways:
            s.append('init(object(highway, '
                    + str(node[0] + (node[1]-1) * self._grid_size[0])
                    +  '), value(at, ('
                    + str(node[0]) + ', ' + str(node[1]) + '))).')


'''
'''
    SOCKETS

    self._sockets = []
    self._notifier = None

    def add_socket(self, socket):
        if socket not in self._sockets:
            self._sockets.append(socket)

            

    def remove_socket(self, socket):
        if socket in self._sockets:
            self._sockets.remove(socket)

            
            
    def notify_sockets(self, iterator, value, step):
        if value.is_waiting():
            if self._notifier is not None:
                self._notifier.stop()

            self._notifier = QTimer()
            self._notifier.setSingleShot(True)
            self._notifier.timeout.connect(lambda: self.notify_sockets(iterator, value, step))
            self._notifier.start(100)
            return
        else:
            try:
                value = next(iterator)
            except StopIteration:
                return
            self.notify_sockets2(iterator, value, step)

            



    def notify_sockets2(self, iterator, value, step):
        if value.is_waiting():
            if self._notifier is not None:
                self._notifier.stop()

            self._notifier = QTimer()
            self._notifier.setSingleShot(True)
            self._notifier.timeout.connect(lambda: self.notify_sockets2(iterator, value, step))
            self._notifier.start(100)
            return
        value.done_step(step)
        self.notify_sockets(iterator, value, step)

        



    def accept_new_items(self, item_kinds = None):
        add_items = []
        if item_kinds == None:
            for item in self._new_items.values():
                add_items.append(item)
        else:
            for item_kind in item_kinds:
                for key in self._new_items:
                    if key[0] == item_kind:
                        add_items.append(self._new_items[key])
        self.discard_new_items(item_kinds)
        for item in add_items:
            self._add_item2(item)
        for socket in self._sockets:
            for item in add_items:
                socket.model_expanded(item.to_init_str())
            if len(add_items) > 0:
                socket.model_expanded('\n')    

    def update():
        if self._displayed_steps < self._current_step and len(self._sockets) > 0 and self._num_steps <= self._current_step:
            self._displayed_steps = self._current_step
            iterator = iter(self._sockets)
            value = next(iterator)
            value.done_step(self._current_step)
            self.notify_sockets(iterator, value, self._current_step)

            

    DISPLAYED STEPS

    self._displayed_steps = -1

    remove from def clear(), clear_scene(), clear_robots(), reset()
'''




