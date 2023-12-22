"""
To add a new kind of graphic items create a **child class
of VisualizerGraphicItem** in this file and add it in the 
**model.create_item** function to the model. This is the only function 
in the model that should be changed. The behaivor and values of the
object should be defindes inside of its own class.

To add new properties to a class modify the attributes of a class
and modify the following methods to adjust the objects behaivor:
 **reset**, **do_action**, **undo_action**, **ros_publish_message_info**. 
To adjust the appearance of an item modify the **set_rect** method.
Note: The set_color function is called automatically by the modelView for
every item if the colors are defined in the configuration.

Look at the method definitions of other classes for examples.
"""

from PyQt5.QtCore import *
from PyQt5.QtWidgets import QGraphicsItem
from PyQt5.QtWidgets import QGraphicsTextItem
from PyQt5.QtWidgets import QGraphicsRectItem
from PyQt5.QtWidgets import QGraphicsEllipseItem
from PyQt5.QtGui import QFont
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QBrush

import mapf_simulation.modelView 
import mapf_simulation.visualizerItem
from mapf_simulation.action import Action
from mapf_simulation.configuration import *

import rclpy
from rclpy.node import Node
from mapf_interfaces.msg import Position, RobotInfo, RobotAction, CheckpointInfo

import math

VIZ_STATE_MOVED     = 0x0001
VIZ_STATE_MOVE      = 0x0010
VIZ_STATE_ACTION    = 0xffff
#VIZ_STATE_DELIVERED = 0x0002
#VIZ_STATE_PICKED_UP = 0x0004
#VIZ_STATE_PUT_DOWN  = 0x0008
#VIZ_STATE_DELIVER   = 0x0020
#VIZ_STATE_PICK_UP   = 0x0040
#VIZ_STATE_PUT_DOWN2 = 0x0080
#VIZ_STATE_CHARGED   = 0x0100
#VIZ_STATE_CHARGE    = 0x1000

def calculate_color(first_color, second_color, multiplier):
    """
    This function interpolates between two color values.

    :return: QtGui.QColor()
    """

    red = (min(first_color.red(), second_color.red()), 
            max(first_color.red(), second_color.red()))
    green = (min(first_color.green(), second_color.green()), 
            max(first_color.green(), second_color.green()))
    blue = (min(first_color.blue(), second_color.blue()), 
            max(first_color.blue(), second_color.blue()))
    return QColor(
                red[0] + (red[1] - red[0]) * multiplier,
                green[0] + (green[1] - green[0]) * multiplier,
                blue[0] + (blue[1] - blue[0]) * multiplier)


class VisualizerGraphicItem(QGraphicsItem, mapf_simulation.visualizerItem.VisualizerItem):
    """
    This is the template class for visualizer graphic items.
    There should never be an instance of this class.
    A visualizer graphic item is a part of the visualizer model
    that is drawn on the model view and can perform actions.

    **Attributes**:
    -   **_kind_name**: str         -   The name of this kind of object.
    -   **_ID**: int                -   The id of this object.
    -   **_model**: model.Model     -   The model this object belongs to
    -   **_position**: tuple        -   A tuple that consists of two integers representing
                                        the position of the item on the grid.
    -   **_start_position**: tuple  -   The position of this item at timestep 0.
    -   **_dragged**: tuple         -   This is the position of this item before a drag
                                        action started.
    -   **_enable_drag**: bool      -   This is true if drag and drop is enabled for this item.        
                                        This will be set by the model. 
    -   **_graphics_item**: QGraphicsItem - This is the main graphic item of this item
                                            and is used for some default functions.
    -   **_text**: QGraphicsTextItem -  This is a text item that is drawn 
                                        on the model view and represents primarily the id of the item.
    -   **_actions**: list          -   This is a sorted list of the actions of this item. The index
                                        of every action is the time step at which the action occurs.    
    -   **_color**: QColor          -   The current main color of the item.
    -   **_colors**: list           -   A list of QColor values. Contains several colors for multi-colored
                                        items and color interpolation.
    -   **_display_mode**: int      -   This is set by the model view and determines whether
                                        the item text should be rendered and whether the item 
                                        should be rendered in the whole grid field.
                                        It is primarily defined by the zoom factor and the grid size.
    -   **_draw_path**: bool        -   This is true if the driven path of the item should be drawn in the simulation. 
                                        Should only be used for items that can have a path like robots.
    -   **_draw_future_path**: bool -   This is true if the calculated path for the item should be drawn in the simulation. 
                                        Should only be used for items that can have a path like robots.
    -   **_draw_startpoint**: bool  -   This is true if the start point of the item should be drawn in the simulation. 
                                        Should only be used for items that can have a start point like robots.
    -   **_state**: int             -   Consists of one or more state flags, that describes which action
                                        a item is currently doing.
    -   **_highlighted**: bool      -   This is true if the item will be highlighted.
                                        Highlighted elements are displayed larger

.
    """

    def __init__(self, ID = 0, x = 0, y = 0):
        """
        :param ID: int, optional    - The ID of the item
        :param x:  int, optional    - The x coordinate on the grid
        :param y:  int, optional    - The y coordinate on the grid
        """
        QGraphicsItem.__init__(self)
        mapf_simulation.visualizerItem.VisualizerItem.__init__(self)

        self._kind_name = ''
        self._id = ID
        self._model = None
        self._position = (x, y)
        self._start_position = (x, y)

        self._dragged = None
        self._enable_drag = False

        self._graphics_item = None  
        self._text = None
        self._pen = None
            
        self._actions = []
        self._future_path = []

        self._color = None
        self._colors = [QColor(0,0,0)]
        self._display_mode = 0

        self._draw_path = False
        self._draw_future_path = False
        self._draw_startpoint = False
        self.setAcceptedMouseButtons(Qt.MouseButtons(1))

        self._state = 0x00
        self._highlighted = False


    #SET FUNCTIONS
    def set_starting_position(self, x, y):
        """
        Sets the starting position of an item.

        :param x:  int      - The x coordinate on the grid
        :param y:  int      - The y coordinate on the grid
        """
        self._start_position = (x, y)

    def set_position(self, x, y):
        """
        Sets the current position of an item.

        :param x:  int      - The x coordinate on the grid
        :param y:  int      - The y coordinate on the grid
        """
        self._position = (x, y)

    def set_color(self, color, color_id = 0):
        """
        Sets a specific color of an item.

        :param color: QColor    - The color
        :param color_id:  int   - The id of the color that will be set
        """
        while color_id >= len(self._colors):
            self._colors.append(QColor(0,0,0))
        self._colors[color_id] = color

    def set_rect(self, rect):
        """
        Sets the rectangle that an item can use to draw things in.
        This equals usually one field of the grid in the model view.
        This function usually defines the appearance of an item
        on the grid.

        :param rect: QtCore.QRect   - The rectangle that an item can use to draw things in.
        """
        return

    def set_action(self, action, time_step):
        """
        Sets the action for the specific time step.
        Overrides existing actions at the time step but prints out
        a warning since this should never happen.

        :param action:      mapf_simulation.action.Action   -   This is the action that will be performed.
        :param time_step:   int                             -   This is the time step at which the action should be performed.
        """
        if time_step < 0:
            print(('Warning: invalid time step in occurs(object('
                    + str(self._kind_name) + ','
                    + str(self._id) + '),' + str(action)
                    + ',' + str(time_step) + ')'))
            print('time step is less than 0')
            return
        for ii in range((time_step + 1) - len(self._actions)):
            self._actions.append(None)
        if not self._actions[time_step] == None:
            print(('Warning: for object(' + str(self._kind_name)
                    + ', ' + str(self._id)
                    + ') multiple actions are defined at time step '
                    + str(time_step)))
        self._actions[time_step] = action

    def set_display_mode(self, display_mode):
        """
        Sets the current display mode.

        :param display_mode: int   - display mode (usually 0 or 1)
        """

        self._display_mode =  display_mode

    def set_draw_path(self, draw_path):
        """
        Sets whether the driven path of the object should be drawn.

        :param draw_path: bool  -   True or False
        """
        self._draw_path = draw_path

    def set_draw_future_path(self, draw_path):
        """
        Sets whether the calculated path for the object should be drawn.

        :param draw_path: bool  -   True or False
        """
        self._draw_future_path = draw_path

    def set_draw_start(self, draw_start):
        """
        Sets whether the starting point of the object should be drawn. 
        This is usually the case if the driven path should be drawn.

        :param draw_start: bool  -   True or False
        """
        self._draw_startpoint = draw_start

    def set_highlighted(self, highlighted):
        """
        Sets whether this item should be highlighted.

        :param highlighted: bool    -   True or False
        :return:    QtCore.QRect    -   The current rectangle of the object
        """
        self._highlighted = highlighted
        return self._graphics_item.rect()


    #GET FUNCTIONS
    def get_name(self):
        """
        Returns the name of the item.

        :return: str    -   The name of this kind of object.
        """

        return self._kind_name

    def get_position(self):
        """
        Return the position of an item.

        :return: tuple  -   The position of this item
        """
        return self._position

    def get_start_position(self):
        """
        Return the position of an item.

        :return: tuple  -   The start position of this item
        """
        return self._start_position

    def get_color(self, color_id):
        """
        Returns a specific color of an item.

        :param color_id:  int   - The id of the color that should be returned
        :return: QtGui.QColor   -   The color of this item with the specific id
        """
        if color_id < len(self._colors):
            return self._colors[color_id]
        return None

    def get_color(self):
        """
        Returns the current main color of an item.
        
        :return: QtGui.QColor   -   The current main color of this item
        """
        return self._color

    def get_rect(self):
        """
        Returns the current rectangle of an item.

        :return: QtCore.QRect   -   The current rectangle of this item
        """
        return None

    def get_action(self, time_step):
        """
        Returns the action at a specific time step.
        Returns None if no action is defined for this time step.

        :param time_step:   int                      -   The time step for which the action should be returned.
        :return: **mapf_simulation.action.Action**   -   The action at a specific time step.
        """
        if time_step >= len(self._actions):
            return None  #break, if no action is defined
        if self._actions[time_step] == None:
            return None  #break, if no action is defined     
        return self._actions[time_step]   

    def get_future_path(self):
        """
        Returns the calculated path for the item.

        :return: list of **mapf_interfaces.msg.Position**   -   The calculated path for this item.
        """
        return self._future_path

    def get_state(self):
        """
        Returns the current state of the item.
        
        :return: 0x00, VIZ_STATE_MOVED, VIZ_STATE_MOVE, VIZ_STATE_ACTION ...
        """
        return self._state

    def get_draw_path(self):
        """
        Returns whether the path of this item should be drawn.
        
        :return: **bool**   -   True or False.
        """
        return self._draw_path

    def get_draw_future_path(self):
        """
        Returns whether the future path of this item should be drawn.
        
        :return: **bool**   -   True or False.
        """
        return self._draw_future_path

    def get_draw_startpoint(self):
        """
        Returns whether the starting point of this item should be drawn.
        
        :return: **bool**   -   True or False.
        """
        return self._draw_startpoint

    def get_highlighted(self):
        """
        Returns whether the item should be highlighted.

        :return: **bool**   -   True or False.
        """
        return self._highlighted


    #EVENTS
    def mousePressEvent(self, event):
        """
        This is a overridden QT function and used for drag and drop.
        """
        if self._enable_drag:
            rect = self.get_rect()
            self._dragged = (event.scenePos().x(),
                             event.scenePos().y())
        event.accept()

    def mouseReleaseEvent(self, event):
        """
        This is a overridden QT function and used for drag and drop.
        """
        model_view = None
        if self._dragged is not None:
            self._dragged = None
            for view in self.scene().views():
                if isinstance(view, mapf_simulation.modelView.ModelView):
                    model_view = view
        if model_view is None:
            return
        node = model_view.scene_coordinates_to_node(event.scenePos().x(),
                                                    event.scenePos().y())
        
        self.setPos(0, 0)

        if node is not None:
            self.edit_position_to(node[0], node[1])
        
        model_view.update()
        event.accept()

    def mouseMoveEvent(self, event):
        """
        This is a overridden QT function and used for drag and drop.
        """
        if self._dragged is None:
            return
        self.setPos(event.scenePos().x() - self._dragged[0], 
                    event.scenePos().y() - self._dragged[1])
        event.accept()


    #VISUAL
    def determine_color(self, number, count, pattern = None):
        """
        Determines the color of an item dependent on its number.

        :param number: int  -   The number of this item in the models sequence. 
                                This does not necessarily have to be the ID of the item.
        :param count: int   -   The maximal count of all items in the model of this kind.
        :param pattern: int -   Not used.
        """
        return

    def paint(self, painter, option, widget):
        """
        This is a overridden QT function.
        """
        return self._graphics_item.paint(painter, option, widget)
    

    #ACTIONS / UPDATE
    def do_action(self, time_step):
        """
        Action handler. Must be implemented for items
        which can perform actions. This function will be called
        every time the model does one time step forward.

        :param time_step:   int     -   The time step for which the action should be executed.
        """

        return

    def undo_action(self, time_step):
        """
        Reverse action handler. Must be implemented for items
        which can perform actions. This function will be called 
        every time the model does one time step backwards.

        :param time_step:   int     -   The time step for which the action should be reversed.
        """

        return

    def clear_actions(self):
        """
        Deletes all actions for an object.
        """

        self._actions = []

    def clear_older_actions(self, time_step):
        """
        Deletes all actions for the given time step (self._actions[time_step]) 
        and after the given time step (self._actions[i>time_step]) for an object.
        
        :param time_step:   int -   Sets the steps (time>=time_step) for which the actions should be deleted.
        """
        if len(self._actions) == 0:
            return
        if len(self._actions)-1 < time_step:
            return
        if time_step == 0:
            self._actions = []
            return
    
        self._actions = self._actions[0:time_step]
        return
        
    def restart(self):
        """
        Resets the item to its original values.
        Sets the items position to its starting position.
        """
        self._position = self._start_position

    def reset(self):
        """
        Resets the item to its original values. This function will be called for every item if 
        the Button "RESET" in the GUI is pressed
        """
        return 


    #MOVE / DRAG
    def edit_position_to(self, x, y):
        """
        Sets a new starting position and new current position for an 
        item. If an item of the same kind is already on the same position
        they swap positions. This function is used to edit instances with
        the drag and drop feature.
        
        :param x:  int      - The x coordinate on the grid
        :param y:  int      - The y coordinate on the grid
        """
        
        if (x, y) == self._position:
            return
        item2 = self._model.filter_items(item_kind = self._kind_name,
                                        position = (x,y),
                                        return_first = True)[0]
        if item2 is not None:
            item2.set_position(self._position[0], self._position[1])
            item2.set_starting_position(
                self._position[0], 
                self._position[1])
            item2.clear_actions()
        
        self.set_position(x,y)
        self.set_starting_position(x, y)
        self.get_model().reset()
        self.get_model().mapf_reset_connection()

    def enable_drag(self, enable):
        """
        Enables and disables the drag and drop feature for the
        model editor.
        
        :param enable: bool  -   enable or disable drag & drop
        """     
        self._enable_drag = enable

    def boundingRect(self):
        """
        This is a overridden QT function and used for drag and drop.
        """
        return self._graphics_item.boundingRect()


    #ROS2 FUNCTIONS
    def ros_publish_message_info(self, msg_id, time_step):
        """
        Must be implemented for items which should publish the information about itself through ROS2 interfaces.
        This function will be called by the model every time an item does an action forward and has to publish
        the updated information about itself.

        :param msg_id:      int  -   The id of the current cycle (sim->controller->planner->controller->sim)
        :param time_step:   int  -   Sets the time step in the published message (msg.step = time_step)
        """
        return


class RobotROS(Node, VisualizerGraphicItem):
    """
    This class represents a robot.

    **Attributes**:
    -   **_kind_name**: str                     -   The name of this kind of item. This is always 'robot_ros'.
    -   **_tasks**:    list                    -   The list of tasks of the robot.
    -   **_graphics_item**: QGraphicsEllipseItem-   This is the main graphic item of the robot
                                                    and is used for some default functions.
    -   **_text**:          QGraphicsTextItem   -   This is a text item that is drawn on the model view
                                                    and represents primarily the id of the robot.
    """

    def __init__(self, ID = 0, x = 0, y = 0, theta = 0.0, bucket = 0, heuristic = 0.0):
        """
        :param ID:      int, optional       - The ID of the robot
        :param x:       int, optional       - The x coordinate on the grid
        :param y:       int, optional       - The y coordinate on the grid
        :param theta:   float, optional     - The orientation of the robot in angle (°)
        :param bucket:  int, optional       - The bucket to which the robot belongs
        :param heuristic: float, optional   - Optimal length from the start to the goal position
        """

        #------------------------------ ROS2 --------------------------------------------------------------
        #Init ROS2 Node with the name mapf_robot_ID, here ID is the ID of the robot
        Node.__init__(self, 'mapf_robot_' + str(ID) + '_node')

        #Create a publisher for the topic "mapf_robot_info_ID" to publish the information 
        #of the robot with the custom message type "RobotInfo.msg"
        self.publisher = self.create_publisher(RobotInfo, 'mapf_robot_info_' + str(ID) , 1)
        '''**rclpy.publisher.Publisher** - ros2 publisher to the topic */mapf_robot_info_''ID''*'''

        #Create a subscriber for the topic "mapf_robot_action_ID" to get the information about 
        #the next action to be executed by the robot, custom message type: RobotAction.msg
        self.subscriber = self.create_subscription(RobotAction, 'mapf_robot_action_' + str(ID) , self.callback_action, 100)
        '''**rclpy.subscription.Subscription** - ros2 subscriber to the topic */mapf_robot_action_''ID''*'''

        #True if RobotAction.msg received for the next step, otherwise False
        self._ready = False


        #------------------------------ GRAPHIC ITEM ------------------------------------------------------
        VisualizerGraphicItem.__init__(self, ID, x, y)
        self._kind_name = 'robot_ros'
        self._tasks = []
        
        self._graphics_item = QGraphicsEllipseItem(self)
        self._graphics_line_item = QGraphicsLineItem(self)
        self._text = QGraphicsTextItem(self)
        self._pen = QPen()
        self.setZValue(2.0)
        
        self._draw_path = True
        self._draw_future_path = True
        self._draw_startpoint = True
        
        #------------------------------ SAVE DATA ------------------------------------------------------
        self.set_position(x,y,theta)
        self.set_starting_position(x,y,theta)
        self.set_bucket(bucket)
        self.set_heuristic(heuristic)

        self._step = -1

    '''    
    def __del__(self):
        self.destroy_node()
    '''
    
    #SET FUNCTIONS
    def set_starting_position(self, x, y, theta = 0.0):
        """
        Sets the starting position of the robot.

        :param x:       int                 - The x coordinate on the grid
        :param y:       int                 - The y coordinate on the grid
        :param theta:   float, optional     - The orientation of the robot in angle (°)
        """
        self._start_position = (x, y, theta)

    def set_position(self, x, y, theta=0.0):
        """
        Sets the position of the robot.

        :param x:       int                - The x coordinate on the grid
        :param y:       int                - The y coordinate on the grid
        :param theta:   float, optional    - The orientation of the robot in angle (°)
        """
        self._position = (x, y, theta)

    def set_theta(self, theta = 0.0):
        """
        Sets the orientation of the robot.

        :param theta:   float, optional      - The orientation of the robot in angle (°).
        """
        self._position = (self._position[0], self._position[1], theta)

    def set_bucket(self, bucket = 0):
        """
        Assign the robot to a certain bucket.

        :param bucket:   int, optional      - The number of the bucket
        """
        self._bucket = bucket

    def set_ready(self, ready = False):
        """
        Sets the state of the robot (READY or NOT READY).  
        (A robot is READY if he knows what action he should execute as next)  
        (A robot is not READY if he still has not received an Action from the CONTROLLER for the next time step)  

        :param ready:   bool, optional    - True or False
        """
        self._ready = ready

    def set_heuristic(self, heuristic = 0.0):
        """
        Assign the optimal length for the robot from his start to his goal.

        :param heuristic:   float, optional - The optimal length for the robot from his start to his goal.
        """
        self._heuristic = heuristic

    def set_future_path(self, path):
        """
        Assign/Save the calculated path for the robot.

        :param path:   list of **mapf_interfaces.msg.Position** -    The calculated path for the robot.
        """
        self._future_path = path      

    def set_action(self, action, time_step):
        """
        Sets the action for the specific time step.
        Overrides existing actions at the time step but prints out
        a warning since this should never happen.

        :param action:      mapf_simulation.action.Action   -   This is the action that will be performed.
        :param time_step:   int                             -   This is the time step at which the action should be performed.
        """
        
        
        print("----------------------------------- SET ACTION -----------------------------------")
        if time_step < 0:
            print('Warning: invalid time step(smaller than 0), Object: ' 
                  + str(self._kind_name) + 
                  ', ID: ' + str(self._id) + 
                  ', Action: ' + str(action.get_name()) + 
                  ', Value: ' + str(action.get_value()) + 
                  ', Time Step: ' + str(time_step))
            print('time step is less than 0')
            return
        for ii in range((time_step + 1) - len(self._actions)):
            self._actions.append(None)
        if not self._actions[time_step] == None:
            print(('Warning: for object(' + str(self._kind_name)
                    + ', ' + str(self._id)
                    + ') multiple actions are defined at time step '
                    + str(time_step)))
        self._actions[time_step] = action


    #GET FUNCTIONS
    def get_bucket(self):
        """
        Returns the number of the bucket to which the robot was assigned.
        
        :return: **int**   -   The number of the bucket
        """
        return self._bucket
    
    def get_heuristic(self):
        """
        Returns the saved heuristic for the robot.
        
        :return: **float**   -   The heuristic
        """
        return self._heuristic
    
    def get_theta(self):
        """
        Returns the orientation of the robot.
        
        :return: **float**   -   The orientation of the robot
        """
        return self._position[2]
    
    def get_draw_path(self):
        """
        Returns whether the driven path of this robot should be drawn.
        
        :return: **bool**   -   True or False.
        """
        if self._highlighted:
            return True
        return self._draw_path

    def get_draw_future_path(self):
        """
        Returns whether the future path of this robot should be drawn.
        
        :return: **bool**   -   True or False.
        """
        if self._highlighted:
            return True
        return self._draw_future_path

    def get_draw_startpoint(self):
        """
        Returns whether the starting point of this robot should be drawn.
        
        :return: **bool**   -   True or False.
        """
        if self._highlighted:
            return True
        return self._draw_startpoint
    
    def get_tasks(self):
        """
        Returns the list of tasks for this robot (for the moment only one task per robot is supported).
        
        :return: list of mapf_simulation.visualizerItem.TaskROS   -   The list of tasks
        """
        return self._tasks

    def get_task(self, index):
        """
        Returns the task from the task list with a specific index.
        
        :param index: int                                   - The index of the task
        :return: **mapf_simulation.visualizerItem.TaskROS** - The selected task from the task list 
        """
        return self._tasks[index]
    
    def get_rect(self):
        """
        Returns the current rectangle of the robot. 

        :return: **QtCore.QRect** - The adjusted current rectangle of the QGraphicsItem which should represent the robot.
        """
        if self._display_mode == 0:
            rect = self._graphics_item.rect()
            width = rect.width()*2
            height = rect.height()*2
            rect.setLeft(rect.x() - 0.25*width)
            rect.setTop(rect.y() - 0.25*height)
            rect.setWidth(width)
            rect.setHeight(height)
            return rect
        elif self._display_mode == 1:
            rect = self._graphics_item.rect()
            width = rect.width()/0.9
            height = rect.height()/0.9
            rect.setLeft(rect.x() - 0.05*width)
            rect.setTop(rect.y() - 0.05*height)
            rect.setWidth(width)
            rect.setHeight(height)
            return rect

    def is_ready(self):
        """
        Returns whether the robot is ready to execute the next action.  
        (A robot is READY if he knows what action he should execute as next)  
        (A robot is not READY if he still has not received an Action from the CONTROLLER for the next time step) 
        
        :return: **bool**   -   True or False.
        """
        return self._ready
    

    #EVENTS (Drag&Drop)
    def mouseReleaseEvent(self, event):
        """
        This is a overridden QT function and used for drag and drop.
        """
        model_view = None
        self._dragged = None
        for view in self.scene().views():
            if isinstance(view, mapf_simulation.modelView.ModelView):
                model_view = view
        if model_view is None:
            return
        node = model_view.scene_coordinates_to_node(event.scenePos().x(),
                                                    event.scenePos().y())
        self.setPos(0, 0)

        if node is not None:
            #if item not moved but only clicked then highlight it and all the corresponding checkpoints
            if (self._position[0] == node[0] and self._position[1] == node[1]):
                self._highlighted = not self._highlighted
                for task in self._tasks:
                    for checkpoint, name in task.get_checkpoints():
                        checkpoint.set_highlighted(self._highlighted)
            elif self._enable_drag:
                if self._model.is_node_free_of_robots(node[0], node[1]):
                    self.edit_position_to(node[0], node[1])
        model_view.update()
        event.accept()

    def edit_position_to(self, x, y):
        """
        Sets a new starting position and new current position for the 
        robot. If another robot is already on the same position
        they swap positions. This function is used to edit instances with
        the drag and drop feature. 

        :param x:       int                - The x coordinate on the grid (new position for the robot)
        :param y:       int                - The y coordinate on the grid (new position for the robot)
        """
        if (x == self._position[0] and y == self._position[1]):
            return
        item2 = self._model.filter_items(item_kind = self._kind_name,
                                        position = (x,y),
                                        return_first = True)[0]
        
        if item2 is not None:
            item2.set_position(self._position[0], self._position[1])
            item2.set_starting_position(self._position[0], self._position[1])
            #item2.clear_actions()
        self.set_position(x,y, self.get_theta())
        self.set_starting_position(x, y, self.get_theta())
        #self.clear_actions()

        self.get_model().reset()
        self.get_model().mapf_reset_connection()


    #VISUAL
    def set_rect(self, rect):
        """    
        Sets the rectangle that a robot can use to draw things in.
        This equals one field of the grid in the model view.
        This function defines the appearance of the robot
        on the grid.

        :param rect:    QtCore.QRect    - The rectangle that a robot can use to draw things in.
        """
        scale = config.get('display', 'id_font_scale')
        bold = config.get('display', 'id_font_bold')

        #SET FONTSIZE, TEXT POSITION AND DEFAULT TEXT COLOR
        self._text.setFont(QFont('', rect.width()*0.3*scale))
        self._text.setPos(rect.x(), rect.y() + 0.1*rect.height())
        self._text.setDefaultTextColor(QColor(config.get('display', 'id_font_color')))

        #SET DISPLAY MODE
        if self._display_mode == 0:
            if bold:
                self._text.setHtml('<b>' + str(self._id) + '</b>')
            else:
                self._text.setHtml(str(self._id))
        else:
            self._text.setPlainText('')

        #SET POSITION AND SIZE OF GRAPHICS ITEM INSIDE THE CELL 
        rect2 = QRectF()
        rect2 = QRectF(rect.x() + 0.05*rect.width(), 
                            rect.y() + 0.05*rect.height(),
                            rect.width()*0.9,
                            rect.height()*0.9)
        
        self._pen.setWidth(rect.width()*0.1)
        
        #IF HIGHLIGHTED
        if self._highlighted:
            rect2 = QRectF(rect.x() + 0.0*rect.width(), 
                            rect.y() + 0.0*rect.height(),
                            rect.width()*1.0,
                            rect.height()*1.0)
            self._pen.setWidth(rect.width()*0.25)
            
        self._graphics_item.setRect(rect2)
        self._graphics_item.setPen(self._pen)

        try:
            action = self._actions[self._step+1].arguments[0]
            value = self._actions[self._step+1].arguments[1]
            if action.name == 'move' and len(value.arguments) == 2:
                try:
                    move_x = value.arguments[0].number
                    move_y = value.arguments[1].number
                    if not (move_x == 0 and move_y == 0):
                        self.set_theta(math.atan2(float(move_y),float(move_x))*180.0/math.pi)    
                except:
                    pass
        except:
            pass
        
        #DRAW THE RIGHT ORIENTATION OF THE ROBOT IN THE DIRECTION OF MOVING
        self._graphics_line_item.setLine(rect2.x() + rect2.width()  *0.5,   #start x
                                         rect2.y() + rect2.height() *0.5,  #start y 
                                         rect2.x() + rect2.width()  *0.5 + math.cos(self._position[2]/180*math.pi)* rect2.width()*0.5,      #end x
                                         rect2.y() + rect2.height() *0.5 + math.sin(self._position[2]/180*math.pi)* rect2.height()*0.5)      #end y                                  
        #UPDATE TOOLTIP
        self.update_tooltip()

    def update_tooltip(self):
        """
        Updates the tooltip for the robot.
        """
        tooltip = ("robot(" + str(self._id) + 
                   ")\nPosition: " + str(self.get_position()))
        self.setToolTip(tooltip)

    def determine_color(self, number, count, pattern = None):
        """
        Determines the color of the robot.
        
        :param number: int  -   The number of this item in the models sequence. 
                                This does not necessarily have to be the ID of the item.
        :param count: int   -   The maximal count of all items in the model of this kind.
        :param pattern: int -   Not used.
        """
        color = calculate_color(self._colors[0], self._colors[1], (float)(number)/count)
        self._color = color
        brush = QBrush(color)
        self._graphics_item.setBrush(brush)

    
    #ACTIONS / UPDATE   
    def clear_actions(self):
        """
        Deletes all saved actions for the robot.
        """
        self._actions = []

    def do_action(self, time_step):
        """
        Executes an action for the robot.
        This function will be called every time the model does
        one time step forward.
        
        :param time_step:   int     - The time step for which an action should be executed.
        :return:   **int**          - 1 if an action was done and 0 if an error occured.
        """
        self._state = self._state & ~VIZ_STATE_ACTION

        #SETS THE STATE FOR THE NEXT ACTION
        if time_step + 1 < len(self._actions):
            if self._actions[time_step + 1] is not None:
                #SETS THE STATE FOR THE NEXT ACTION
                self._state = self._state | VIZ_STATE_MOVE
        
        if time_step >= len(self._actions):
            return 0  #break, if no action is defined
        if self._actions[time_step] is None:
            return 0  #break, if no action is defined
        if self._model is None:
            return 0

        #TRY TO GET THE ACTION
        try:
            action = self._actions[time_step].arguments[0]
            value = self._actions[time_step].arguments[1]
        except:
            return 0

        #EXECUTE ACTION
        if action.name == 'move':
            if len(value.arguments) != 2: 
                return 0
          
            #MOVE 
            try:
                move_x = value.arguments[0].number
                move_y = value.arguments[1].number
                #MOVE AND CHANGE ORIENTATION IF VALUE NOT ZERO
                if not (move_x == 0 and move_y == 0):
                    self.set_position(self._position[0] + move_x, self._position[1] + move_y, math.atan2(float(move_y),float(move_x))*180.0/math.pi)
                    self._state = self._state | VIZ_STATE_MOVED
            except:
                #NO ACTION, NO CHANGE IN POSITION
                self.set_position(self._position[0], self._position[1], self._position[2])
            
            #MARK VISITED CHECKPOINTS
            for task in self._tasks:
                for checkpoint, name in task.get_checkpoints():
                    pos = checkpoint.get_position()
                    if pos[0] == self._position[0] and pos[1] == self._position[1]:
                        task.visit_checkpoint(checkpoint)
                        break

            self._step = time_step
            return 1
        return 0

    def undo_action(self, time_step):
        """
        Reverse an executed action for the robot.
        This function will be called every time the model does
        one time step backwards.  

        :param time_step:   int     - The time step for which an action should be reversed.
        :return:   **int**          - 1 if an action was done and 0 if an error occured.
        """
        self._state = self._state & ~VIZ_STATE_ACTION
        if time_step >= len(self._actions):  
            return 0  #break, if no action is defined
        if self._actions[time_step] == None: 
            return 0  #break, if no action is defined
        if self._model is None:
            return 0

        try:
            action = self._actions[time_step].arguments[0]
            value = self._actions[time_step].arguments[1]
        except:
            return 0

        #EXECUTE ACTION
        if action.name == 'move':
            if len(value.arguments) != 2: 
                return 0
            
            #UNMARK VISITED CHECKPOINTS
            for task in self._tasks:
                for checkpoint in task.get_checkpoints():
                    #checkpoint[0] -> checkpoint object, checkpoint[1] -> name of the checkpoint
                    checkpoint = checkpoint[0]
                    pos = checkpoint.get_position()
                    if pos[0] == self._position[0] and pos[1] == self._position[1]:
                        task.unvisit_checkpoint(checkpoint)
                        break

            #MOVE BACK
            try:
                move_x = value.arguments[0].number
                move_y = value.arguments[1].number
                #MOVE BACK AND CHANGE ORIENTATION IF VALUE NOT ZERO
                if not (move_x == 0 and move_y == 0):
                    self.set_position(self._position[0] - move_x, self._position[1] - move_y, math.atan2(float(move_y),float(move_x))*180.0/math.pi)
                    self._state = self._state | VIZ_STATE_MOVE
            except:
                #NO ACTION, NO CHANGE IN POSITION
                self.set_position(self._position[0], self._position[1], self._position[2])
            
            return 1
        return 0

    def reset(self):
        """
        Resets the item to its original values. This function will be called for every item if 
        the Button "RESET" in the GUI is pressed
        -   delete old actions for the robot
        -   move the robot back to it's start position
        -   mark all the checkpoints of the robot as not visited yet
        """
        self.clear_actions()
        self.set_position(self._start_position[0], self._start_position[1], self._start_position[2])
        self.unvisit_all_checkpoints()

    def restart(self):
        """
        Sets the robot to it's starting position.
        """
        super(self.__class__, self).restart()


    #TASK / CHECKPOINTS
    def add_task(self, task):
        """
        Adds a task to the robot (For the moment only one task is supported).

        :param task:   mapf_simulation.visualizerItem.TaskROS  -    A new task for the robot
        """
        if task is None:
            return
        if task in self._tasks:
            return
        self._tasks.append(task)
        task.set_robot(self)

    def check_robot_at_target(self):
        """
        This function checks whether the robot has reached it's target. 
        At the moment for one robot only one checkpoint and only one task are supported.
        Therefore this function checks only if the robot has reached the position 
        of the first checkpoint from it's first task from his task list.
        
        :return: **True**   -   if the robot has reached it's target   
                 **False**  -   otherwise
        """
        task = self.get_task(index =0)
        checkpoint = task.get_checkpoints()[0][0]
        if (checkpoint.get_position()[0] == self.get_position()[0] and 
            checkpoint.get_position()[1] == self.get_position()[1]):
            return True
        return False

    def unvisit_all_checkpoints(self):
        """
        This function iterates over the tasks of this robot and 
        marks all the checkpoints assigned to the tasks as not visited.
        """
        for task in self._tasks:
            task.unvisit_all_checkpoints()


    #ROS2 FUNCTIONS
    def ros_spin_once(self):
        """
        Spin the ROS2-node once (every robot is also a ROS2-node) to process the data sent through ROS2-interfaces
        """
        rclpy.spin_once(self, timeout_sec=0)

    def ros_publish_message_info(self, msg_id, time_step):
        """
        This function generates a ROS2 message of type [mapf_interfaces/RobotInfo.msg] 
        and publishes it to the following topic: */mapf_robot_info_"ID"*
        
        This function will be called by the model every time a robot performs an action and then has to publish
        the updated information about itself.

        :param msg_id:      int  -   The id of the current cycle (sim->controller->planner->controller->sim)
        :param time_step:   int  -   Sets the time step in the published message (msg.step = time_step)
        """

        msg  = RobotInfo()
        pos  = Position()
        checkpoint_msg = CheckpointInfo()
        goal = Position()
        
        #Add CYCLE-ID        
        msg.id = msg_id
        #Add STEP        
        msg.step = time_step
        #Add ROBOT_ID
        msg.robot_id = int(self.get_id())
        #Add BUCKET
        msg.bucket = int(self.get_bucket())
        #Add OPTIMAL_LENGTH
        msg.optimal_length = float(self.get_heuristic())
        #Add POSITION
        pos.pos_x, pos.pos_y, pos.theta = self.get_position()
        msg.position = pos

        #Add CHECKPOINTS
        task = self.get_task(index =0)
        checkpoints = task.get_checkpoints()
        for checkpoint, name in checkpoints:    
            checkpoint_msg.position.pos_x, checkpoint_msg.position.pos_y = checkpoint.get_position()
            checkpoint_msg.position.theta = 0.0
            checkpoint_msg.visited = checkpoint.get_visit_status()
            msg.checkpoints.append(checkpoint_msg)

        #publish message: RobotInfo.msg
        self.publisher.publish(msg)
        return

    #TODO: check action = move or action = rotate
    def callback_action(self, msg):
        """
        This function will be called every time a message on the following topics is received: 
        */mapf_robot_info_"ID"*

        This function creates an action from the received message and 
        saves the action in the robot for the (in the message) defined step!
        
        :param msg: mapf_interfaces/RobotAction.msg - message from the topic /mapf_robot_action_ID
        """

        print("Message received")
        print("STEP: %d" % msg.step)
        print("robot_id: %d"  % msg.robot_id)
        print("move x: %d"  % msg.move_x)
        print("move y: %d"  % msg.move_y)
        print("rotate: %d" % msg.rotate)
        
        #skip message if action not for current step
        if msg.step != self._model.get_current_step():
            print("SKIP MESSAGE, WRONG STEP")
            print("Robot ID: " + str(msg.robot_id))
            print("Msg step: " + str(msg.step))
            print("Current step: " + str(self._model.get_current_step()))
            return
        
        action = Action(msg.move_x, msg.move_y, 'move')
        self.set_action(action, msg.step)
        self.set_future_path(msg.path)
        self.set_ready(True)     


class CheckpointROS(VisualizerGraphicItem):
    """
    This class represents a Checkpoint.

    **Attributes**:
    -   **_kind_name**:     str                 -   The name of this kind of item. This is always 'checkpoint_ros'.
    -   **_graphics_item**: QGraphicsRectItem   -   This is the main graphic item of the checkpoint
                                                    and is used for some default functions.
    -   **_text**:          QGraphicsTextItem   -   This is a text item that is drawn on the model view
                                                    and represents primarily the id of the checkpoint.
    -   **_shine**:         bool                -   This is true if the checkpoint should be highlighted. Checkpoints
    """

    def __init__(self, ID = 0, x = 0, y = 0, theta = 0.0):
        """
        :param ID:      int, optional    - The ID of the item
        :param x:       int, optional    - The x coordinate on the grid
        :param y:       int, optional    - The y coordinate on the grid
        :param theta:   int, optional    - Orientation of the item (not used currently)
        """
        super(self.__class__, self).__init__(ID, x, y)
        self._kind_name = 'checkpoint_ros'
        self.set_position(x,y)
        self.set_starting_position(x,y)

        #self._graphics_item = QGraphicsEllipseItem(self)
        self._graphics_item = QGraphicsRectItem(self)
        self._text = QGraphicsTextItem(self._graphics_item)
        self._pen = QPen()
        self.setZValue(1.0)

        self._shine = False
        self._visited = False
        #self._checkpoint_number = 0
    

    #GET
    def get_rect(self):
        """
        Returns the current rectangle of an item.

        :return: QtCore.QRect   -   The current rectangle of this item
        """
        return self._graphics_item.rect()

    def get_visit_status(self):
        """
        Returns whether the item was visited by a roboter or not.

        :return: bool   -   True or False
        """
        return self._visited


    #VISUAL
    def determine_color(self, number, count, pattern = None):
        """
        Determines the color of the checkpoint.
        
        :param number: int  -   The number of this item in the models sequence. 
                                This does not necessarily have to be the ID of the item.
        :param count: int   -   The maximal count of all items in the model of this kind.
        :param pattern: int -   Not used.
        """
        color = calculate_color(self._colors[0], self._colors[1], (float)(number)/count)
        #color = QColor(255,255,255) #white
        self._color = color
        
        #if self._shine:
        #    color = QColor(51,255,0)
        self._graphics_item.setBrush(QBrush(color))

    def set_rect(self, rect):
        """    
        Sets the rectangle that a checkpoint can use to draw things in.
        This equals one field of the grid in the model view.
        This function defines the appearance of the checkpoint
        on the grid.

        :param rect: QtCore.QRect   - The rectangle that a checkpoint can use to draw things in.
        """
        scale = config.get('display', 'id_font_scale')
        bold = config.get('display', 'id_font_bold')
        
        #SET FONTSIZE, TEXT POSITION AND DEFAULT TEXT COLOR
        self._text.setFont(QFont('', rect.width()*0.3*scale))
        self._text.setPos(rect.x(), rect.y() + 0.1*rect.height())
        self._text.setDefaultTextColor(QColor(config.get('display', 'id_font_color')))

        #SET DISPLAY MODE
        if self._display_mode == 0:
            if bold:
                self._text.setHtml('<b>' + str(self._id) + '</b>')
            else:
                self._text.setHtml(str(self._id))
        else:
            self._text.setPlainText('')

        #SET POSITION AND SIZE OF GRAPHICS ITEM INSIDE THE CELL 
        rect2 = QRectF()
        rect2 = QRectF(rect.x() + 0.05*rect.width(), 
                            rect.y() + 0.05*rect.height(),
                            rect.width()*0.9,
                            rect.height()*0.9)

        self._pen.setColor(QColor(255, 0, 0, 255))
        self._pen.setWidth(rect.width()*0.1)

        #IF HIGHLIGHTED
        if self._highlighted:        
            rect2 = QRectF(rect.x() + 0.0*rect.width(), 
                            rect.y() + 0.0*rect.height(),
                            rect.width()*1.0,
                            rect.height()*1.0)
            self._pen.setColor(QColor(0, 0, 255, 255))
            self._pen.setWidth(rect.width()*0.2)

        #IF SHINE
        if self._shine:    
            self._pen.setColor(QColor(0, 255, 0, 255))
        
        #SAVE 
        self._graphics_item.setRect(rect2)        
        self._graphics_item.setPen(self._pen)
     

    #ACTIONS
    def do_action(self, time_step):
        """
        Resets the shine value at every time step.

        :param time_step:   int     -   Not used.
        """
        self._shine = False
        return

    def undo_action(self, time_step):
        """
        Resets the shine value at every time step.

        :param time_step:   int     -   Not used.
        """
        self._shine = False
        return

    def visit(self):
        """
        Sets the shine value to True.
        """    
        self._visited = True
        self._shine = True

    def unvisit(self):
        """
        Sets the shine value to False.
        """ 
        self._visited = False
        self._shine = False




'''
    VisualizerGraphicItem

    def to_occurs_str(self):
        """
        This function returns a list of strings that represents all
        actions of an item.        
        """

        actions = []
        count = 0
        for action in self._actions:
            if action is None:
                actions.append(None)
            else:
                actions.append('occurs(object('
                    + self._kind_name + ', '
                    + str(self._id) + '), '
                    + str(action) + ', '
                    + str(count) + ').\n')
            count = count + 1
        return actions

    def parse_init_value(self, name, value):
        """
        This function handels the input phrases for every item.
        This is called for every phrase the model receives with the
        following syntax: 
        init(object([object type], [object ID]), value([value name], [value])).
        While [object type] is the same as self._kind_name and [object ID]
        is the ID of the object and is the same as self._id.
        The model decides based on this value the object that receives
        the phrase.

        Parameters:
        name: str
            This is the name of the value.
        value: clingo.symbol.Symbol
            This is the actual value. It contains the [value]
            part of the phrase.        

        Returns 1 if the phrase cannot be parsed, -1 if one parameter is
        invalid and 0 if the function succeeded.
        """

        if value is None or name is None:
            return -1
        if name == 'at':
            pos = (value.arguments[0].number, value.arguments[1].number)
            self.set_starting_position(pos[0], pos[1])
            self.set_position(pos[0], pos[1])
            return 0
        return 1

    def to_init_str(self):
        """
        Converts the item to a string that represents the items values.
        This function is used to send the whole model to a solver
        and to save an instance to a file.
        """

        return ('init(object('
                + self._kind_name + ','
                + str(self._id) + '), value(at,('
                + str(self._position[0]) + ','
                + str(self._position[1]) + '))).')

'''

'''
    RobotROS

    self._position_history = [] #list of pairs of x, y and theta
     
    
    from set_rect():
         
        rect2 = QRectF(rect.x() + 0.25*rect.width(), 
                        rect.y() + 0.25*rect.height(),
                        rect.width()*0.5,
                        rect.height()*0.5)
        #draw pose of the robot
        self._graphics_line_item.setLine(rect2.x()+rect2.width()*0.5, rect2.y()+rect2.height()*0.5,  
                                         rect2.x()+rect2.width(),      
                                         rect2.y()+rect2.height()*0.5)

    """
    from def edit_position_to(self, x, y):

        shelf = self._model.filter_items('shelf',
                                        position = self._position,
                                        return_first = True)[0]
        shelf2 = self._model.filter_items('shelf',
                                        position = (x,y),
                                        return_first = True)[0]

        if shelf2 is not None and shelf is not None:
            if shelf.get_carried() is not None or shelf2.get_carried() is not None:
                shelf.set_position(x,y)
                shelf.set_starting_position(x,y)
                shelf2.set_position(self._position[0], self._position[1])
                shelf2.set_starting_position(self._position, self._position[1])
    """

    #delete this functions later
    def parse_init_value(self, name, value):
        """
        This function handels the input phrases for robots.
        This is called for every phrase the model receives with the
        following syntax: 
        init(object(robot, [robot ID]), value([value name], [value])).

        Parameters:
        name: str
            This is the name of the value.
        value: clingo.symbol.Symbol
            This is the actual value. It contains the [value]
            part of the phrase.        

        Returns 1 if the phrase cannot be parsed, -1 if one parameter is
        invalid and 0 if the function succeeded.   
        """

        result = super(self.__class__, self).parse_init_value(name, value)
        if result <= 0: return result
        if name == 'carries':
            shelf = self._model.get_item('shelf', value, True, True)
            self.set_initial_carries(shelf)
            self.set_carries(shelf)
            return 0
        elif name == 'energy':
            self.set_current_energy(value.number)
            self.set_initial_energy(value.number)
            return 0
        elif name == 'max_energy':
            self.set_max_energy(value.number)
            return 0
        elif name == 'energy_cost':
            self._energy_costs[value.arguments[0].name] = value.arguments[1].number
            return 0
        return 1

    def to_init_str(self):
        """
        Converts the robot to a string that represents the robots values.
        This function is used to send robots to a solver
        and to save robots to a file.
        """

        s = super(self.__class__, self).to_init_str()
        if self._initial_carries is not None:
            s += ("init(object(robot,"
                    + str(self._id) + "),value(carries,"
                    + str(self._initial_carries.get_id())
                    + ")).")
        s += ("init(object(robot," 
                + str(self._id) + "), value(max_energy,"
                + str(self._max_energy)
                + ")).")
        s += ("init(object(robot," 
                + str(self._id) + "), value(energy,"
                + str(self._current_energy)
                + ")).")
        for key in self._energy_costs:
            s += ("init(object(robot,"
                    + str(self._id) + "),value(energy_cost, ("
                    + key + ", " + str(self._energy_costs[key]) + "))).")
        return s

    def to_occurs_str(self):
        """
        This function returns a list of strings that represents all
        actions of a robot.        
        """

        actions = []
        count = 0
        for action in self._actions:
            str_out = ''
            if action is None:
                actions.append(None)
            else:
                str_out = ''
                if action is not None:
                    str_out = ('occurs(object('
                        + self._kind_name + ', '
                        + str(self._id) + '), '
                        + str(action) + ', '
                        + str(count) + ').\n')
                actions.append(str_out)
            count = count + 1
        return actions
'''

'''
    CheckpointROS

    in def set_rect:
        #pen.setStyle(Qt.DashLine)
        #pen.setStyle(Qt.DotLine)
        #space = 4
        #dashes = [1, space]
        #pen.setDashPattern(dashes)


    self._name = ""



    def set_name(self, name = ""):
        self._name = name

    
    

    def get_name(self):
        return self._name

    def set_position(self, x, y):
        """
        Sets the current position of the checkpoint.
        """
        self._position = (x, y)
        #super(self.__class__, self).set_position(x, y)
        
    def set_starting_position(self, x, y):
        """
        Sets the starting position of the robot.
        """
        self._start_position = (x, y)
        #super(self.__class__, self).set_starting_position(x, y)

    #delete this function later
    def parse_init_value(self, name, value):
        """
        This function handels the input phrases for checkpoints.
        This is called for every phrase the model receives with the
        following syntax: 
        init(object(checkpoint, [checkpoint ID]), value([value name], [value])).

        Parameters:
        name: str
            This is the name of the value.
        value: clingo.symbol.Symbol
            This is the actual value. It contains the [value]
            part of the phrase.

        Returns 1 if the phrase cannot be parsed, -1 if one parameter is
        invalid and 0 if the function succeeded.   
        """

        result = super(self.__class__, self).parse_init_value(name, value)
        if result != 1:
            return result

        if name == 'checkpoint' and len(value.arguments) == 3:
            if str(value.arguments[0]) not in self._ids:
                self._ids[str(value.arguments[0])] = []
            self._ids[str(value.arguments[0])].append((str(value.arguments[1]), value.arguments[2].number))
            return 0
        return 1

'''
'''
class Shelf(VisualizerGraphicItem):
    """
    This class represents a shelf.

    Attributes:
    _kind_name: str
        The name of this kind of item. This is always 'shelf'.
    _carried: Robot
        The robot that carries this shelf.
    _products: list
        This is a list of products. Every product is a triple.
        (product id, product amount, removed product amount)
    _graphics_item: QGraphicsEllipseItem
        This is the main graphic item of the shelf
        and is used for some default functions.
    _graphics_carried: QGraphicsEllipseItem
        This is a circle appearing on a carried shelf.
    _text: QGraphicsTextItem
        This is a text item that is drawn 
        on the model view and represents primarily the id of the shelf.
    """

    def __init__(self, ID = 0, x = 0, y = 0):
        """
        Parameters:
        ID : int, optional
            The ID of the shelf
        x: int, optional
            The x coordinate on the grid
        y: int, optional
            The y coordinate on the grid
        """

        super(self.__class__, self).__init__(ID, x, y)
        self._kind_name = 'shelf'
        self._carried = None
        self._products   = []
        self._graphics_item = QGraphicsEllipseItem(self)
        self._graphics_carried = QGraphicsEllipseItem()
        self._text = QGraphicsTextItem(self)
        self.setZValue(2.0)
        self.update_tooltip()

    def set_rect(self, rect):
        """
        Sets the rectangle that a shelf can use to draw things in.
        This equals one field of the grid in the model view.
        This function defines the appearance of the shelf
        on the grid.
        """

        if self._carried is not None:
            rect = self._carried.get_rect()

        scale = config.get('display', 'id_font_scale')
        bold = config.get('display', 'id_font_bold')
        self._text.setFont(QFont('', rect.width()*0.08*scale))
        self._text.setPos(rect.x(), rect.y() + 0.4*rect.height())
        self._text.setDefaultTextColor(
            QColor(config.get('display', 'id_font_color')))

        if self._display_mode == 0:
            if bold:
                self._text.setHtml('<b>S(' + str(self._id) + ')</b>')
            else:
                self._text.setHtml('S(' + str(self._id) + ')')
            self._graphics_item.setRect(rect.x() + 0.25*rect.width(), 
                                        rect.y() + 0.25*rect.height(),
                                        rect.width()*0.5,
                                        rect.height()*0.5)

            self._graphics_carried.setRect(rect.x() + 0.325*rect.width(), 
                                        rect.y() + 0.325*rect.height(),
                                        rect.width()*0.35,
                                        rect.height()*0.35)
        elif self._display_mode == 1:
            self._text.setPlainText('')
            self._graphics_item.setRect(rect.x() + 0.05*rect.width(),
                                        rect.y() + 0.05*rect.height(),
                                        rect.width()*0.9,
                                        rect.height()*0.9)
            self._graphics_carried.setRect(rect.x() + 0.125*rect.width(),
                                        rect.y() + 0.125*rect.height(),
                                        rect.width()*0.75,
                                        rect.height()*0.75)


    def set_carried(self, robot):
        """
        Sets the robot that carries this shelf at the current time step.
        Sets also the shelf that is carried by the robot to this.
        If the shelf is already carried by another robot the carried 
        shelf of this robot will be set to None.
        """

        #Checks if the shelf is already carried by the robot 
        #to prevent an infinite loop.
        if robot == self._carried:
            return
        temp = self._carried
        self._carried = robot
        if temp is not None:
            temp.set_carries(None)
        if self._carried is not None:
            self._graphics_carried.setParentItem(self._graphics_item)
            self._carried.set_carries(self)
        else:
            self._graphics_carried.setParentItem(None)

    def restart(self):
        """
        Reset the products in the shelf.
        Sets the shelf to its starting position.
        """

        super(self.__class__, self).restart()
        products = []
        for product in self._products:
            products.append((product[0], product[1], 0))
        self._products = products

    def to_init_str(self):
        """
        Converts the shelf to a string that represents the shelfs values.
        This function is used to send shelves to a solver
        and to save shelves to a file.
        """

        s = super(self.__class__, self).to_init_str()
        for product in self._products:
            s += ('init(object(product,' 
                    + str(product[0]) + '),value(on,('
                    + str(self._id) + ',' 
                    + str(product[1]) + '))).')
        return s

    def update_tooltip(self):
        """
        Updates the tooltip for the shelf.
        """

        tooltip = "shelf(" + str(self._id) + ")\nproducts:\n"
        for product in self._products:
            tooltip = tooltip + str(product) + "\n"
        self.setToolTip(tooltip)

    def find_product(self, product_id):
        """
        Returns a product with the given product id.
        If no product with the given id is on the shelf it returns None.
        """

        for product in self._products:
            if str(product_id) == str(product[0]):
                return product
        return None


    def set_product_amount(self, product_id, amount):
        """
        Sets the carried amount of a product on this shelf.
        If the shelf already contains products with the 
        given id the amount of products will be overriden.
        """

        product = self.find_product(product_id)
        if product is None:
            self._products.append((product_id, amount, 0))
        else:
            self._products.remove(product)
            self._products.append((product_id, amount, product[2]))
        self.update_tooltip()

    def add_product(self, product_id, amount):
        """
        Adds to the carried amount of a product on this shelf.
        If amount is 0 and the shelf already contains products
        with the given amount it will be set to 0.
        """

        product = self.find_product(product_id)
        if product is None:
            self._products.append((product_id, amount, 0))
        else: 
            if amount == 0:
                self._products.append((product_id, 0, 0))
            else:
                self._products.append((product_id, product[1] + amount, 0))
            self._products.remove(product)
        self.update_tooltip()

    def remove_product(self, product_id, amount):
        """
        Increases the removed amount counter of a product on the shelf.
        If the shelf does not contain a product with the 
        given id nothing happens.
        """

        product = self.find_product(product_id)
        if product is not None:
            self._products.append((product_id, product[1], product[2] + amount))
            self._products.remove(product)

    def delete_product(self, product_id):
        """
        Deletes a product from a shelf.
        If the shelf does not contain a product with the 
        given id nothing happens.
        """

        product = self.find_product(product_id)
        if product is not None:
            self._products.remove(product)
        self.update_tooltip()

    def determine_color(self, number, count, pattern = None):
        """
        Determines the color of the picking station.
        """
        
        color = calculate_color(self._colors[0], self._colors[1], (float)(number)/count)
        brush = QBrush(color)
        self._graphics_item.setBrush(brush)
        self._graphics_carried.setBrush(QBrush(self._colors[2]))

    def get_product_amount(self, product_id):
        """
        Returns the current amount of a specific product on a shelf.
        """

        product = self.find_product(product_id)
        if product is None:
            return 0
        return product[1] - product[2]

    def get_rect(self):
        """
        Returns the current rectangle of the shelf.
        """

        if self._display_mode == 0:
            rect = self._graphics_item.rect()
            width = rect.width()*2
            height = rect.height()*2
            rect.setLeft(rect.x() - 0.25*width)
            rect.setTop(rect.y() - 0.25*height)
            rect.setWidth(width)
            rect.setHeight(height)
            return rect
        elif self._display_mode == 1:
            rect = self._graphics_item.rect()
            width = rect.width()/0.9
            height = rect.height()/0.9
            rect.setLeft(rect.x() - 0.05*width)
            rect.setTop(rect.y() - 0.05*height)
            rect.setWidth(width)
            rect.setHeight(height)
            return rect


    def get_carried(self):
        """
        Returns the robot that carries the shelf at the current 
        time step. This can be None.
        """

        return self._carried

    def iterate_products(self):
        """
        Iterates through every product on the shelf.
        """

        for product in self._products:
            yield product

    def edit_position_to(self, x, y):
        """
        Sets a new starting position and new current position for the 
        shelf. If another shelf is already on the same position
        they swap positions. This function is used to edit instances with
        the drag and drop feature. Only shelf that are currently not carried 
        can be moved.        
        """

        if (x, y) == self._position:
            return
        if self._carried is not None:
            return
        item2 = self._model.filter_items(item_kind = self._kind_name,
                                        position = (x,y),
                                        return_first = True)[0]
        if item2 is not None:
            if item2.get_carried() is not None:
                return
            item2.set_position(self._position[0], self._position[1])
            item2.set_starting_position(self._position[0], self._position[1])
        self.set_position(x,y)
        self.set_starting_position(x, y)

    def mousePressEvent(self, event):
        """
        This is a overridden QT function and used for drag and drop.
        """

        if self._carried is not None:
            self._carried.mousePressEvent(event)
        else:
            super(self.__class__, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """
        This is a overridden QT function and used for drag and drop.
        """

        if self._carried is not None:
            self._carried.mouseMoveEvent(event)
        else:
            super(self.__class__, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        """
        This is a overridden QT function and used for drag and drop.
        """

        if self._carried is not None:
            self._carried.mouseReleaseEvent(event)
        else:
            super(self.__class__, self).mouseReleaseEvent(event)
'''

'''
class Checkpoint(VisualizerGraphicItem):
    """
    This class represents a Checkpoint.

    Attributes:
    _kind_name: str
        The name of this kind of item. This is always 'checkpoint'.
    _ids: dictionary
        A dictionary that contains all ids of this checkpoint.
        One checkpoint can be the destination of different kinds
        of waypoints like a goal and start node.
    _graphics_item: QGraphicsRectItem
        This is the main graphic item of the checkpoint
        and is used for some default functions.
    _text: QGraphicsTextItem
        This is a text item that is drawn on the model view
        and represents primarily the id of the checkpoint.
    _shine: bool
        This is true if the checkpoint should be highlighted. Checkpoints
        are highlighted when they were visited.
    """

    def __init__(self, ID = 0, x = 0, y = 0):
        super(self.__class__, self).__init__(ID, x, y)
        self._kind_name = 'checkpoint'

        self._ids = {}

        self._graphics_item = QGraphicsRectItem(self)
        self._text = QGraphicsTextItem(self._graphics_item)
        self._shine = False

    def set_rect(self, rect):
        """    
        Sets the rectangle that a checkpoint can use to draw things in.
        This equals one field of the grid in the model view.
        This function defines the appearance of the checkpoint
        on the grid.
        """

        scale = config.get('display', 'id_font_scale')
        bold = config.get('display', 'id_font_bold')
        font = QFont('', rect.width()*0.08*scale)
        self._text.setFont(font)
        self._text.setPos(rect.x(), rect.y() + 0.6*rect.height())
        self._text.setDefaultTextColor(QColor(config.get('display', 'id_font_color')))

        if self._display_mode == 0:            
            ss = ''
            if bold:
                ss = '<b>'
            for key in self._ids:
                count = 0
                for ii in self._ids[key]:
                    if count == 0:
                        ss = ss + '(' + key + ': ' + ii[0]
                    else:
                        ss = ss + ', ' + ii[0]
                    count += 1
                ss = ss + ')\n'

            if bold:
                ss += '</b>'

            self._text.setHtml(ss)
            self._graphics_item.setRect(rect.x(), rect.y(), rect.width(), rect.height())

        elif self._display_mode == 1:
            self._text.setPlainText('')
            self._graphics_item.setRect(rect.x(), rect.y(), rect.width(), rect.height())        

    def do_action(self, time_step):
        """
        Rests the shine value at every time step.
        """

        self._shine = False
        return

    def undo_action(self, time_step):
        """
        Rests the shine value at every time step.
        """

        self._shine = False
        return

    def visit(self):
        """
        Sets the shine value to true.
        """    

        self._shine = True

    def determine_color(self, number, count, pattern = None):
        """
        Determines the color of the checkpoint.
        """

        color = None
        if len(self._ids) == 1:
            for key in self._ids:
                color_id = self._ids[key][0][1] + 1
                if color_id < len(self._colors):
                    color = self._colors[color_id]    
                        
        if color is None:
            color = self._colors[0]

        if self._shine:
            color = calculate_color(color, QColor(255,255,255), 0.4)
        self._graphics_item.setBrush(QBrush(color))

    def parse_init_value(self, name, value):
        """
        This function handels the input phrases for checkpoints.
        This is called for every phrase the model receives with the
        following syntax: 
        init(object(checkpoint, [checkpoint ID]), value([value name], [value])).

        Parameters:
        name: str
            This is the name of the value.
        value: clingo.symbol.Symbol
            This is the actual value. It contains the [value]
            part of the phrase.

        Returns 1 if the phrase cannot be parsed, -1 if one parameter is
        invalid and 0 if the function succeeded.   
        """

        result = super(self.__class__, self).parse_init_value(name, value)
        if result != 1:
            return result

        if name == 'checkpoint' and len(value.arguments) == 3:
            if str(value.arguments[0]) not in self._ids:
                self._ids[str(value.arguments[0])] = []
            self._ids[str(value.arguments[0])].append((str(value.arguments[1]), value.arguments[2].number))
            return 0
        return 1

    def get_rect(self):
        """
        Returns the current rectangle of the checkpoint.
        """

        return self._graphics_item.rect()
'''

'''
class Robot(VisualizerGraphicItem):
    """
    This class represents a robot.

    Attributes:
    _kind_name: str
        The name of this kind of item. This is always 'robot'.
    _carried: Shelf
        The shelf that is carried by the robot.
    _initial_carries: Shelf
        The shelf that is carried by the robot at time step 0.
    _tasks: list
        The list of tasks of the robot
    _graphics_item: QGraphicsRectItem
        This is the main graphic item of the robot
        and is used for some default functions.
    _energy_bar_full: QGraphicsRectItem
        This graphic item represents a full energy bar.
    _energy_bar_empty: QGraphicsRectItem
        This graphic item represents a empty energy bar.
    _text: QGraphicsTextItem
        This is a text item that is drawn 
        on the model view and represents primarily the id of the robot.
    _initial_energy: int
        The robots energy at time step 0.
    _max_energy: int
        The robots maximal energy
    _current_energy: int
        The current energy of the robot
    _energy_costs: dictionary
        A dictionary that contains energy costs for actions.
        The cost should be an integer. The dictionary keys
        are the name of the actions.
    _parallel_delivers: dictionary
        A dictionary that contains deliver actions that are 
        processed simultaneously to other actions. The deliver actions
        are stored in lists to allow multiple deliveries at the same time
        step.
    """

    def __init__(self, ID = 0, x = 0, y = 0):
        """
        Parameters:
        ID : int, optional
            The ID of the robot
        x: int, optional
            The x coordinate on the grid
        y: int, optional
            The y coordinate on the grid
        """

        super(self.__class__, self).__init__(ID, x, y)
        self._kind_name = 'robot'
        self._carries = None
        self._initial_carries = None
        self._tasks = []
        self._graphics_item = QGraphicsRectItem(self)
        self._energy_bar_full = QGraphicsRectItem(self)
        self._energy_bar_empty = QGraphicsRectItem(self)
        self._text = QGraphicsTextItem(self)
        self.setZValue(1.0)

        self._energy_bar_empty.setBrush(QBrush(QColor(200,0,0)))
        self._energy_bar_full.setBrush(QBrush(QColor(0,200,0)))

        self._initial_energy = 0
        self._max_energy = 0
        self._current_energy = 0
        self._energy_costs = {}

        self._parallel_delivers = {}

    def set_position(self, x, y):
        """
        Sets the current position of the robot.
        Sets also the current position of a carried shelf to the same position.
        """
        super(self.__class__, self).set_position(x, y)
        if self._carries is not None:
           self._carries.set_position(x, y)


    def set_starting_position(self, x, y):
        """
        Sets the starting position of the robot.
        Sets also the starting position of a carried shelf to the same position.
        """

        super(self.__class__, self).set_starting_position(x, y)
        if self._initial_carries is not None:
           self._initial_carries.set_starting_position(x, y)

    def set_carries(self, shelf):
        """
        Sets the shelf that is carried by this robot the at current time step.
        Sets also the robot that carries the given shelf.
        If the robot already carries another shelf the other shelves 
        carried value will be set to None.
        """

        #Checks whether the shelf is already carried by the robot 
        #to prevent a infinite loop.
        if shelf == self._carries:
            return
        old = self._carries
        self._carries = shelf
        if old != None: old.set_carried(None)
        if self._carries != None: self._carries.set_carried(self)

    def set_initial_carries(self, shelf):
        """
        Sets a shelf that is carried at the begin.
        """
        self._initial_carries = shelf

    def set_rect(self, rect):
        """    
        Sets the rectangle that a robot can use to draw things in.
        This equals one field of the grid in the model view.
        This function defines the appearance of the robot
        on the grid.
        """

        scale = config.get('display', 'id_font_scale')
        bold = config.get('display', 'id_font_bold')
        self._text.setFont(QFont('', rect.width()*0.08*scale))
        self._text.setPos(rect.x(), rect.y() + 0.2*rect.height())
        self._text.setDefaultTextColor(QColor(config.get('display', 'id_font_color')))
        rect2 = QRectF()

        if self._display_mode == 0:
            if bold:
                self._text.setHtml('<b>R(' + str(self._id) + ')</b>')
            else:
                self._text.setHtml('R(' + str(self._id) + ')')
            if not self._highlighted:
                rect2 = QRectF(rect.x() + 0.25*rect.width(), 
                               rect.y() + 0.25*rect.height(),
                               rect.width()*0.5,
                               rect.height()*0.5)
            else:
                rect2 = QRectF(rect.x() + 0.05*rect.width(), 
                               rect.y() + 0.05*rect.height(),
                               rect.width()*0.9,
                               rect.height()*0.9,)

        else:
            self._text.setPlainText('')
            if not self._highlighted:
                rect2 = QRectF(rect.x() + 0.05*rect.width(), 
                               rect.y() + 0.05*rect.height(),
                               rect.width()*0.9,
                               rect.height()*0.9)
            else:
                rect2 = QRectF(rect.x() - 0.15*rect.width(), 
                               rect.y() - 0.15*rect.height(),
                               rect.width()*1.3,
                               rect.height()*1.3,)

        self._graphics_item.setRect(rect2)
        #draw energy bar if max energy > 0
        if self._max_energy > 0:
            per_energy = max(0.0, min(1.0, float(self._current_energy) / self._max_energy))
            rect2.setLeft(rect2.x() + rect2.width()*0.7)
            rect2.setWidth(rect2.width()*0.5)

            self._energy_bar_empty.setRect(rect2.x(), 
                                           rect2.y(),
                                           rect2.width(),
                                           rect2.height() * (1.0 - per_energy))
            self._energy_bar_full.setRect(rect2.x(), 
                                          rect2.y() + rect2.height() * (1.0 - per_energy), 
                                          rect2.width(), 
                                          rect2.height() * per_energy)

        if self._carries is not None:
            self._carries.set_rect(rect)
        self.update_tooltip()

    def set_initial_energy(self, energy):
        """
        Sets the initial energy of the robot.
        """

        self._initial_energy = energy

    def set_action(self, action, time_step):
        """
        Sets the action for the specific time step.
        Overrides existing actions at the time step but prints out
        a warning since this should never happen. Accepts deliver
        actions simultaneously to other actions.

        Parameters:
        action: clingo.symbol.Symbol
            This is the action that will be performed.
        time_step: int
            This is the time step at which the action
        shold be performed.
        """

        if time_step < 0:
            print(('Warning: invalid time step in occurs(object('
                    + str(self._kind_name) + ','
                    + str(self._ID) + '),' + str(action)
                    + ',' + str(time_step) + ')'))
            print('time step is less than 0')
            return
        for ii in range((time_step + 1) - len(self._actions)):
            self._actions.append(None)

        action_name = action.arguments[0].name

        if action_name == 'deliver' and time_step in self._parallel_delivers:
            deliver_list = self._parallel_delivers[time_step]
            deliver_list.append(action)
            return

        elif self._actions[time_step] is not None:

            action_name2 = self._actions[time_step].arguments[0].name

            if action_name == 'deliver':
                if not ll_config.get('features', 'domainc'):
                    print(('Warning: for object(' + str(self._kind_name)
                        + ', ' + str(self._id)
                        + ') multiple actions are defined at time step '
                        + str(time_step)))

                if not time_step in self._parallel_delivers:
                    self._parallel_delivers[time_step] = []
                self._parallel_delivers[time_step].append(action)
                if action_name2 == 'deliver':
                    self._parallel_delivers[time_step].append(
                        self._actions[time_step])
                    self._actions[time_step] = None
                return

            elif action_name2 == 'deliver':
                if not ll_config.get('features', 'domainc'):
                    print(('Warning: for object(' + str(self._kind_name)
                        + ', ' + str(self._id)
                        + ') multiple actions are defined at time step '
                        + str(time_step)))

                if not time_step in self._parallel_delivers:
                    self._parallel_delivers[time_step] = []
                self._parallel_delivers[time_step].append(
                    self._actions[time_step])
                self._actions[time_step] = action
                return

            else:
                print(('Warning: for object(' + str(self._kind_name)
                    + ', ' + str(self._id)
                    + ') multiple actions are defined at time step '
                    + str(time_step)))
        self._actions[time_step] = action

    def set_current_energy(self, energy):
        """
        Sets the current energy of the robot.
        """

        self._current_energy = energy

    def set_max_energy(self, energy):
        """
        Sets the maximal energy of the robot.
        """

        self._max_energy = energy

    def add_energy(self, energy):
        """
        Adds energy to or removes energy from the robot.
        """

        self._current_energy = self._current_energy + energy

    def add_task(self, task):
        """
        Adds a task to the robot.
        """

        if task is None:
            return
        if task in self._tasks:
            return
        self._tasks.append(task)
        task.set_robot(self)

    def parse_init_value(self, name, value):
        """
        This function handels the input phrases for robots.
        This is called for every phrase the model receives with the
        following syntax: 
        init(object(robot, [robot ID]), value([value name], [value])).

        Parameters:
        name: str
            This is the name of the value.
        value: clingo.symbol.Symbol
            This is the actual value. It contains the [value]
            part of the phrase.        

        Returns 1 if the phrase cannot be parsed, -1 if one parameter is
        invalid and 0 if the function succeeded.   
        """

        result = super(self.__class__, self).parse_init_value(name, value)
        if result <= 0: return result
        if name == 'carries':
            shelf = self._model.get_item('shelf', value, True, True)
            self.set_initial_carries(shelf)
            self.set_carries(shelf)
            return 0
        elif name == 'energy':
            self.set_current_energy(value.number)
            self.set_initial_energy(value.number)
            return 0
        elif name == 'max_energy':
            self.set_max_energy(value.number)
            return 0
        elif name == 'energy_cost':
            self._energy_costs[value.arguments[0].name] = value.arguments[1].number
            return 0
        return 1

    def update_tooltip(self):
        """
        Updates the tooltip for the robot.
        """

        tooltip = ("robot(" + str(self._id) + ")\nenergy: " + 
                   str(self._current_energy) + "/" + 
                   str(self._max_energy))
        if self._max_energy > 0:
            tooltip += ("(" + 
                        str(float(self._current_energy)/self._max_energy*100.0) + 
                        "%)")
        
        self.setToolTip(tooltip)

    def restart(self):
        """
        Resets the energy and the carried shelf of the robot.
        Sets the robot to its starting position.
        """

        super(self.__class__, self).restart()
        self.set_carries(self._initial_carries)
        self.set_current_energy(self._initial_energy)

    def clear_actions(self):
        """
        Deletes all actions for an object.
        """

        self._actions = []
        self._parallel_delivers = {}

    def to_init_str(self):
        """
        Converts the robot to a string that represents the robots values.
        This function is used to send robots to a solver
        and to save robots to a file.
        """

        s = super(self.__class__, self).to_init_str()
        if self._initial_carries is not None:
            s += ("init(object(robot,"
                    + str(self._id) + "),value(carries,"
                    + str(self._initial_carries.get_id())
                    + ")).")
        s += ("init(object(robot," 
                + str(self._id) + "), value(max_energy,"
                + str(self._max_energy)
                + ")).")
        s += ("init(object(robot," 
                + str(self._id) + "), value(energy,"
                + str(self._current_energy)
                + ")).")
        for key in self._energy_costs:
            s += ("init(object(robot,"
                    + str(self._id) + "),value(energy_cost, ("
                    + key + ", " + str(self._energy_costs[key]) + "))).")
        return s

    def to_occurs_str(self):
        """
        This function returns a list of strings that represents all
        actions of a robot.        
        """

        actions = []
        count = 0
        for action in self._actions:
            str_out = ''
            if action is None and count not in self._parallel_delivers:
                actions.append(None)
            else:
                str_out = ''
                if action is not None:
                    str_out = ('occurs(object('
                        + self._kind_name + ', '
                        + str(self._id) + '), '
                        + str(action) + ', '
                        + str(count) + ').\n')
                if count in self._parallel_delivers:
                    for action2 in self._parallel_delivers[count]:
                        str_out += ('occurs(object('
                            + self._kind_name + ', '
                            + str(self._id) + '), '
                            + str(action2) + ', '
                            + str(count) + ').\n')
                actions.append(str_out)
            count = count + 1
        return actions

    def _do_deliver(self, action, value, time_step):
        """
        Performs the given deliver action.

        Parameters:
        action: clingo.symbol.Symbol
            This symbol contains the name of the action.
        value: clingo.symbol.Symbol
            This symbol contains the values of the
            deliver action.
        """
        #deliver with 3 arguments
        if len(value.arguments) > 2:
            try:
                if self._carries is not None:
                    self._carries.remove_product(value.arguments[1], value.arguments[2].number)
                order = self._model.filter_items(item_kind = 'order', 
                            ID = value.arguments[0], 
                            return_first = True)[0]
                if order is None:
                    return -2
                order.deliver(value.arguments[1], value.arguments[2].number, time_step)
                self._state = self._state | VIZ_STATE_DELIVERED
            except:
                return -3
            return 4

        #deliver with 2 arguments
        elif len(value.arguments) > 1:
            try:
                if self._carries is not None:
                    self._carries.remove_product(value.arguments[1], 0)
                order = self._model.filter_items(item_kind = 'order', 
                            ID = value.arguments[0], 
                            return_first = True)[0]
                if order is None:
                    return -2
                order.deliver(value.arguments[1], 0, time_step)
                self._state = self._state | VIZ_STATE_DELIVERED
            except:
                return -3
            return 5
        return 0

    def _undo_deliver(self, action, value, time_step):
        """
        Performs the given deliver action backwards.

        Parameters:
        action: clingo.symbol.Symbol
            This symbol contains the name of the action.
        value: clingo.symbol.Symbol
            This symbol contains the values of the
            deliver action.
        """

        #deliver with 3 arguments
        if len(value.arguments) > 2:
            try:
                if self._carries is not None:
                    self._carries.remove_product(value.arguments[1], -value.arguments[2].number)
                order = self._model.filter_items(item_kind = 'order', 
                            ID = value.arguments[0], 
                            return_first = True)[0]
                if order is None:
                    return -2
                order.deliver(value.arguments[1], -value.arguments[2].number, time_step)
                self._state = self._state | VIZ_STATE_DELIVERED
            except:
                return -3
            return 4

        #deliver with 2 arguments
        elif len(value.arguments) > 1:
            try:
                if self._carries is not None:
                    self._carries.remove_product(value.arguments[1], 0)
                order = self._model.filter_items(item_kind = 'order', 
                            ID = value.arguments[0], 
                            return_first = True)[0]
                if order is None:
                    return -2
                order.deliver(value.arguments[1], 0, time_step)
                self._state = self._state | VIZ_STATE_DELIVERED
            except:
                return -3
            return 5
        return 0

    def do_action(self, time_step):
        """
        Performs the actions of the robot.
        This function will be called every time the model does
        one time step forward.
        Returns 0 if no action was done.
        Returns a negative integer if an error occurred.
        Returns a positive integer if an action was done.
        """

        self._state = self._state & ~VIZ_STATE_ACTION

        #sets the state for the next action
        if time_step + 1 < len(self._actions):
            if self._actions[time_step + 1] is not None:
                action = self._actions[time_step + 1].arguments[0]
                if action.name == 'move':
                    self._state = self._state | VIZ_STATE_MOVE
                elif action.name == 'deliver':
                    self._state = self._state | VIZ_STATE_DELIVER
                elif action.name == 'pickup':
                    self._state = self._state | VIZ_STATE_PICK_UP
                elif action.name == 'putdown':
                    self._state = self._state | VIZ_STATE_PUT_DOWN2
                elif action.name == 'charge':
                    self._state = self._state | VIZ_STATE_CHARGE


        if time_step >= len(self._actions):
            return 0  #break, if no action is defined
        if self._actions[time_step] is None:
            return 0  #break, if no action is defined
        if self._model is None:
            return -3

        try:
            action = self._actions[time_step].arguments[0]
            value = self._actions[time_step].arguments[1]
        except:
            return -1

        #energy cost
        if str(action.name) in self._energy_costs:
            try:
                self.add_energy(-self._energy_costs[action.name])
            except:
                print("Energy cost is not an integer.")
                self._energy_costs[action.name] = None

        #do actions
        #do deliveries
        if time_step in self._parallel_delivers:
            for action2 in self._parallel_delivers[time_step]:
                self._do_deliver(
                    action2.arguments[0], 
                    action2.arguments[1], 
                    time_step)

        #move
        if action.name == 'move':
            if len(value.arguments) != 2: 
                return -1
            try:
                move_x = value.arguments[0].number
                move_y = value.arguments[1].number
                self.set_position(self._position[0] + move_x, self._position[1] + move_y)
                self._state = self._state | VIZ_STATE_MOVED
            except:
                self.set_position(self._position[0], self._position[1])

            for task in self._tasks:
                for checkpoint in task.get_checkpoints():
                    checkpoint = checkpoint[0]
                    pos = checkpoint.get_position()
                    if pos[0] == self._position[0] and pos[1] == self._position[1]:
                        task.visit_checkpoint(checkpoint)
                        break
            return 1

        #pick up
        elif action.name == 'pickup':
            shelf = self._model.filter_items(item_kind = 'shelf',
                        position = self._position,
                        return_first = True)[0]
            if shelf is None:
                return -2
            self.set_carries(shelf)
            self._state = self._state | VIZ_STATE_PICKED_UP
            return 2

        #put down
        elif action.name == 'putdown':
            if self._carries == None:
                return -2
            self.set_carries(None)
            self._state = self._state | VIZ_STATE_PUT_DOWN
            return 3

        #deliver
        elif action.name == 'deliver':
            self._do_deliver(action, value, time_step)
            return 4

        #charge
        elif action.name == 'charge':
            try:
                self.add_energy(value.number)
            except:
                return -4
            return 6
        return 0

    def undo_action(self, time_step):
        """
        Performs the actions of the robot.
        This function will be called every time the model does
        one time step backwards.
        Returns 0 if no action was done.
        Returns a negative integer if an error occurred.
        Returns a positive integer if an action was done.
        """

        self._state = self._state & ~VIZ_STATE_ACTION
        if time_step >= len(self._actions):  
            return 0  #break, if no action is defined
        if self._actions[time_step] == None: 
            return 0  #break, if no action is defined
        if self._model is None:
            return -3

        try:
            action = self._actions[time_step].arguments[0]
            value = self._actions[time_step].arguments[1]
        except:
            return -1

        #energy cost
        if action.name in self._energy_costs:
            try:
                self.add_energy(self._energy_costs[action.name])
            except:
                print("Energy cost is not an integer.")
                self._energy_costs[action.name] = None

        #undo actions
        #undo deliveries
        if time_step in self._parallel_delivers:
            for action2 in self._parallel_delivers[time_step]:
                self._undo_deliver(
                    action2.arguments[0], 
                    action2.arguments[1], 
                    time_step)

        #move
        if action.name == 'move':
            if len(value.arguments) != 2: 
                return -1

            for task in self._tasks:
                for checkpoint in task.get_checkpoints():
                    checkpoint = checkpoint[0]
                    pos = checkpoint.get_position()
                    if pos[0] == self._position[0] and pos[1] == self._position[1]:
                        task.unvisit_checkpoint(checkpoint)
                        break

            try:
                move_x = value.arguments[0].number
                move_y = value.arguments[1].number
                self.set_position(self._position[0] - move_x, self._position[1] - move_y)
                self._state = self._state | VIZ_STATE_MOVE
            except:
                self.set_position(self._position[0], self._position[1])
            if self._carries is not None: 
                self._carries.set_position(self._position[0], self._position[1])
            return 1

        #put down
        elif action.name == 'putdown':
            shelf = self._model.filter_items(item_kind = 'shelf',
                        position = self._position,
                        return_first = True)[0]
            if shelf is None:
                return -2
            self.set_carries(shelf)
            self._state = self._state | VIZ_STATE_PUT_DOWN2
            return 3

        #pick up
        elif action.name == 'pickup':
            if self._carries == None:
                return -2
            self.set_carries(None)
            self._state = self._state | VIZ_STATE_PICK_UP
            return 2

        #deliver
        elif action.name == 'deliver':
            self._undo_deliver(action, value, time_step)
            return 4
            

        #charge
        elif action.name == 'charge':
            try:
                self.add_energy(-value.number)
            except:
                return -4
            return 6
        return 0

    
    def determine_color(self, number, count, pattern = None):
        """
        Determines the color of the robot.
        """

        color = calculate_color(self._colors[0], self._colors[1], (float)(number)/count)
        self._color = color
        brush = QBrush(color)
        self._graphics_item.setBrush(brush)

    def get_rect(self):
        """
        Returns the current rectangle of the robot.
        """

        if self._display_mode == 0:
            rect = self._graphics_item.rect()
            width = rect.width()*2
            height = rect.height()*2
            rect.setLeft(rect.x() - 0.25*width)
            rect.setTop(rect.y() - 0.25*height)
            rect.setWidth(width)
            rect.setHeight(height)
            return rect
        elif self._display_mode == 1:
            rect = self._graphics_item.rect()
            width = rect.width()/0.9
            height = rect.height()/0.9
            rect.setLeft(rect.x() - 0.05*width)
            rect.setTop(rect.y() - 0.05*height)
            rect.setWidth(width)
            rect.setHeight(height)
            return rect

    def get_carries(self):
        """
        Returns the shelf that is carried by this robot.
        This value can be None.
        """

        return self._carries

    def get_initial_carries(self):
        """
        Returns the shelf that is carried by this robot at
        time step 0. This value can be None.
        """

        return self._initial_carries

    def edit_position_to(self, x, y):
        """
        Sets a new starting position and new current position for the 
        robot. If another robot is already on the same position
        they swap positions. This function is used to edit instances with
        the drag and drop feature. If on the destination node is a
        carried shelf or this robot carries a shelf and the destination
        node is occupied by another shelf the shelfs switch places as well.
        """

        if (x, y) == self._position:
            return
        item2 = self._model.filter_items(item_kind = self._kind_name,
                                        position = (x,y),
                                        return_first = True)[0]
        shelf = self._model.filter_items('shelf',
                                        position = self._position,
                                        return_first = True)[0]
        shelf2 = self._model.filter_items('shelf',
                                        position = (x,y),
                                        return_first = True)[0]

        if shelf2 is not None and shelf is not None:
            if shelf.get_carried() is not None or shelf2.get_carried() is not None:
                shelf.set_position(x,y)
                shelf.set_starting_position(x,y)
                shelf2.set_position(self._position[0], self._position[1])
                shelf2.set_starting_position(self._position, self._position[1])
        if item2 is not None:
            item2.set_position(self._position[0], self._position[1])
            item2.set_starting_position(self._position[0], self._position[1])
        self.set_position(x,y)
        self.set_starting_position(x, y)

    def get_current_energy(self):
        """
        Returns the current energy of the robot.
        """

        return self._current_energy

    def get_max_energy(self):
        """
        Returns the maximal energy of the robot.
        """

        return self._max_energy

'''