from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from mapf_simulation.configuration import *
from mapf_simulation.model import *
import time

# Different possible states of ModelView
PUBLISH  = 0
'''
= state of the simulation at the begin of every cycle. Here the simulation publishes all information 
(1xmapf_interfaces/MapInfo.msg and for every robot 1xmapf_interfaces/RobotInfo.msg).
'''
WAIT = 1
'''
= state of the simulation after it has published all information and is waiting until 
it has received all information from the CONTROLLER   
(for every robot 1xmapf_interfaces/RobotAction.msg).
'''
EXEC = 2
'''
= state of the simulation after it has received all information from the CONTROLLER. 
Here the simulation executes the given actions for every robot.
'''

class MainScene(QGraphicsScene):
    def __init__(self):
        super(self.__class__, self).__init__()

class ModelView(QGraphicsView):
    '''
    This class is responsible for the visualization the simulation.

    To update the simulation a timer is created. 
    After the time is up the window is updated with the new positions of the items(robots) in the map. 
    Items are moved to their new position step by step.   
    For a complete execution of the action for a robot per default 10 timers are created. So every
    time the timer is up the robots are moved incrementally (around 10%).  
    After the running out of the 10 timers the robots have reached their new final position. Then the model 
    is updated with the new positions of the robots.
    '''
    def __init__(self):
        super(self.__class__, self).__init__()
        self._scene = MainScene()
        self.setScene(self._scene)

        self._model = None
        self._windows = []

        self._mode  = PUBLISH
        
        self._object_size = 35
        self._screen_width  = 600
        self._screen_height = 600
        self._border_size = 3             #the size of the borders of the grid

        self._h_distance = 60             #the size of one grid cell
        self._w_distance = 60

        self._line_hlength = self._w_distance
        self._line_vlength = self._h_distance

        self._display_mode = 0
        self._scaling = 1.0
        self._zoom = (1.0, 1.0)

        self._lines = []

        self._items_in_scene = []

        self._timer = None
        self._timer_count = 0
        self._timer_scale = 1
        self._step_time, self._timer_count_max = self.get_step_time()  #2000.0


        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setContextMenuPolicy(Qt.DefaultContextMenu)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.resize(self._screen_width, self._screen_height)
        self._menu = QMenu()
        self._menu.setParent(self)
        self.setToolTip('tooltip')
        

    def set_model(self, model):
        if self._model is not None:
            self._model.remove_window(self)
        self._model = model
        if self._model is not None:
            self._model.add_window(self)
        self.resize_to_fit()
        self.update()

    def get_model(self):
        return self._model


    #EVENTS
    def wheelEvent(self, event):
        self.zoom_event(event.angleDelta().y())

    def zoom_event(self, angle):
        zoom = (1.0 + angle/1440.0)
        self._scale(zoom, zoom)
        
    def resizeEvent(self, event):
        super(self.__class__, self).resizeEvent(event)
        if (event.oldSize().width() < 1 
            or event.oldSize().height() < 1 
            or event.size().width() < 1 
            or event.size().height() < 1):
            return
        self._screen_width  = event.size().width()
        self._screen_height = event.size().height()
        scaling = ((float)(self._screen_width) / (event.oldSize().width()), 
                    (float)(self._screen_height) / (event.oldSize().height()))
        self._scale(scaling[0], scaling[1])

    def resize_to_fit(self):
        if self.width() <= 1 or self.height() <= 1: 
            return
        zoom = ((float)(self._screen_width) / (self._line_hlength*1.04), 
                (float)(self._screen_height) / (self._line_vlength*1.04))

        self._scale(zoom[0]/self._zoom[0], zoom[1]/self._zoom[1])
        self._scene.setSceneRect(0, 0, self._line_hlength, self._line_vlength*1)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton or event.button() == Qt.MiddleButton:
            self._menu.hide()
        super(self.__class__, self).mousePressEvent(event)

    def event(self, event):
        if event.type() == QEvent.ToolTip:
            pos = self.mapToScene(event.pos())
            pos = self.scene_coordinates_to_node(pos.x(), pos.y())
            if pos is None:
                self.setToolTip('')
                return super(self.__class__, self).event(event)
            ss = 'node(' + str(self._model.get_node_id(pos)) +') at ' + str(pos)
            for item in self._model.filter_items(position = pos):
                ss += '\n' + item.get_name() + '('+ str(item.get_id()) +')'
            self.setToolTip(ss)
        return super(self.__class__, self).event(event)


    #CONTEXTMENU
    def contextMenuEvent(self, event):
        '''
        This function creates the contextmenu by right click in the simulation window.
        For the moment only two options defined: "enable node" and "disable node"
        '''
        if self._model == None:
            return

        point = self.mapToScene(event.x(), event.y())
        node = self.scene_coordinates_to_grid(point.x(), point.y())
        if node is None:
            return
        x = node[0]
        y = node[1]

        if not self._model.get_editable():
            return 

        self._menu.clear() 


        if self._model.is_node(x,y):
            action = QAction('disable node', self)
            action.setShortcut('Ctrl + N')
            action.setStatusTip('Disables the selected node')
            action.triggered.connect(lambda: self._remove_node(x, y))
            self._menu.addAction(action)   
        else:
            action = QAction('enable node', self)
            action.setShortcut('Ctrl + N')
            action.setStatusTip('Enables the selected node')
            action.triggered.connect(lambda: self._add_node(x, y))
            self._menu.addAction(action) 
        
        '''
        robot = self._model.filter_items('robot_ros', position = (x,y), return_first = True)[0]
        if robot is not None:
            action = QAction('remove robot', self)
            action.setShortcut('Ctrl + R')
            action.setStatusTip('Removes a robot from the selected node')
            action.triggered.connect(lambda: self._remove_item(robot))
            self._menu.addAction(action)
        elif self._model.is_node(x,y):
            action = QAction('add robot', self)
            action.setShortcut('Ctrl + R')
            action.setStatusTip('Adds a robot to the selected node')
            action.triggered.connect(lambda: self._add_item('robot_ros', x, y))
            self._menu.addAction(action)
        '''
        self._menu.popup(QPoint(event.x(),event.y()))

    def _add_node(self, x, y):
        '''
        This function adds a specific node (a cell in the map with x,y coordinates) to the model. 
        The function is used by the context menu to enable a node.
        '''
        self._model.add_node(x, y)
        self._menu.hide()
        self._model.update_windows()
    
    def _remove_node(self, x, y):
        '''
        This function removes a specific node (a cell in the map with x,y coordinates) from the model. 
        The function is used by the context menu to disable a node.
        '''
        self._model.remove_node(x, y)
        self._menu.hide()
        self._model.update_windows()

    def _add_item(self, kind, x, y):
        '''
        This function adds a specific item (robot or checkpoint) to the model. 
        The function could be used by the context menu and is not used at the moment.
        '''
        item = self._model.create_item(kind, add_immediately = True)
        item.set_starting_position(x, y)
        item.set_position(x, y)
        self._menu.hide()
        self._model.update_windows()
    
    def _remove_item(self, item):
        '''
        This function removes a specific item (robot or checkpoint) from the model. 
        The function could be used by the context menu and is not used at the moment.
        '''
        self._model.remove_item(item)
        self._menu.hide()
        self._model.update_windows()


    #TIMER
    def start_timer(self):
        '''
        This function stops the previous timer and starts a new one. 
        '''
        self.stop_timer()
        self._timer = QTimer()
        self._timer.timeout.connect(lambda: self.update_model(False))
        self.adjust_timer()
        self._mode = PUBLISH
        
    def adjust_timer(self):
        '''
        This function sets the timeout for the timer.   

        Timeout = time to execute the complete step / number of intermediate steps  

        Example: the duration of one complete step schould be 1 sec. with 10 intermediate steps. 
        Then every timer should run for = 1sec/10 = 0.1seconds.
        '''
        timeout = self._step_time / self._timer_count_max
        if timeout < 10:
            timeout = 10
        if self._timer is not None:
            self._timer.start(timeout)

    def switch_timer(self):
        '''
        This function switches the timer  
        (start the timer if there is no timer running / stop the timer if there is a timer running).
        '''
        if self._timer is None:
            self.start_timer()
        else:
            self.stop_timer()

    def stop_timer(self):
        '''
        This function stops the timer if there is a timer running.
        '''
        if self._timer is not None:
            self._timer.stop()
            self._timer_count = 0
            self._timer = None

    def speed_up_timer(self):
        '''
        This function decreases the time for completion of one complete step.

        Following values are allowed (in milliseconds):  
        2000 (intermediate steps=20), 1000 (10), 500 (10), 250(5), 0(0)

        By decreasing the time the value is moved from left to right  
        (example: old value 500, new value 250)
        
        The timeout for one timer is defined by "time for completion of one step" / "intermediate steps"
        (example: value 2000, then 20 intermediate steps with every timer running for 100 millisec.)
        '''
        values_time =  [2000, 1000, 500, 250, 0]
        values_count = [20,   10,   10,  5,   1]
        
        self._timer_scale += 1
        if self._timer_scale > len(values_time):
            self._timer_scale = len(values_time) 
        elif self._timer_scale <= 1:
            self._timer_scale = 2

        self._step_time = values_time[self._timer_scale-1]
        self._timer_count_max = values_count[self._timer_scale-1]
        self.adjust_timer()

    def speed_down_timer(self):
        '''
        This function increaes the time for completion of one complete step.

        Following values are allowed (in milliseconds):  
        2000 (intermediate steps=20), 1000 (10), 500 (10), 250(5), 0(0)

        By increasing the time the value is moved from right to left  
        (example: old value 250, new value 500)
        
        The timeout for one timer is defined by "time for completion of one step" / "intermediate steps"
        (example: value 250, then 5 intermediate steps with every timer running for 50 millisec.)
        '''
        values_time =  [2000, 1000, 500, 250, 0]
        values_count = [20,   10,   10,  5,   1]
        
        self._timer_scale -= 1
        if self._timer_scale >= len(values_time):
            self._timer_scale = len(values_time)-1
        elif self._timer_scale < 1:
            self._timer_scale = 1

        self._step_time = values_time[self._timer_scale-1]
        self._timer_count_max = values_count[self._timer_scale-1]
        self.adjust_timer()

    def is_timer_running(self):
        '''
        This function checks whether the timer is running.

        :return: **True**, if a timer is running   
        **False**, otherwise
        '''
        return self._timer is None

    def get_timer_scale(self):
        '''
        @private
        '''
        return self._timer_scale

    def get_step_time(self):
        '''
        @private
        '''
        values_time =  [2000, 1000, 500, 250, 0]
        values_count = [20,   10,   10,  5,   1]
        return values_time[self._timer_scale-1], values_count[self._timer_scale-1]


    
    #UPDATE WINDOW
    def update_windows(self):
        '''
        This function updates all registered windows. 
        '''
        for window in self._windows:
            window.update()
    
    def add_window(self, window):
        '''
        This function adds a widget to the list of widgets which should be updated every time the function update_windows(self) is called.
        This could be the robot table (RightSideWidget) or widgets from the left side.
        This widgets can then be updated all together if the model view has changed. 

        :param window: PyQt5.QtWidgets.QWidget - widget which should be added to the list
        
        '''
        if window not in self._windows:
            self._windows.append(window)

    def update(self):
        '''
        This function draws the map, the items and other elements in the window. 

        Following elements will be plotted:
        -   vertical and horizontal lines
        -   disabled nodes (=blocked spots)
        -   every graphic item in the model

        Following elements are optional and will be plotted only if activated by the user:
        -   start points of the items
        -   driven path by the items
        -   future/calculated path for the items
        '''

        if self._model == None: 
            return 0
        if self._model.mapf_get_filename_map() == '':
            return 0

        self._line_hlength = (self._w_distance) * self._model.get_grid_size()[0] + self._border_size
        self._line_vlength = (self._h_distance) * self._model.get_grid_size()[1] + self._border_size

        for item_dic in self._model.iterate_graphic_dictionaries():
            for item in item_dic.values():
                color_id = 0
                hex_color = config.get('color', 'color_' + item.get_kind_name() + str(color_id))
                while(hex_color is not None):
                    item.set_color(QColor(hex_color), color_id)
                    color_id += 1
                    hex_color = config.get('color', 'color_' + item.get_kind_name() + str(color_id))

        pen = QPen()
        pen.setWidth(self._border_size)
        pen.setStyle(Qt.DotLine)
        brush_disabled_node = QBrush(QColor(config.get('color', 'color_disabled_node')))
        brush_start_point = QBrush(QColor(config.get('color', 'color_start_node')))
        brush_start_point_highlighted = QBrush(QColor(config.get('color', 'color_start_node_highlighted')))
        color_travelled_path_highlighted = QColor(config.get('color', 'color_travelled_path_highlighted'))
        color_future_path_highlighted = QColor(config.get('color', 'color_future_path_highlighted'))

        self.clear()

        #---------------------------------------draw vertival lines---------------------------------------
        for i in range(0,self._model.get_grid_size()[1] + 1):
            line = self._scene.addLine(0, i * self._h_distance*self._scaling,
                                        self._line_hlength*self._scaling - self._border_size,
                                        i *(self._h_distance*self._scaling), pen)
            self._lines.append(line)

        #---------------------------------------draw horizontal lines---------------------------------------
        for i in range(0,self._model.get_grid_size()[0] + 1):
            line = self._scene.addLine(i * self._w_distance*self._scaling, 
                                        0, i * (self._w_distance*self._scaling), 
                                        self._line_vlength*self._scaling - self._border_size, pen)
            self._lines.append(line)

        #---------------------------------------draw disabled nodes---------------------------------------
        for node in self._model.get_blocked_nodes():
            xPos = (node[0]-1) * self._w_distance
            yPos = (node[1]-1) * self._h_distance
            
            rect = self._scene.addRect( (xPos+self._w_distance*0.1)*self._scaling, 
                                        (yPos+self._h_distance*0.1)*self._scaling, 
                                        (self._w_distance*0.8)*self._scaling, 
                                        (self._h_distance*0.8)*self._scaling, 
                                        pen, brush_disabled_node)
            '''
            rect = self._scene.addRect( xPos*self._scaling, 
                                        yPos*self._scaling, 
                                        self._w_distance*self._scaling, 
                                        self._h_distance*self._scaling, 
                                        pen, brush_disabled_node)
            '''
            self._items_in_scene.append(rect)

        #---------------------------------------draw items---------------------------------------
        for item_dic in self._model.iterate_graphic_dictionaries():
            count = len(item_dic)
            number = 1
            for item in item_dic.values():
                item.set_display_mode(self._display_mode)
                #item.set_display_mode(0)
                x_pos = item.get_position()[0] - 1
                y_pos = item.get_position()[1] - 1
                action = item.get_action(self._model.get_current_step())
                if action is not None:
                    action_name = action.arguments[0].name
                    action_value = action.arguments[1]
                    if action_name == 'move':
                        action_x = action_value.arguments[0].number
                        action_y = action_value.arguments[1].number
                        x_pos = float(x_pos + float(action_x * float(self._timer_count / float(self._timer_count_max))))
                        y_pos = float(y_pos + float(action_y * float(self._timer_count / float(self._timer_count_max))))
                x_pos = x_pos*self._w_distance*self._scaling + self._border_size/2
                y_pos = y_pos*self._h_distance*self._scaling + self._border_size/2
                item.set_rect(QRect(x_pos, y_pos,
                                self._w_distance*self._scaling - self._border_size*self._scaling,
                                self._h_distance*self._scaling - self._border_size*self._scaling))
                
                item.determine_color(number, count)
                
                #---------------------------------------draw start point---------------------------------------
                if item.get_draw_startpoint():

                    w = self._w_distance*self._scaling*0.5
                    h = self._h_distance*self._scaling*0.5
                                           
                    x_pos = item.get_start_position()[0]-1
                    y_pos = item.get_start_position()[1]-1
                    x_pos = float(x_pos)*self._w_distance*self._scaling + 0.5*self._w_distance*self._scaling
                    y_pos = float(y_pos)*self._h_distance*self._scaling + 0.5*self._h_distance*self._scaling

                    pen = QPen(item.get_color())
                    if item.get_highlighted():
                        startpoint = self._scene.addEllipse(x_pos-0.75*w, 
                                                  y_pos-0.75*h,
                                                  1.5*w,
                                                  1.5*h,
                                                  pen, brush_start_point_highlighted)
                        self._items_in_scene.append(startpoint)

                    else:
                        startpoint = self._scene.addEllipse(x_pos-w/2.0, 
                                                  y_pos-h/2.0,
                                                  w,
                                                  h,
                                                  pen, brush_start_point)
                        self._items_in_scene.append(startpoint)
                                 
                #---------------------------------------draw path---------------------------------------
                if item.get_draw_path():

                    x_pos = item.get_start_position()[0]
                    y_pos = item.get_start_position()[1]
                    pen = QPen(item.get_color())
                    pen.setWidth(self._border_size*2)
                    if item.get_highlighted():
                        pen = QPen(color_travelled_path_highlighted)      
                        pen.setWidth(self._border_size*4)

                    if number % 4 == 0:
                        pen.setStyle(Qt.DotLine)
                    elif number % 3 == 0:
                        pen.setStyle(Qt.DashDotDotLine)
                    elif number % 2 == 0:
                        pen.setStyle(Qt.DashDotLine)           
                    else:
                        pen.setStyle(Qt.DashLine)           

                    cc = 0
                    while(cc <= self._model.get_num_steps()):
                        action = item.get_action(cc)
                        if action is not None:
                            
                            if action.arguments[0].name == 'move':
                                action_value = action.arguments[1]
                                
                                x_posf = (x_pos-0.5)*self._w_distance*self._scaling
                                y_posf = (y_pos-0.5)*self._h_distance*self._scaling
                                x2_posf = (x_pos+action_value.arguments[0].number-0.5)*self._w_distance*self._scaling
                                y2_posf = (y_pos+action_value.arguments[1].number-0.5)*self._h_distance*self._scaling
                                
                                if (cc == ( self._model.get_num_steps()) and self._timer_count != 0):

                                    x2_posf = (x_pos+action_value.arguments[0].number*float(self._timer_count / float(self._timer_count_max))
                                            -0.5)*self._w_distance*self._scaling
                                    y2_posf = (y_pos+action_value.arguments[1].number*float(self._timer_count / float(self._timer_count_max))
                                            -0.5)*self._h_distance*self._scaling
                                
                                x_pos = x_pos + action_value.arguments[0].number
                                y_pos = y_pos + action_value.arguments[1].number

                                line = self._scene.addLine(x_posf, y_posf, x2_posf, y2_posf, pen)
                                self._items_in_scene.append(line)
                        cc = cc + 1
                        
                #---------------------------------------draw future path---------------------------------------
                if item.get_draw_future_path():
                    if (len(item.get_future_path()) > 1):
                        x_pos = item.get_future_path()[0].pos_x
                        y_pos = item.get_future_path()[0].pos_y

                        pen = QPen(item.get_color())
                        pen.setWidth(self._border_size)
                        if item.get_highlighted():
                            pen = QPen(color_future_path_highlighted)      
                            pen.setWidth(self._border_size*3)

                        if number % 4 == 0:
                            pen.setStyle(Qt.DotLine)
                        elif number % 3 == 0:
                            pen.setStyle(Qt.DashDotDotLine)
                        elif number % 2 == 0:
                            pen.setStyle(Qt.DashDotLine)           
                        else:
                            pen.setStyle(Qt.DashLine)           

                        first = True
                        for position in item.get_future_path()[1:]:

                            if position is None:
                                break

                            x2_posf = (position.pos_x-0.5)*self._w_distance*self._scaling
                            y2_posf = (position.pos_y-0.5)*self._h_distance*self._scaling
                        
                            if first:
                                first = False
                                if self._timer_count != 0:
                                    x_posf = (x_pos-0.5+(position.pos_x-x_pos)*float(self._timer_count / float(self._timer_count_max)
                                                                                        ))*self._w_distance*self._scaling
                                    y_posf = (y_pos-0.5+(position.pos_y-y_pos)*float(self._timer_count / float(self._timer_count_max)
                                                                                        ))*self._h_distance*self._scaling
                                else:
                                    x_posf = x2_posf
                                    y_posf = y2_posf
                                
                            else:
                                x_posf = (x_pos-0.5)*self._w_distance*self._scaling
                                y_posf = (y_pos-0.5)*self._h_distance*self._scaling

                            x_pos = position.pos_x
                            y_pos = position.pos_y
                            
                            line = self._scene.addLine(x_posf, y_posf, x2_posf, y2_posf, pen)
                            self._items_in_scene.append(line)
                            first = False

                self._scene.addItem(item)
                self._items_in_scene.append(item)
                number += 1
                
        self.update_windows()

    def update_model(self, forced):
        '''
        This function will be called every time the timer is running out. 
        The function changes the state of this class depending on 
        the actual state of this class and the data received by the model through ROS2. 
        Depending on the state of this class different actions will be performed by the model.
        
        Following states are accepted: 
        -   PUBLISH: model sends information about the map (MapInfo.msg) and every robot (RobotInfo.msg) to the CONTROLLER
        -   WAIT:    no action until the model has received all information about the next actions from the CONTROLLER
        -   EXEC:    modelView executes all received actions from the CONTROLLER by the model for the next step incrementally.
                     After completing the step by the modelview the model will be updated with new positions of the robots and 
                     the simulation changes it's state to PUBLISH again!
        '''
        if self._model == None: 
            return        
                
        #STATE 0: PUBLISH ROS MESSAGES
        if self._mode == PUBLISH:
            print("MODE: PUBLISH INFO (MapInfo.msg and RobotInfo.msg)")    
            self._model.mapf_init_next_step()
            #set time for next timeout, after timeout come back
            self._timeout_end = self.get_next_ros_timeout()
            self._mode = WAIT
            
        #STATE 1: UPDATE, WAIT
        if self._mode == WAIT:
            print("MODE: WAITING (for RobotAction.msg)")    
            simulation_ready, timeout = self._model.mapf_spin_once(self._timeout_end)        
            if simulation_ready:
                self._mode = EXEC
            elif timeout:
                self._mode = PUBLISH
                
        #STATE 2: EXECUTE ONE STEP
        if self._mode == EXEC:
            print("MODE: EXEC (moving robots)")    
            #if self._timer_count >= 10 or forced:
            if self._timer_count >= self._timer_count_max or forced:
                self._model.update()
                self._timer_count = 0
                self._timeout_end = self.get_next_ros_timeout()
                self._mode = PUBLISH
            else: 
                self._timer_count += 1
    
            self.update()

    def do_model(self):
        '''
        This function performs a step forward when the button ">" in the GUI was pressed
        '''
        if self._model == None: 
            return
        self._model.redo()
        self.update() 

    def undo_model(self):
        '''
        This function performs a step back when the button "<" in the GUI was pressed
        '''
        if self._model is None: 
            return
        self._model.undo()
        self.update()
 
    def _scale(self, scale_x, scale_y):
        '''
        This function sets the scale factors for painting the elements in the visualization.
        '''
        old_zoom = min(self._zoom[0], self._zoom[1])
        new_zoom = min(self._zoom[0]*scale_x, self._zoom[1]*scale_y)
        scale = new_zoom/old_zoom
        if old_zoom < 1.0:
            scale = new_zoom
        if  new_zoom >= 1.0:
            self.scale(scale, scale)
            self._display_mode = 0
            self._scaling = 1.0
            self._border_size = 3
        else:
            if self._display_mode == 0:
                self.scale(1.0/old_zoom, 1.0/old_zoom)
            self._display_mode = 1
            self._scaling = min(self._zoom[0]*scale_x, self._zoom[1]*scale_y)
            self._border_size = 2
            if new_zoom < 0.5:
                self._border_size = 1                
        self._zoom = (self._zoom[0]*scale_x, self._zoom[1]*scale_y)
        self.update()

    def clear(self):
        '''
        This function removes the items from the window which belong to the previous paint. 
        Every time the visualization is updated all old elements are removed from the window. 
        '''
        for line in self._lines:
            self._scene.removeItem(line)
        self._lines = []        

        for item in self._items_in_scene:
            self._scene.removeItem(item)
        self._items_in_scene = []
        #self._scene.clear()

    def scene_coordinates_to_grid(self, x, y):
        '''
        @private
        '''
        if (x < 0 or y < 0 
            or x > self._line_hlength*self._scaling 
            or y > self._line_vlength*self._scaling):
            return None
        result = (int(x/(self._w_distance*self._scaling) + 1),
                    int(y/(self._h_distance*self._scaling) + 1))
        if result[0] > self._model.get_grid_size()[0]:
            result = (self._model.get_grid_size()[0], result[1])
        if result[1] > self._model.get_grid_size()[1]:
            result = (result[0], self._model.get_grid_size()[1])
        return result

    def scene_coordinates_to_node(self, x, y):
        '''
        @private
        '''
        node = self.scene_coordinates_to_grid(x,y)
        if node is None:
            return None
        if self._model.is_node(node[0], node[1]):
            return node
        return None

    def get_next_ros_timeout(self):
        '''
        This function returns the next timeout for ROS2 after which the simulation should move back to the mode PUBLISH.  
        
        This timeout is needed in the case the simulation has not received the needed information from the controller after 
        a specific time. In this case the complete cycle should be restarted. 
        This done by the simulation publishing the information about the map and the items once again.
        '''
        return (time.time() + 60)


    '''
    self._grid_size_x = 1  
    self._grid_size_y = 1  
    
    def restart_model(self):
        if self._model is None: 
            return
        self._model.restart()
        self.update()
    
    def get_timer_speedup(self):
        return self._timer_scale
    
    def _pickup(self, robot, shelf):
        robot.set_initial_carries(shelf)
        robot.set_carries(shelf)
        self._menu.hide()
        self._model.update_windows()

    def _add_highway(self, x, y):
        self._model.add_highway(x, y)
        self._menu.hide()
        self._model.update_windows()
    
    def _remove_highway(self, x, y):
        self._model.remove_highway(x, y)
        self._menu.hide()
        self._model.update_windows()
    '''

    '''    
    def enable_drag(self, enable):
        for item in self.get_model().iterate_graphic_items():
            item.enable_drag(enable)
    '''

    '''
    def contextMenuEvent(self, event):
        if self._model == None:
            return

        point = self.mapToScene(event.x(), event.y())
        node = self.scene_coordinates_to_grid(point.x(), point.y())
        if node is None:
            return
        x = node[0]
        y = node[1]

        if not self._model.get_editable():
            return 

        self._menu.clear() 

        if self._model.is_node(x,y):
            action = QAction('disable node', self)
            action.setShortcut('Ctrl + N')
            action.setStatusTip('Disables the selected node')
            action.triggered.connect(lambda: self._remove_node(x, y))
            self._menu.addAction(action)   
        else:
            action = QAction('enable node', self)
            action.setShortcut('Ctrl + N')
            action.setStatusTip('Enables the selected node')
            action.triggered.connect(lambda: self._add_node(x, y))
            self._menu.addAction(action) 
        
        if self._model.is_highway(x,y):
            action = QAction('remove highway', self)
            action.setShortcut('Ctrl + H')
            action.setStatusTip('Removes a highway from the selected node')
            action.triggered.connect(lambda: self._remove_highway(x,y))
            self._menu.addAction(action)                 
        elif self._model.is_node(x,y):
            action = QAction('add highway', self)
            action.setShortcut('Ctrl + H')
            action.setStatusTip('Adds a highway to the selected node')
            action.triggered.connect(lambda: self._add_highway(x,y))
            self._menu.addAction(action)                   

        robot = self._model.filter_items('robot', position = (x,y), return_first = True)[0]
        shelf = self._model.filter_items('shelf', position = (x,y), return_first = True)[0]

        if robot is not None:
            if shelf is not None: 
                if robot.get_initial_carries() is not None:
                    action = QAction('putdown', self)
                    action.setShortcut('Ctrl + P')
                    action.setStatusTip('The robot in the selected node puts down the carried shelf')
                    action.triggered.connect(lambda: self._pickup(robot, None))
                    self._menu.addAction(action)
                else:
                    action = QAction('pickup', self)
                    action.setShortcut('Ctrl + P')
                    action.setStatusTip('The robot in the selected node picks up the shelf in the selcted node')
                    action.triggered.connect(lambda: self._pickup(robot, shelf))
                    self._menu.addAction(action)

                    action = QAction('remove robot', self)
                    action.setShortcut('Ctrl + R')
                    action.setStatusTip('Removes a robot from the selected node')
                    action.triggered.connect(lambda: self._remove_item(robot))
                    self._menu.addAction(action)
            else:
                action = QAction('remove robot', self)
                action.setShortcut('Ctrl + R')
                action.setStatusTip('Removes a robot from the selected node')
                action.triggered.connect(lambda: self._remove_item(robot))
                self._menu.addAction(action)

        elif self._model.is_node(x,y):
            action = QAction('add robot', self)
            action.setShortcut('Ctrl + R')
            action.setStatusTip('Adds a robot to the selected node')
            action.triggered.connect(lambda: self._add_item('robot', x, y))
            self._menu.addAction(action)

        if shelf is not None:
            action = QAction('remove shelf', self)
            action.setShortcut('Ctrl + S')
            action.setStatusTip('Removes a shelf from the selected node')
            action.triggered.connect(lambda: self._remove_item(shelf))
            self._menu.addAction(action)
        elif self._model.is_node(x,y):
            action = QAction('add shelf', self)
            action.setShortcut('Ctrl + S')
            action.setStatusTip('Adds a shelf to the selected node')
            action.triggered.connect(lambda: self._add_item('shelf', x, y))
            self._menu.addAction(action)

        station = self._model.filter_items('pickingStation', position = (x,y), return_first = True)[0]
        if station is not None:
            action = QAction('remove picking station', self)
            action.setShortcut('Ctrl + I')
            action.setStatusTip('Removes a picking station from the selected node')
            action.triggered.connect(lambda: self._remove_item(station))
            self._menu.addAction(action)
        elif self._model.is_node(x,y):
            action = QAction('add picking station', self)
            action.setShortcut('Ctrl + I')
            action.setStatusTip('Adds a picking Station to the selected node')
            action.triggered.connect(lambda: self._add_item('pickingStation', x, y))
            self._menu.addAction(action)           

        station2 = self._model.filter_items('chargingStation', position = (x,y), return_first = True)[0]
        if station is not None:
            action = QAction('remove charging station', self)
            action.setShortcut('Ctrl + B')
            action.setStatusTip('Removes a charging station from the selected node')
            action.triggered.connect(lambda: self._remove_item(station))
            self._menu.addAction(action)
        elif self._model.is_node(x,y):
            action = QAction('add charging station', self)
            action.setShortcut('Ctrl + B')
            action.setStatusTip('Adds a charging Station to the selected node')
            action.triggered.connect(lambda: self._add_item('chargingStation', x, y))
            self._menu.addAction(action)

        self._menu.popup(QPoint(event.x(),event.y()))
    '''

    '''
    def speed_up_timer(self, speed_up):
        self._timer_scale *= 1 - speed_up
        if self._timer_scale <= 0.1:
            self._timer_scale = 0.1
        self.adjust_timer()
    
    '''

    '''
        #STATE 0: PUBLISH MESSAGES
        if self._mode == PUB:

        
        #STATE 1: UPDATE, WAIT 
        if self._mode == WAIT:
            simulation_ready, self._just_started, reset_time = self._model.mapf_spin_once(self._just_started, 
                                                                                          self._timeout_end)        
            #reset timeout(only if not all needed ros messages received(e.g. RobotAction.msg) after a certain time
            if reset_time:
                self._timeout_end = self.get_next_ros_timeout()

            #if simulation ready then change to the EXECUTION mode
            if simulation_ready:
                self._mode == EXEC
                
        #STATE 2: EXECUTE ONE STEP
        elif self._mode == EXEC:
            if self._timer_count >= 10 or forced:
                self._model.update()
                self._timer_count = 0
                self._timeout_end = self.get_next_ros_timeout()
                self._mode == PUB
            else: 
                self._timer_count += 1

            self.update()
        '''

    '''
    #at first run init step
    if self._just_started == True:
        print("RESET ALL AND PUBLISH INFO")
        self._model.mapf_init_next_step()
        self._just_started = False
        
    #process all ros messages from callback functions
    self._model.mapf_ros_spin_once()

    #init step again if no ros messages received from controller (e.g. RobotAction.msg) after a certain time
    #if not self._model.mapf_all_ros_msg_received() and (time.time() > self._timeout_end): 
    if not simulation_ready and (time.time() > self._timeout_end): 
        print("RESET ALL AND PUBLISH INFO after TIMEOUT and NO ACTIONS RECEIVED")
        self._model.mapf_init_next_step()
        self._timeout_end = self.get_next_ros_timeout()
        return

    #return if not all messages received for the next step
    #if not self._model.mapf_all_ros_msg_received():
    if not simulation_ready:
        self._timer_count = 0
        return 
    
    '''