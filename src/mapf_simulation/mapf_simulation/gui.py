from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import os
import sys
import time
import re
import mapf_simulation.configuration
from mapf_simulation.parser import MAPFfileParser


#VIZ_STATE_MOVED     = 0x01
#VIZ_STATE_DELIVERED = 0x02
#VIZ_STATE_PICKED_UP = 0x04
#VIZ_STATE_PUT_DOWN  = 0x08
#VIZ_STATE_MOVE      = 0x10
#VIZ_STATE_DELIVER   = 0x20
#VIZ_STATE_PICK_UP   = 0x40
#VIZ_STATE_PUT_DOWN2 = 0x80
#VIZ_STATE_ACTION    = 0xff


class VizWidget(QWidget):
    '''
    @private
    This class taken from "asprilo visualizer"
    https://github.com/potassco/asprilo/tree/master/visualizer
    '''

    def __init__(self):
        super(VizWidget, self).__init__()

    def release_widget(self):
        parent = self.parent()
        if parent is not None:
            self.setParent(None)
            self.show()

    def mousePressEvent(self, event):
        QWidget.mousePressEvent(self, event)
        if(event.isAccepted()):
            return
        #if(event.button() is not Qt.LeftButton):
        #    return
        event.accept()
        self.release_widget()
        self.move(event.globalX(), event.globalY())
        self.grabMouse()

    def mouseMoveEvent(self, event):
        self.move(event.globalX(), event.globalY())

    def mouseReleaseEvent(self, event):
        self.releaseMouse()
        widget_manager.drop_widget(self, event.globalX(), event.globalY())

class VizSplitter(QSplitter):
    '''
    @private
    This class taken from "asprilo visualizer"
    https://github.com/potassco/asprilo/tree/master/visualizer
    '''

    def __init__(self):
        super(VizSplitter, self).__init__()
        self._childrean_count = 0

    def add_widget(self, widget):
        splitter.addWidget(widget)
        widget.show()
  
    def closeEvent(self, close):
        for ii in range (0, self.count()):
            widget = self.widget(ii)
            if widget.parent() is self:
                widget.setParent(None)
                widget.hide()
        widget_manager.remove(self)

    def childEvent(self, event):
        if event.added() is True:
            self._childrean_count += 1
        elif event.removed() is True:
            self._childrean_count -= 1
        else:
            return
        if self._childrean_count <= 1 and event.removed() is True:
            self.close()

class VizTableWidgetItem(QTableWidgetItem):
    '''
    @private
    This class taken from "asprilo visualizer"
    https://github.com/potassco/asprilo/tree/master/visualizer
    '''

    def __lt__(self, other):
        try:
            return int(self.text()) < int(other.text())
        except:
            return super(VizTableWidgetItem, self).__lt__(other)

class WidgetManager(object):
    '''
    @private
    This class taken from "asprilo visualizer"
    https://github.com/potassco/asprilo/tree/master/visualizer
    '''

    def __init__(self):
        self._splitter = []
        self._main_widget = None

    def add(self, widget):
        self._splitter.append(widget)

    def set_main_widget(self, widget):
        self._main_widget = widget

    def remove(self, widget):
        for splitter in self._splitter:
            if widget is splitter:
                self._splitter.remove(splitter)
                return

    def exit(self):
        for splitter in self._splitter:
            splitter.close()

    def drop_widget(self, widget, x, y):
        if widget is None:
            return
        #main widget
        if self._main_widget is not None:
            splitter = self._main_widget.get_splitter()
            splitter_pos = splitter.mapToGlobal(splitter.pos())
            if (splitter_pos.x() < x and splitter_pos.y() < y and
                splitter_pos.x() + splitter.size().width() > x and
                splitter_pos.y() + splitter.size().height() > y):
                pos = splitter_pos.x()
                index = 0
                for size in splitter.sizes():
                    if x > pos and x < pos + size:
                        if x < pos + size/2:
                            splitter.insertWidget(index, widget)
                            splitter.resize(splitter.width() + widget.width(), splitter.height())
                            return
                        else:
                            splitter.insertWidget(index + 1, widget)
                            splitter.resize(splitter.width() + widget.width(), splitter.height())
                            return
                    else:
                        pos += size
                        index += 1

        #other widgets
        for splitter in self._splitter:
            if (splitter.pos().x() < x and splitter.pos().y() < y and
                splitter.pos().x() + splitter.size().width() > x and
                splitter.pos().y() + splitter.size().height() > y):
                pos = splitter.pos().x()
                index = 0
                for size in splitter.sizes():
                    if x > pos and x < pos + size:
                        if x < pos + size/2:
                            splitter.insertWidget(index, widget)
                            splitter.resize(splitter.width() + widget.width(), splitter.height())
                            return
                        else:
                            splitter.insertWidget(index + 1, widget)
                            splitter.resize(splitter.width() + widget.width(), splitter.height())
                            return
                    else:
                        pos += size
                        index += 1

        #create new widget
        splitter = VizSplitter()
        splitter.insertWidget(0, widget)
        splitter.resize(widget.size())
        splitter.move(x,y)
        widget.show()
        splitter.show()
        self._splitter.append(splitter)

widget_manager = WidgetManager()
'''
@private
'''


class LeftSideWidget(VizWidget):
    '''
    This class represents the left side of the GUI. It provides graphical elements/interfaces 
    for the user to interact with the simulation.
    The left side can be divided into the following four clusters:
    -   LoadFilesWidget (for loading the files)
    -   SimulationWidget (to start/stop/adjust the simulation)
    -   ZoomWidget (for visual settings)
    -   OthersWidget (for the rest)
    '''

    def __init__(self):
        '''
        Creates the following widgets: LoadFilesWidget, SimulationWidget, ZoomWidget, OthersWidget  
        Creates a vertical QSplitter element and adds the four widgets to QSplitter  
        Adjusts the size of the widgets inside the QSplitter
        '''
        super(LeftSideWidget, self).__init__()
        self.setMinimumWidth(120)
        self._splitter = QSplitter(Qt.Vertical, self)
        #self._splitter.setChildrenCollapsible(False)

        self._loadfiles_widget = LoadFilesWidget()
        #self._loadfiles_widget.setFixedHeight(220)
        self._sim_widget = SimulationWidget()
        self._zoom_widget = ZoomWidget()
        self._others_widget = OthersWidget()

        self._splitter.addWidget(self._loadfiles_widget)
        self._splitter.addWidget(self._sim_widget)
        self._splitter.addWidget(self._zoom_widget)
        self._splitter.addWidget(self._others_widget)

        self._splitter.setSizes([210, 210, 90, 150])
        self._splitter.show()

    def set_model_view(self, model_view):
        self._loadfiles_widget.set_model_view(model_view)
        self._sim_widget.set_model_view(model_view)
        self._zoom_widget.set_model_view(model_view)
        self._others_widget.set_model_view(model_view)
        
    def set_model(self, model):
        self._loadfiles_widget.set_model(model)

    def resizeEvent(self, event):
        self._splitter.resize(event.size())

class RightSideWidget(VizWidget):
    '''
    This class represents the right side of the GUI. It provides a table with detailed information 
    about the robots.
    '''
     
    def __init__(self):
        '''
        Creates a QTableWidget with the following horizontal header labels:  
        Robot ID, Position, Start, Goal, Step, Last action, Heuristics, Bucket
        '''
        super(RightSideWidget, self).__init__()
        self._table = QTableWidget(self)
        self._table.move(0, 20)
        self.setWindowTitle('Robot Table')
        self._table.setWindowTitle('Robot Table')
        self._table.setToolTip('select Row by clicking \ndeselect row by Strg+clicking')
        #self.setMinimumWidth(100)
        
        self._model = None
        self._model_view = None
        self._table.setColumnCount(8)
        self._table.setHorizontalHeaderLabels(['Robot ID', 'Position', 'Start', 'Goal',
                                               'Step', 'Last action', 'Heuristics', 'Bucket'])
        self._table.itemSelectionChanged.connect(self.on_selection_changed)
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table_items = {}
        
    def set_model_view(self, model_view):
        #self._loadfiles_widget.set_model_view(model_view)
        self._model_view = model_view
        if self._model_view is not None:
            self._model_view.add_window(self)

    def set_model(self, model):
        self._model = model
        if self._model is not None:
            self._model.add_window(self)
        self.update()

    def set_item_text(self, column, row, text, brush):
        if not row in self._table_items:
            self._table_items[row] = {}
        if not column in self._table_items[row]:
            self._table_items[row][column] = VizTableWidgetItem(text)
            self._table.setItem(column, row, self._table_items[row][column])
        self._table_items[row][column].setFlags( Qt.ItemIsSelectable |  Qt.ItemIsEnabled )
        self._table_items[row][column].setBackground(brush)
        self._table_items[row][column].setText(text)

    def on_selection_changed(self):
        robot_ids = []
        rows = []
        for index in self._table.selectedIndexes():
            row = index.row()
            if row not in rows:
                rows.append(row)
        for row in rows:
            robot_ids.append(self._table.item(row, 0).text())
        robots = self._model.filter_items(item_kind = 'robot_ros')
        for robot in robots:
            task = robot.get_task(0)
            checkpoints = task.get_checkpoints()
            highlight = False
            if robot.get_id() in robot_ids:
                robot_ids.remove(robot.get_id())
                highlight = True

            robot.set_highlighted(highlight)
            for checkpoint, name in checkpoints:
                    checkpoint.set_highlighted(highlight)
        self._model.update_windows()

    def resizeEvent(self, event):
        self._table.resize(event.size())
        #self._table.resize(event.size().width(), event.size().height()-20)

    def selectRow(self, count, select):
        columns = self._table.columnCount()
        self._table.setRangeSelected(QTableWidgetSelectionRange(count, 0, count, columns-1), select)
        
    def update(self):
        self._table.setSortingEnabled(False)
        red_brush   = QBrush(QColor(200, 100, 100))
        green_brush = QBrush(QColor(100, 200, 100))
        white_brush = QBrush(QColor(255, 255, 255))
        blue_brush  = QBrush(QColor(155, 155, 255))
        orange_brush = QBrush(QColor(255, 128, 0))
        ignore_first = 1
        if self._model is None:
            return
        count = 0
        robots = self._model.filter_items(item_kind = 'robot_ros')
        if(len(robots) != self._table.rowCount()):
            self._table.clearSelection()
            self._table_items = {}
            self._table.setRowCount(len(robots))

        for robot in robots:
            #HERE SELECT ROW IF HIGHLIGHTED
            if robot.get_highlighted():
                self.selectRow(count, True)
            else:
                self.selectRow(count, False)

            #Column 1: ID of the robot
            if robot.check_robot_at_target():
                self.set_item_text(count, 0, robot.get_id(), green_brush)
            else:
                self.set_item_text(count, 0, robot.get_id(), white_brush)

            #Column 2: Current position
            brush = white_brush
            self.set_item_text(count, 1, str(robot.get_position()[0]) + ', ' + str(robot.get_position()[1]), brush)
            #Column 3: Start position
            self.set_item_text(count, 2, str(robot.get_start_position()[0]) + ', ' + str(robot.get_start_position()[1]), brush)
    
            #Column 4: Goal position (here the position of the only one checkpoint in the only one task of the robot)
            #assume there is only one task in the task list of the robot (-> index 0)
            task = robot.get_task(index =0)
            #assume there is only one checkpoint in the task -> [0], 
            checkpoint = task.get_checkpoints()[0][0]
            self.set_item_text(count, 3, str(checkpoint.get_position()[0]) + ', ' + str(checkpoint.get_position()[1]), brush)
            
            #Column 5: Step
            step = robot.get_model().get_current_step()
            self.set_item_text(count, 4, str(step), brush)
    
            #Column 6: Last Action
            if step != 0: #if step 0, then simulation has not started yet
                action = robot.get_action(step-1) #step 1 corresponds to the action 0 from the robot action list
                
                #if there is an action defined for the current step
                if action is not None:
                    #if the action is of type "move" with X,Y values
                    if action.arguments[0].name == 'move':
                        action_value = action.arguments[1]
                        self.set_item_text(count, 5, str(action_value.arguments[0].number) + 
                                    ", " + str(action_value.arguments[1].number), brush)
                    else:
                        self.set_item_text(count, 5, "0, 0", brush)
                else:
                    self.set_item_text(count, 5, "0, 0", brush)    
            else:
                self.set_item_text(count, 5, "0, 0", brush)

            #Column 7: Heuristic, optimal length
            self.set_item_text(count, 6, str(robot.get_heuristic()), brush)

            #Column 8: Bucket number
            self.set_item_text(count, 7, str(robot.get_bucket()), brush)
            
            count += 1
        
        return 1


class LoadFilesWidget(QWidget):
    '''
    This widget offers buttons and other elements for the loading of the .map and .scen files. 
    It is usually placed on the top on the left side of the window.

    Elements:
    -   Button:     for loading the .map file
    -   Button:     for loading the .scen file
    -   SpinBox:    for setting the number of the agents
    -   TextEdit:   for setting the list of buckets before loading the .scen file
    '''

    def __init__(self):
        '''  
        Creates the widget
        '''
        super(self.__class__, self).__init__()
        self.setFixedHeight(210)

        self._model_view = None
        self._model      = None
        self._file_loaded_map   = True
        self._file_loaded_scene = False

        self._mapf_file_parser  = MAPFfileParser()

        self._groupbox = QGroupBox("Input",self)
        self._groupbox.setGeometry(20,10,170,200)

        self._verticalLayout = QVBoxLayout()
        self._groupbox.setLayout(self._verticalLayout)

        self._file_dialog   = QFileDialog(self)
        
        #---------------------          Button: "Load map"          ---------------------
        self._buttonLoadMap = QPushButton("Open Map File", self)
        self._buttonLoadMap.clicked.connect(lambda: 
                        (self.open_file_dialog(QFileDialog.AcceptOpen, 
                         self.load_map, "map")))
        self._verticalLayout.addWidget(self._buttonLoadMap)
        
        #---------------------          Button: "Load scene"        ---------------------
        self._buttonLoadScene = QPushButton("Open Agents File", self)
        self._buttonLoadScene.clicked.connect(lambda: 
                        (self.open_file_dialog(QFileDialog.AcceptOpen, 
                         self.load_scene, "scene")))
        self._verticalLayout.addWidget(self._buttonLoadScene)

        #---------------------          Function: "Choose k-Agents" ---------------------
        self._horizontalLayout = QHBoxLayout()
        self._verticalLayout.addLayout(self._horizontalLayout)
 
        #   Label: "Agents [number]"
        self._l1 = QLabel("Agents:\n[number]")
        self._l1.setAlignment(Qt.AlignCenter)
        self._horizontalLayout.addWidget(self._l1)
        #   SpinBox
        self._sp = QSpinBox()
        self._horizontalLayout.addWidget(self._sp)
        self._sp.valueChanged.connect(self.number_agents_changed)

        #---------------------          Function: "Choose Bucket Number" ---------------------
        self._horizontalLayout2 = QHBoxLayout()
        self._verticalLayout.addLayout(self._horizontalLayout2)
 
        #   Label: "number agents"
        self._l2 = QLabel("Bucket:\n[number]")
        self._l2.setAlignment(Qt.AlignCenter)
        self._horizontalLayout2.addWidget(self._l2)
        #   LineEdit
        self._sp2 = QLineEdit()
        self._horizontalLayout2.addWidget(self._sp2)
        self._sp2_last_value = self._sp2.text()
        self._sp2.textEdited.connect(self.number_bucket_changed)

        #Set which elements should be disabled at the begin
        self._buttonLoadScene.setEnabled(False)
        self._l1.setEnabled(False)
        self._sp.setEnabled(False)
        self._l2.setEnabled(False)
        self._sp2.setEnabled(False)

    def set_model(self, model):
        self._model = model
        self._mapf_file_parser.set_model(model)
    
    def set_model_view(self, model_view):
        self._model_view = model_view
              
    def get_main_widget(self):
        '''
        @private
        '''
        return self.parent().parent().parent().parent()
    
    #LOAD FILES    
    def open_file_dialog(self, mode, function, source):
        '''
        This function opens a QFileDialog.   
        The function although adapts the title and file filters depending on what type of files should be opened
        (.map or .scene)

        :param mode: PySide2.QtWidgets.QFileDialog.AcceptMode   
        (QFileDialog.AcceptOpen or QFileDialog.AcceptSave)
        :param function: - function that should be executed after selecting the file
        :param source: - shows what type of file should be opened ("map" or "scene")
        '''
        if source == "map":
            self._file_dialog.setWindowTitle('Open Map File (*.map)')
            self._file_dialog.setNameFilters(['Map files (*.map)', 'All files (*)'])
        else:
            self._file_dialog.setWindowTitle('Open Scene File (*.scen)')
            self._file_dialog.setNameFilters(['Scene files (*.scen)', 'All files (*)'])
        
        self._file_dialog.setAcceptMode(mode)
        try:
            self._file_dialog.disconnect()
        except TypeError as error:
            pass
        self._file_dialog.accepted.connect(function)
        self._file_dialog.open()

    def load_map(self, event = None):
        '''
        This function loads the .map file.   
        The function is called when the button "Open Map File" is pressed.
        '''

        if self._model_view is None:
            return
        if self._model is None:
            return
        #if not self._model.get_editable():
        #    return

        self._mapf_file_parser.set_model(self._model)
        self._mapf_file_parser.set_model_view(self._model_view)
        file_name = self._file_dialog.selectedFiles()[0]
        
        if self._mapf_file_parser.load_map(file_name):
            self._file_loaded_map = True
            self._buttonLoadScene.setEnabled(True)
            self._l1.setEnabled(True)
            self._sp.setEnabled(True)
            self._sp.setMinimum(0)
            self._sp.setValue(0)
            self._sp.setMaximum(self._model.get_max_number_robots())
            self._l2.setEnabled(True)
            self._sp2.setEnabled(True)
            self.get_main_widget().setWindowTitle('MAPF Simulator   -   map file:   ' 
                                                  + self._model.mapf_get_filename_map())       
        return 

    def load_scene(self, event = None):
        '''
        This function loads the .scen file.   
        The function is called when the button "Open Agents File" is pressed.
        '''
        if self._model_view is None:
            return
        if self._model is None:
            return
        #if not self._model.get_editable():
        #    return
        if not self._file_loaded_map:
            return

        if not self.check_bucket_input():
            msg = QMessageBox()
            msg.setWindowTitle("WARNING")
            msg.setIcon(QMessageBox.Information)
            msg.setText("WRONG INPUT FOR THE FIELD:   \"Bucket [number]\"    ")
            #msg.setInformativeText("This is additional information")
            msg.setDetailedText("Input tested by the following regular expression: \n (^([0-9\;])*(([0-9])+(\-)?([0-9\;])+)*(([0-9\;])*)$)+"
                               + "\n\nValid exampes:"
                               + "\nempty field     ---> all buckets"
                               + "\n1               ---> bucket 1"
                               + "\n2-5             ---> buckets 2, 3, 4, 5"
                               + "\n1;3-6;9         ---> buckets 1, 3, 4, 5, 6, 9"
                               + "\n1;3-5;7; 11-13  ---> buckets 1, 3, 4, 5, 7, 11, 12 , 13"
                               )
            msg.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            msg.setMinimumSize(400,400)
            msg.setFixedSize(400,400)
            
            retval = msg.exec_()
            msg.show()

        self._mapf_file_parser.set_model(self._model)
        self._mapf_file_parser.set_model_view(self._model_view)
        file_name = self._file_dialog.selectedFiles()[0]
        
        if self._mapf_file_parser.load_scene(file_name, self._sp2.text()):
            self._file_loaded_scene = True
            self.parent().parent()._sim_widget.setEnabled(True)
            self.parent().parent()._zoom_widget.setEnabled(True)
            

            self._sp.setValue(0)
            self._sp.setMinimum(1)
            self._sp.setMaximum(self._model.get_max_number_robots())
            
            #self._model.reset_max_number_robots()
            self._sp.setValue(self._model.get_max_number_robots())
            self._l1.setText("Agents:\n[max:" + str(self._model.get_max_number_robots()) + "]")
            #self._l1.setText("Agents:\n[max:" + str(1000) + "]")

            self._model.mapf_robots_enable_draw_start(self.parent().parent()._zoom_widget._checkbox1.isChecked())
            self._model.mapf_robots_enable_draw_path(self.parent().parent()._zoom_widget._checkbox1.isChecked())
            self._model.mapf_robots_enable_draw_future_path(self.parent().parent()._zoom_widget._checkbox2.isChecked())
            self._model.set_editable(self.parent().parent()._zoom_widget._checkbox3.isChecked())

            self.get_main_widget().setWindowTitle('MAPF Simulator   -   map file:   ' 
                                                  + self._model.mapf_get_filename_map() + '   -   scene file:   ' 
                                                  + self._model.mapf_get_filename_scene() + '  -   buckets: '
                                                  + self._model.mapf_get_bucket_list())
        return   

    #NUMBER AGENTS
    def number_agents_changed(self):
        '''
        This function gets called when the number of agents in the QSpinBox "Agents[number]" has been changed.
        The function checks whether the input is valid.   
        If the number is valid then the number of robots will be updated.
        Otherwise the number will be set to the previuos value.
        '''
        #INVALID NUMBER: x>max (max = number of robots loaded from .scen file)
        if (self._sp.value() > self._model.get_max_number_robots()):
            self._sp.setValue(self._model.get_max_number_robots())
        
        #VALID NUMBER: 0<x<max
        if self._sp.value() >= 0:
            self._model.mapf_update_number_robots(self._sp.value())
        
        #INVALID: x<0
        if (self._sp.value() < 0):
            self._sp.setValue(0)

    #NUMBER BUCKETS
    def number_bucket_changed(self):
        '''
        This function gets called when the input in TextEdit "Bucket[number]" has been modified.  
        The function checks whether the input string contains only the following characters: 0-9 ; and -  
        If the input is invalid then it will be set to the previuos value.

        :return: **True**, if the input is valid  
        **False**, if the input is not valid
        '''
        acceptable_chars = ['0','1','2','3','4','5','6','7','8','9', ';', '-']
        valid = [i in acceptable_chars for i in self._sp2.text()]
        if not all(valid):
            print("Invalid character, only following characters are acceptable: 0-9 ; -")
            self._sp2.setText(self._sp2_last_value)
            return False
        
        self._sp2_last_value = self._sp2.text()
        return True
    
    def check_bucket_input(self):
        '''
        This function checks whether the input string fur buckets matches the following regular expression:  
        > (^([0-9\;])*(([0-9])+(\-)?([0-9\;])+)*(([0-9\;])*)$)+  

        Explanation:
        >   input starts with 0-9 or ;       ^([0-9\;])*   
        >   input ends with 0-9 or ;         (([0-9\;])*)$   
        >   in the middle 0, 1 or multipal expressions of type: x-x and x, like 11-14 or 2-8, 7, all separated by a semicolon ;      

        Valid Examples:
        >   7        --> bucket 7  
        >   4;6      --> buckets 4 and 6  
        >   1-3;6-10 --> buckets 1,2,3, 6,7,8,9,10  

        :return: **True**, if the input string matches the regular expression  
        **False**, otherwise
        '''

        if not re.match('(^([0-9\;])*(([0-9])+(\-)?([0-9\;])+)*(([0-9\;])*)$)+', self._sp2.text()):
            return False

        return True
    

class SimulationWidget(QWidget):
    '''
    This widget offers following elements:
    -   button "Start/Stop Sim" for starting/stopping the simulation
    -   buttons "<", ">", "|<", ">|" for navigation through the steps
    -   buttons  "UP" and "DOWN" to adjust the speed of the simulation 
    '''
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setFixedHeight(210)

        self._model_view = None

        self._groupbox = QGroupBox("Simulation",self)
        self._groupbox.setGeometry(20,10,170,200)

        self._verticalLayout = QVBoxLayout()
        self._groupbox.setLayout(self._verticalLayout)

        #---------------------          Button: "Start Sim"          ---------------------
        self._buttonStartSim = QPushButton("Start Sim", self)
        self._buttonStartSim.clicked.connect(self.on_pause)
        self._verticalLayout.addWidget(self._buttonStartSim)
        

        self._subGridLayout = QGridLayout()
        self._verticalLayout.addLayout(self._subGridLayout)
        #---------------------          Button: "<"        ---------------------
        self._buttonStepBack = QPushButton("<", self)
        self._buttonStepBack.clicked.connect(self.on_undo)
        self._subGridLayout.addWidget(self._buttonStepBack, 0,0)

        #---------------------          Button: ">"        ---------------------
        self._buttonStepForward = QPushButton(">", self)
        self._buttonStepForward.clicked.connect(self.on_update)
        self._subGridLayout.addWidget(self._buttonStepForward, 0,1)

        #---------------------          Button: "|<"        ---------------------
        self._buttonStepFirst = QPushButton("|<", self)
        self._buttonStepFirst.clicked.connect(self.on_restart)
        self._subGridLayout.addWidget(self._buttonStepFirst, 1,0)

        #---------------------          Button: ">|"        ---------------------
        self._buttonStepLast = QPushButton(">|", self)
        self._buttonStepLast.clicked.connect(self.on_skip_to_end)
        self._subGridLayout.addWidget(self._buttonStepLast, 1,1)

        #---------------------          Label: "Speed"      ---------------------
        self._labelSpeed = QLabel("Speed:")
        self._labelSpeed.setAlignment(Qt.AlignCenter)
        self._subGridLayout.addWidget(self._labelSpeed, 2,0)

        #---------------------          TextEdit: "Speed"             ---------------------
        self._textSpeed = QTextEdit()
        self._textSpeed.setReadOnly(True)
        self._subGridLayout.addWidget(self._textSpeed, 2,1)
        #self._sp.valueChanged.connect(self.number_agents_changed)

        #---------------------          Button: "UP"        ---------------------
        self._buttonUP = QPushButton("UP", self)
        self._buttonUP.clicked.connect(self.on_speed_up)
        self._subGridLayout.addWidget(self._buttonUP, 3,0)

        #---------------------          Button: "DOWN"        ---------------------
        self._buttonDOWN = QPushButton("DOWN", self)
        self._buttonDOWN.clicked.connect(self.on_speed_down)
        self._subGridLayout.addWidget(self._buttonDOWN, 3,1)

        self.setEnabled(False)
        self._sim_started_once = False
        self._sim_running = False

    def set_model_view(self, model_view):
        self._model_view = model_view
        self._model_view.get_model().add_window(self)
        self.update()

    def update(self):
        '''
        The function is called when the widget (self) should be updated.   
        Depending on whether the simulation is running:
        -   sets the right text for the button "Start/Stop Sim" 
        -   enables / disables several features in the gui
        -   updates the output for speed in the widget (QTextEdit) 
            in the case the speed of the simulation has been changed outside
        '''
        if self._model_view is None:
            return
        if self._model_view.get_model() is None:
            return
        
        #IF SIMULATION IS NOT RUNNING
        if not self._sim_running: #if self._model_view.is_timer_running():
            self._buttonStartSim.setText('Start Sim')    
            if self._sim_started_once:
                self._buttonStepBack.setEnabled(True)
                self._buttonStepForward.setEnabled(True)
                self._buttonStepFirst.setEnabled(True)
                self._buttonStepLast.setEnabled(True)
            else:
                self._buttonStepBack.setEnabled(False)
                self._buttonStepForward.setEnabled(False)
                self._buttonStepFirst.setEnabled(False)
                self._buttonStepLast.setEnabled(False)
        
        #IF SIMULATION IS RUNNING
        else:
            self._buttonStartSim.setText('Stop Sim')
            self._buttonStepBack.setEnabled(False)
            self._buttonStepForward.setEnabled(False)
            self._buttonStepFirst.setEnabled(False)
            self._buttonStepLast.setEnabled(False)
            
        #UPDATE TEXT FOR TEXTEDIT "SPEED"
        self.update_speed_text()
        
        super(self.__class__, self).update()
   
    def on_pause(self, event = None):        
        '''
        The function is called when the button "Start Sim" (or "Stop Sim") is pressed.  
        If the simulation is running then:  
        -   stop the process by switching the timer in the model_view (stops the timer)
        -   enable several features in the gui for the time the simulation is not running

        If the simulation is not running then:  
        -   start the process by switching the timer in the model_view (starts the timer)
        -   disable several features in the gui for the time the simulation is running

        In addition adjust the model (delete old no more relevant actions for the robots 
        and set the new max number of steps equal to the current step).

        At the end update the widget.
        '''
        if self._model_view is None:
            return
        if self._model_view.get_model() is None:
            return

        self._sim_started_once = True
        self._sim_running = not self._sim_running
        
        #IF SIMULATION IS RUNNING
        if self._sim_running:
            self._model_view.get_model().set_init_step(True)
            self.parent().parent()._loadfiles_widget.setEnabled(False)
            self.parent().parent()._zoom_widget._checkbox3.setEnabled(False)
            self.parent().parent()._others_widget._buttonReset.setEnabled(False)
            self.parent().parent()._others_widget._buttonClear.setEnabled(False)
            #disable drag&drop for all graphic elements in model view
            self._model_view.get_model().set_editable(False) #self._model_view.enable_drag(not self._sim_running)

        #IF SIMULATION IS NOT RUNNING
        else:
            self.parent().parent()._loadfiles_widget.setEnabled(True)
            self.parent().parent()._zoom_widget._checkbox3.setEnabled(True)
            self.parent().parent()._others_widget._buttonReset.setEnabled(True)
            self.parent().parent()._others_widget._buttonClear.setEnabled(True)
            self._model_view.get_model().set_editable(self.parent().parent()._zoom_widget._checkbox3.isChecked())

        #RESET MAX STEP NUMBER
        self._model_view.get_model().set_num_steps(self._model_view.get_model().get_current_step())
        #DELETE THOSE ACTIONS OF ROBOTS WHICH NO LONGER RELEVANT
        self._model_view.get_model().clear_older_actions()

        #SWITCH THE TIMER: START or STOP (starts or stops the simulation)
        self._model_view.switch_timer()

        #UPDATE SimulationsWidget
        self.update()
        
    def on_update(self, event = None):
        '''
        The function is called when the button ">" is pressed.  
        Stops the timer and performs one step for every item in the model.
        '''
        if self._model_view is not None:
            self._model_view.stop_timer()
            self._model_view.do_model()

    def on_undo(self, event = None):
        '''
        The function is called when the button "<" is pressed.  
        Stops the timer and performs one step back for every item in the model.
        '''
        if self._model_view is not None:
            self._model_view.stop_timer()
            self._model_view.undo_model()

    def on_restart(self):
        '''
        The function is called when the button "|<" is pressed.  
        Stops the timer and sets every item in the model to its initial state.
        '''
        if self._model_view is not None:
            self._model_view.stop_timer()
            self._model_view.get_model().restart()

    def on_skip_to_end(self):
        '''
        The function is called when the button ">|" is pressed.  
        Stops the timer and performs a step forward for every item in the model
        till the last step is reached.
        '''
        if self._model_view is not None:
            self._model_view.stop_timer()
            self._model_view.get_model().skip_to_end()

    def on_speed_up(self):
        '''
        The function is called when the button "UP" is pressed.  
        Increase the speed of the simulation by reducing the timer in the model view
        '''
        if self._model_view is not None:
            self._model_view.speed_up_timer()
            self.update_speed_text()
            
    def on_speed_down(self):
        '''
        The function is called when the button "DOWN" is pressed.  
        Decreases the speed of the simulation by increasing the timer in the model view
        '''
        if self._model_view is not None:
            self._model_view.speed_down_timer()
            self.update_speed_text()
   
    def update_speed_text(self):
        '''
        @private
        Additional help function for setting up the right text in QTextEdit 
        for marking the speed of the simulation.
        min-speed:      x
        low-speed:      xx
        average speed:  xxx
        high-speed:     xxxx
        max-speed:      xxxxx
        '''
        text = ""
        for i in range (0,self._model_view.get_timer_scale()):
            text = text + "*"
        self._textSpeed.setText(text)  
        #alternative: self.textSpeed.setText(str(self._model_view.get_timer_scale()))


class ZoomWidget(QWidget):
    '''
    This widget offers following elements:
    -   buttons "Zoom +" and "Zoom -" for zooming in and out the modelview (visualization of the simulation)  
    -   checkbox "DRIVEN PATH" for showing the driven path by the robots 
    -   checkbox "FUTURE PATH" for showing the calculated path for the robots
    -   checkbox "Drag&Drop" to enable/disable the moving of visual elements in the simulation by the user
    '''
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setFixedHeight(160)

        self._model_view = None

        self._groupbox = QGroupBox("Visual",self)
        self._groupbox.setGeometry(20, 10,170,150)
        self._verticalLayout = QVBoxLayout()
        self._verticalLayout.setAlignment(Qt.AlignCenter)

        self._horizontalLayout = QHBoxLayout()
        self._verticalLayout.addLayout(self._horizontalLayout)
 
        #---------------------          Button: "Zoom +"          ---------------------
        self._buttonZoomIn = QPushButton("Zoom +", self)
        self._buttonZoomIn.clicked.connect(lambda: self.on_zoom(100))
        self._horizontalLayout.addWidget(self._buttonZoomIn)

        #---------------------          Button: "Zoom -"          ---------------------
        self._buttonZoomOut = QPushButton("Zoom -", self)
        self._buttonZoomOut.clicked.connect(lambda: self.on_zoom(-100))
        self._horizontalLayout.addWidget(self._buttonZoomOut)
       
        #---------------------  Checkbox: draw  Path              ---------------------
        self._checkbox1 = QCheckBox()
        self._checkbox1.setLayoutDirection(Qt.RightToLeft)
        self._checkbox1.setChecked(False)
        self._checkbox1.setText("DRIVEN PATH:       ")
        self._checkbox1.stateChanged.connect(lambda:self.checkbox_draw_path())
        self._verticalLayout.addWidget(self._checkbox1)

        #---------------------  Checkbox: draw future Path       ---------------------
        self._checkbox2 = QCheckBox()
        self._checkbox2.setLayoutDirection(Qt.RightToLeft)
        self._checkbox2.setChecked(True)
        self._checkbox2.setText("FUTURE PATH:       ")
        self._checkbox2.stateChanged.connect(lambda:self.checkbox_draw_future_path())
        self._verticalLayout.addWidget(self._checkbox2)
       
        #---------------------  Checkbox: enable Drag & Drop       ---------------------
        self._checkbox3 = QCheckBox()
        self._checkbox3.setLayoutDirection(Qt.RightToLeft)
        self._checkbox3.setChecked(True)
        self._checkbox3.setText('DRAG && DROP:      ')
        self._checkbox3.stateChanged.connect(lambda:self.checkbox_enable_drag())
        self._verticalLayout.addWidget(self._checkbox3)
        
        self._groupbox.setLayout(self._verticalLayout)
        self.setEnabled(False)
        
    def set_model_view(self, model_view):
        self._model_view = model_view
        
    def on_zoom(self, zoom):
        '''
        The function is called when the button "Zoom +" or "Zoom -" is pressed.  
        Changes the view of the modelview(QGraphicsView) according to the zoom value.

        :param zoom: float - scaling factor for modelview (QGraphicsView)
        '''
        if self._model_view is not None:
            self._model_view.zoom_event(zoom)
        
    def checkbox_draw_path(self):
        '''
        The function is called when the checkbox "DRIVEN PATH" is enabled/disabled.
        (enables/disables the painting of the driven path for all the robots in the model)
        '''
        if self._model_view is None:
            return
        if self._model_view.get_model() is None:
            return
        self._model_view.get_model().mapf_robots_enable_draw_start(self._checkbox1.isChecked())
        self._model_view.get_model().mapf_robots_enable_draw_path(self._checkbox1.isChecked())
        self._model_view.update()

    def checkbox_draw_future_path(self):
        '''
        The function is called when the checkbox "FUTURE PATH" is enabled/disabled.
        (enables/disables the painting of the calculated path for all the robots in the model)
        '''
        if self._model_view is None:
            return        
        if self._model_view.get_model() is None:
            return
        self._model_view.get_model().mapf_robots_enable_draw_future_path(self._checkbox2.isChecked())
        self._model_view.update()

    def checkbox_enable_drag(self):
        '''
        The function is called when the checkbox "DRAG&DROP" is enabled/disabled   
        (enables/disables drag&drop for all the elements in the model)
        '''
        if self._model_view is None:
            return        
        if self._model_view.get_model() is None:
            return
        self._model_view.get_model().set_editable(self._checkbox3.isChecked())


class OthersWidget(QWidget):
    '''
    This widget offers three buttons:

    -   RESET: reset the simulation to the beginning
    -   CLEAR: delete data loaded from the .map and .scen files
    -   HELP:  create a help window for assistance purposes for the user
    '''
    def __init__(self):
        super(self.__class__, self).__init__()

        self._model_view = None

        self._groupbox = QGroupBox("",self)
        self._groupbox.setGeometry(20,0,170,150)

        self._verticalLayout = QVBoxLayout()
        
        #---------------------          Button: "Reset"          ---------------------
        self._buttonReset = QPushButton("Reset", self)
        self._buttonReset.clicked.connect(self.reset)
        self._verticalLayout.addWidget(self._buttonReset)
        
        #---------------------          Button: "Clear"          ---------------------
        self._buttonClear = QPushButton("Clear", self)
        self._buttonClear.clicked.connect(self.clear)
        self._verticalLayout.addWidget(self._buttonClear)
        
        #---------------------          Button: "Help"           ---------------------
        self._buttonHelp = QPushButton("Help", self)
        self._buttonHelp.clicked.connect(self.help)
        self._verticalLayout.addWidget(self._buttonHelp)

        self._groupbox.setLayout(self._verticalLayout)

    def set_model_view(self, model_view):
        self._model_view = model_view

    def reset(self, event = None):
        '''
        This function resets the simulation to the initial state.
        The function is called when the button "RESET" is pressed.
        '''
        if self._model_view is not None:
            self._model_view.get_model().mapf_reset_map_scene()    
        
    def clear(self, event = None):
        '''
        This function deletes the data loaded from the .map und .scen files and resets everything else.
        The function is called when the button "CLEAR" is pressed.
        '''
        #clear the model, update the ModelView and Robot Table
        self.parent().parent()._loadfiles_widget._model.clear()

        #disable several gui elements until data is loaded
        self.parent().parent()._loadfiles_widget._buttonLoadScene.setEnabled(False)
        self.parent().parent()._loadfiles_widget._l1.setEnabled(False)
        self.parent().parent()._loadfiles_widget._sp.setEnabled(False)
        self.parent().parent()._loadfiles_widget._l2.setEnabled(False)
        self.parent().parent()._loadfiles_widget._sp2.setEnabled(False)

        #disable the simulation and zoom-widgets and change the title of the window
        self.parent().parent()._sim_widget.setEnabled(False)
        self.parent().parent()._zoom_widget.setEnabled(False)
        self.parent().parent().parent().parent().setWindowTitle('MAPF Simulator')

    def help(self, event = None):
        '''
        This function creates a help window for the user for assistance purposes.
        The function is called when the button "HELP" is pressed.
        '''
        dlg = QMessageBox(self)
        dlg.setWindowTitle("HELP")
        text = """  
        --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        STEP 1: Load a .map file   --->   BUTTON "Open Map File"\n\n
        --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        STEP 2: [OPTIONAL] Select the buckets, per default all buckets will be loaded\n\n
        --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        STEP 3: Load a .scen file   --->   BUTTON "Open Agents File"\n\n
        --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        STEP 4: [OPTIONAL] Change the number of robots\n\n
        --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        STEP 5: Start the CONTROLLER by running the following command in the terminal:\n
                "ros2 run mapf_controller controller"\n\n
        --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        STEP 6: Start a PATHPLANNER by running one of the following commands in the terminal:\n
                "ros2 run mapf_pathplanner_cbs pathplanner"    or
                "ros2 run mapf_pathplanner_cpp pathplanner"\n\n
        --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        STEP 7: Start the simulation   --->   BUTTON "Start Sim"\n\n            
        --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        """
        dlg.setText(text)
        dlg.findChild(QGridLayout).setColumnMinimumWidth(1,len(dlg.text())+100)
        button = dlg.exec()
        
    

'''

class VizModelWidget(VizWidget):
    def __init__(self):
        super(VizModelWidget, self).__init__()
        self._model = None
 
    def update_model(self):
        pass

    def update_step(self):
        pass

    def set_model(self, model):
        self._model = model
        if self._model is not None:
            self._update_model



class InstanceFileTree(QTreeView):
    def __init__(self, widget):
        super(InstanceFileTree, self).__init__(widget)
        self._menu = QMenu()
        self.setContextMenuPolicy(Qt.DefaultContextMenu)
        self._parser = None

    def contextMenuEvent(self, event):
        self._menu.clear()

        action = QAction('load instance', self)
        action.setStatusTip('Load the selected instance. Delete the current model.')
        action.triggered.connect(lambda: self._load_selected(clear = True, clear_actions = False, ground = True))
        self._menu.addAction(action)

        action = QAction('load plan', self)
        action.setStatusTip('Load the selected plan. Delete actions but keeps the model.')
        action.triggered.connect(lambda: self._load_selected(clear = False, clear_actions = True, ground = True))
        self._menu.addAction(action)

        action = QAction('parse file', self)
        action.setStatusTip('Load the selected file and adds all atoms to the current model.')
        action.triggered.connect(lambda: self._load_selected(clear = False, clear_actions = False, ground = True))
        self._menu.addAction(action)

        action = QAction('load file', self)
        action.setStatusTip('Load the selected file and add it to the parser. Adds nothing to the current model.')
        action.triggered.connect(lambda: self._load_selected(clear = False, clear_actions = False, ground = False))
        self._menu.addAction(action)

        self._menu.popup(self.mapToGlobal(QPoint(event.x(),event.y())))

    def _load_selected(self, clear = True, clear_actions = False, ground = True):
        indexes = self.selectedIndexes()
        if len(indexes) != 0:
            if not self.model().isDir(indexes[0]) and not self._parser is None:
                if clear and ground:
                    self._parser.load_instance(self.model().filePath(indexes[0]))
                elif ground:
                    self._parser.parse_file(self.model().filePath(indexes[0]),
                                            clear = clear,
                                            clear_actions = clear_actions)
                else:
                    self._parser.load(self.model().filePath(indexes[0]))

    def mouseClickEvent (self, event):
        self._menu.hide()
        return super(self.__class__, self).mouseDoubleClickEvent(event)

    def mouseDoubleClickEvent (self, event):
        self._load_selected()
        return super(self.__class__, self).mouseDoubleClickEvent(event)

    def keyPressEvent (self, event):
        if event.key() == Qt.Key_Return:
            self._load_selected()
        elif event.key() == Qt.Key_W:
            self.setTreePosition(self.treePosition() + 1)
        elif event.key() == Qt.Key_S:
            self.setTreePosition(self.treePosition() - 1)
        return super(self.__class__, self).keyPressEvent(event)

    def set_parser(self, parser):
        self._parser = parser

class InstanceFileBrowser(VizWidget):
    def __init__(self, directory = None):
        super(InstanceFileBrowser, self).__init__()
        self._tree_model = QFileSystemModel()
        self._tree_model.setRootPath(QDir.rootPath())

        self._tree = InstanceFileTree(self)
        self._tree.setModel(self._tree_model)
        self._tree.move(0,20)
        if directory is not None:
            self._tree.setRootIndex(self._tree_model.index(directory))
        else:
            self._tree.setRootIndex(self._tree_model.index(QDir.rootPath()))

        self._parser = None
        str_filter = mapf_simulation.configuration.config.get('visualizer', 'file_filters')
        str_filter = str_filter.replace(' ', '')
        self._tree_model.setNameFilters(str_filter.split(','))

    def resizeEvent (self, event):
        self._tree.setColumnWidth(0,self.width())
        self._tree.resize(self.width(), self.height()-20)

    def set_parser(self, parser):
        self._tree.set_parser(parser)

class TimestepWidget(QTextEdit):
    def __init__(self):
        super(self.__class__, self).__init__()
        self._model_view = None
        self.setReadOnly(True)

    def set_model_view(self, model_view):
        self._model_view = model_view
        self._model_view.get_model().add_window(self)
        self.update()

    def resizeEvent(self, event):
        font_size = min(event.size().width() / 12, event.size().height() / 2)
        if font_size > 20:
            font_size = 20
        elif font_size < 1:
            font_size = 1
        self.setFontPointSize(font_size)
        self.update()
        super(self.__class__, self).resizeEvent(event)

    def update(self):
        if self._model_view is None:
            return
        if self._model_view.get_model() is None:
            return
        step = self._model_view.get_model().get_current_step()
        speed_up = 1/self._model_view.get_timer_speedup()
        self.setText('current step: '
                                        + str(step)
                                        + '\nspeed: '
                                        + '%.2f' % speed_up)
        super(self.__class__, self).update()

class ControlWidget(QWidget):
    def __init__(self):
        super(self.__class__, self).__init__()

        self._model_view = None
        self._timestep_widget = None
        key_string = mapf_simulation.configuration.config.get('controls', 'do_backstep')
        self.create_button('<', (20, 0),
                            self.on_undo,
                            QKeySequence(key_string),
                            'Do one step\n[' + key_string + ']')
        key_string = mapf_simulation.configuration.config.get('controls', 'do_step')
        self.create_button('>', (100, 0),
                            self.on_update,
                            QKeySequence(key_string),
                            'Undo one step\n[' + key_string + ']')
        key_string = mapf_simulation.configuration.config.get('controls', 'pause')
        self._pause_button = self.create_button('|>', (60, 0),
                                                self.on_pause,
                                                QKeySequence(key_string),
                                                'Pause/Unpause the visualisation\n[' + key_string + ']')[0]
        key_string = mapf_simulation.configuration.config.get('controls', 'step_slow_down')
        self.create_button('|<|<', (20, 35),
                            lambda: self.on_speed_up(-0.1),
                            QKeySequence(key_string),
                            'Slow down the visualisation\n[' + key_string + ']')
        key_string = mapf_simulation.configuration.config.get('controls', 'step_speed_up')
        self.create_button('|>|>', (60, 35),
                            lambda: self.on_speed_up(0.0909),
                            QKeySequence(key_string),
                            'Speed up the visualisation\n[' + key_string + ']')
        self.create_button('|<', (100, 35),
                            self.on_restart, None,
                            'Restart the visualisation')
        self.create_button('>|', (140, 35),
                            self.on_skip_to_end, None,
                            'Skip to the end of the visualisation')

        key_string = mapf_simulation.configuration.config.get('controls', 'zoom_in')
        self.create_button('+', (20, 75),
                            lambda: self.on_zoom(100),
                            QKeySequence(key_string),
                            'Zoom in\n[' + key_string + ']')
        key_string = mapf_simulation.configuration.config.get('controls', 'zoom_out')
        self.create_button('-', (60, 75),
                            lambda: self.on_zoom(-100),
                            QKeySequence(key_string),
                            'Zoom out\n[' + key_string + ']')
        self.setFixedHeight(105)

    def create_button(self, name, pos, function, shortcut, tooltip = None):
        button = QPushButton(name, self)
        button.move(pos[0], pos[1])
        button.resize(30,30)
        button.clicked.connect(function)
        if tooltip is not None:
            button.setToolTip(tooltip)

        action = None
        if shortcut is not None:
            action = QAction(self)
            action.setShortcut(shortcut)
            action.setShortcutContext(Qt.ApplicationShortcut)
            action.triggered.connect(function)
            self.addAction(action)
        return (button, action)

    def set_model_view(self, model_view):
        self._model_view = model_view
        self._model_view.get_model().add_window(self)
        self.update()

    def set_timestep_widget(self, timestep_widget):
        self._timestep_widget = timestep_widget

    def on_update(self, event = None):
        if self._model_view is not None:
            self._model_view.stop_timer()
            self._model_view.update_model(True)

    def on_pause(self, event = None):
        if self._model_view is not None:
            self._model_view.switch_timer()
        self.update()

    def on_undo(self, event = None):
        if self._model_view is not None:
            self._model_view.stop_timer()
            self._model_view.undo_model()

    def on_speed_up(self, speed_up):
        if self._model_view is not None:
            self._model_view.speed_up_timer(speed_up)
            self._timestep_widget.update()

    def on_restart(self):
        if self._model_view is not None:
            self._model_view.get_model().restart()

    def on_skip_to_end(self):
        if self._model_view is not None:
            model = self._model_view.get_model().skip_to_end()

    def on_zoom(self, zoom):
        if self._model_view is not None:
            self._model_view.zoom_event(zoom)

    def resizeEvent(self, event):
        super(self.__class__, self).resizeEvent(event)

    def update(self):
        if self._model_view is None:
            return
        if self._model_view.get_model() is None:
            return
        if self._model_view.is_timer_running():
            self._pause_button.setText('|>')
        else:
            self._pause_button.setText('||')
        super(self.__class__, self).update()

class OccursWidget(QTextEdit):
    def __init__(self):
        super(self.__class__, self).__init__()
        self._model = None
        self.setReadOnly(True)
        self.resize(400, 400)
        self.setWindowTitle('Occurs')
        self.setFontPointSize(14)

    def update(self):
        action_lists = []
        max_len = 0
        text = ''
        scroll_bar = self.verticalScrollBar()
        pos = scroll_bar.sliderPosition()
        for agent in self._model.iterate_graphic_items():
            action_list = agent.to_occurs_str()
            if len(action_list) != 0:
                action_lists.append(action_list)
                max_len = max(max_len, len(action_list))

        join_list = []      
        for i in range(0, max_len):
            if i == self._model.get_current_step():
                join_list.append('<font color = green>')


            for action_list in action_lists:
                if len(action_list) > i:
                    if action_list[i] is not None:
                        join_list.append(action_list[i])


            if i == self._model.get_current_step():
                join_list.append('</font>')
                text = text + ''.join(join_list)
                join_list = []
                self.setHtml(text)
                self.moveCursor(QTextCursor.End)
                pos = self.verticalScrollBar().sliderPosition()
        text = text + ''.join(join_list)

        self.setHtml(text)
        scroll_bar.setSliderPosition(pos)
        super(self.__class__, self).update()

    def set_model(self, model):
        self._model = model
        if self._model is not None:
            self._model.add_window(self)
'''

'''
class ControlSplitter(VizWidget):
    def __init__(self):
        super(ControlSplitter, self).__init__()
        self._splitter = QSplitter(Qt.Vertical, self)
        self._timestep_widget = TimestepWidget()
        self._control_widget = ControlWidget()
        self._occurs_widget = OccursWidget()
        self._splitter.addWidget(self._timestep_widget)
        self._splitter.addWidget(self._control_widget)
        self._splitter.addWidget(self._occurs_widget)
        self._control_widget.set_timestep_widget(self._timestep_widget)
        self._splitter.setSizes([40, 105, self.size().height() - 145])
        self._splitter.show()
        self._timestep_widget.show()
        self._control_widget.show()
        self._occurs_widget.show()

    def set_model_view(self, model_view):
        self._timestep_widget.set_model_view(model_view)
        self._control_widget.set_model_view(model_view)

    def set_model(self, model):
        self._occurs_widget.set_model(model)

    def resizeEvent(self, event):
        self._splitter.resize(event.size())

class ServerDialog(QWidget):
    def __init__(self, title, host, port, socket):
        super(ServerDialog, self).__init__()

        self._socket = socket

        self.setWindowTitle(title)
        self._host_text = QLineEdit(self)
        self._host_text.setText('host: ')
        self._host_text.setReadOnly(True)
        self._port_text = QLineEdit(self)
        self._port_text.setText('port: ')
        self._port_text.setReadOnly(True)
        self._host_textbox = QLineEdit(self)
        self._host_textbox.setText(host)
        self._port_textbox = QLineEdit(self)
        self._port_textbox.setText(port)
        self._ok_button = QPushButton('Ok', self)
        self._cancel_button = QPushButton('Cancel', self)
        self._function = None

        self._ok_button.clicked.connect(self.on_ok)
        self._cancel_button.clicked.connect(self.on_cancel)
        self.setFixedSize(320, 110)
        self._host_text.resize(140, 30)
        self._port_text.resize(140, 30)
        self._host_textbox.resize(140,30)
        self._port_textbox.resize(140,30)
        self._host_text.move(0,0)
        self._port_text.move(0,40)
        self._host_textbox.move(140,0)
        self._port_textbox.move(140,40)
        self._ok_button.move(20,80)
        self._cancel_button.move(140,80)

    def set_address(self, host, port):
        self._host_textbox.setText(str(host))
        self._port_textbox.setText(str(port))

    def on_ok(self, event = None):
        try:
            if (self._socket.connect(self._host_textbox.text(),
                    int(self._port_textbox.text())) < 0):
                return
            self._socket.run()
        except(ValueError, TypeError):
            print('the port must be an integer value')
        self.hide()
    def on_cancel(self, event):
        self.hide()

class InitServerDialog(QWidget):
    def __init__(self, socket_name, command, port, socket):
        super(self.__class__, self).__init__()

        self._socket = socket

        self.setWindowTitle('Initialize ' + str(socket_name))
        self._textbox = QLineEdit(self)
        self._textbox.setText(command)

        self._port_text = QLineEdit(self)
        self._port_text.setText('port: ')
        self._port_text.setReadOnly(True)
        self._port_text.resize(140, 30)
        self._port_text.move(0,35)
        self._port_textbox = QLineEdit(self)
        self._port_textbox.setText(port)
        self._port_textbox.resize(140,30)
        self._port_textbox.move(140,35)

        self._ok_button = QPushButton('Ok', self)
        self._cancel_button = QPushButton('Cancel', self)
        self._function = None

        self._ok_button.clicked.connect(self.on_ok)
        self._cancel_button.clicked.connect(self.on_cancel)
        self.setFixedSize(320, 100)
        self._textbox.resize(280,30)
        self._ok_button.move(20,70)
        self._cancel_button.move(140,70)

    def on_ok(self, event):
        self.hide()
        try:
            self._socket.run_script(
                    self._textbox.text().replace('__dir__', os.path.dirname(sys.argv[0])),
                    int(self._port_textbox.text()))
        except(ValueError):
            print('the port must be an integer value')
    def on_cancel(self, event):
        self.hide()

class GridSizeDialog(QWidget):
    def __init__(self):
        super(self.__class__, self).__init__()

        self._model = None
        self.model_view = None
        self._item_window = None
        self.setWindowTitle('Grid size')
        self._width_text = QLineEdit(self)
        self._width_text.setText('x: ')
        self._width_text.setReadOnly(True)
        self._height_text = QLineEdit(self)
        self._height_text.setText('y: ')
        self._height_text.setReadOnly(True)
        self._width_textbox = QLineEdit(self)
        self._width_textbox.setText('0')
        self._height_textbox = QLineEdit(self)
        self._height_textbox.setText('0')
        self._checkbox = QCheckBox('enable new nodes', self)
        self._ok_button = QPushButton('Ok', self)
        self._cancel_button = QPushButton('Cancel', self)
        self._function = None

        self._ok_button.clicked.connect(self.on_ok)
        self._cancel_button.clicked.connect(self.on_cancel)
        self.setFixedSize(280, 150)
        self._width_text.resize(140, 30)
        self._height_text.resize(140, 30)
        self._width_textbox.resize(140,30)
        self._height_textbox.resize(140,30)
        self._width_text.move(0,0)
        self._height_text.move(0,40)
        self._width_textbox.move(140,0)
        self._height_textbox.move(140,40)
        self._checkbox.move(0, 80)
        self._ok_button.move(20,120)
        self._cancel_button.move(140,120)

    def set_model(self, model):
        self._model = model

    def set_model_view(self, model_view):
        self._model_view = model_view

    def on_ok(self, event):
        self.hide()
        if self._model is None:
            return
        if not self._model.get_editable():
            return
        try:
            self._model.set_grid_size(int(self._width_textbox.text()),
                                      int(self._height_textbox.text()),
                                      self._checkbox.isChecked())
        except ValueError:
            print('x and y must be interger values')
        if self._model_view is not None:
            self._model_view.update()
            self._model_view.resize_to_fit()

    def on_cancel(self, event):
        self.hide()

class OrderDialog(QWidget):
    def __init__(self):
        super(self.__class__, self).__init__()

        self._model = None
        self.setWindowTitle('Add order')
        self.setFixedSize(280, 190)

        self._id_text = QLineEdit(self)
        self._id_text.setText('order id: ')
        self._id_text.setReadOnly(True)
        self._id_text.move(0,0)
        self._id_text.resize(140, 30)
        self._id_textbox = QLineEdit(self)
        self._id_textbox.setText('0')
        self._id_textbox.move(140,0)
        self._id_textbox.resize(140,30)

        self._product_id_text = QLineEdit(self)
        self._product_id_text.setText('product id: ')
        self._product_id_text.setReadOnly(True)
        self._product_id_text.move(0,40)
        self._product_id_text.resize(140, 30)
        self._product_id_textbox = QLineEdit(self)
        self._product_id_textbox.setText('0')
        self._product_id_textbox.move(140,40)
        self._product_id_textbox.resize(140,30)

        self._product_amount_text = QLineEdit(self)
        self._product_amount_text.setText('product amount: ')
        self._product_amount_text.setReadOnly(True)
        self._product_amount_text.move(0,80)
        self._product_amount_text.resize(140, 30)
        self._product_amount_textbox = QLineEdit(self)
        self._product_amount_textbox.setText('0')
        self._product_amount_textbox.move(140,80)
        self._product_amount_textbox.resize(140,30)

        self._ps_text = QLineEdit(self)
        self._ps_text.setText('picking station id: ')
        self._ps_text.setReadOnly(True)
        self._ps_text.move(0,120)
        self._ps_text.resize(140, 30)
        self._ps_textbox = QLineEdit(self)
        self._ps_textbox.setText('0')
        self._ps_textbox.move(140,120)
        self._ps_textbox.resize(140,30)

        self._ok_button = QPushButton('Ok', self)
        self._ok_button.move(20,160)
        self._ok_button.clicked.connect(self.on_ok)
        self._cancel_button = QPushButton('Cancel', self)
        self._cancel_button.move(140,160)
        self._cancel_button.clicked.connect(self.on_cancel)

    def on_ok(self, event):
        if self._model is None:
            return
        try:
            order = self._model.get_item(item_kind = 'order',
                        ID = self._id_textbox.text(),
                        create = True,
                        add_immediately = self._model.get_editable())
            if not self._model.get_editable() and self._model.contains(order):
                print('commited orders can not be edited')
            else:
                order.set_station_id(self._ps_textbox.text())
                order.add_request(self._product_id_textbox.text(),
                                int(self._product_amount_textbox.text()))
                self._model.update_windows()
        except:
            print('failed to add new request')
            return
        self.hide()

    def on_cancel(self, event):
        self.hide()

    def set_model(self, model):
        self._model = model

class OrderTable(QTableWidget):
    def __init__(self):
        super(self.__class__, self).__init__()
        self._model = None
        self.setColumnCount(6)
        self.setHorizontalHeaderLabels(['Order ID', 'Picking Station', 'Product', 'Product Amount', 'Delivered', 'Open'])
        self.setMinimumHeight(60)

        self.setContextMenuPolicy(Qt.DefaultContextMenu)
        self._menu = QMenu()
        self._menu.setParent(self)

        self._order_dialog = OrderDialog()

    def contextMenuEvent(self, event):
        order = None
        request = None
        item = self.itemAt(event.x(), event.y())
        if item is not None:
            row = self.row(item)
            count = 0
            for order2 in self._model.filter_items(item_kind = 'order'):
                if count <= row:
                    for request2 in order2.iterate_requests():
                        if count == row:
                            if self._model.get_editable():
                                request = request2
                                order = order2
                            count += 1
                            break
                        count += 1

            if count <= row:
                for order2 in self._model.filter_items(
                                    item_kind = 'order',
                                    return_non_buffered = False,
                                    return_buffered = True):
                    if count <= row:
                        for request2 in order2.iterate_requests():
                            if count == row:
                                request = request2
                                order = order2
                                break
                            count += 1

        self._menu.clear()
        action = QAction('add order', self)
        action.setShortcut('Ctrl + O')
        action.setStatusTip('Adds a new order')
        action.triggered.connect(self.add_order)
        self._menu.addAction(action)

        if request is not None:
            action = QAction('remove request', self)
            action.setShortcut('Ctrl + R')
            action.setStatusTip('Removes the selected request')
            action.triggered.connect(lambda: self.remove_request(order, request.product_id))
            self._menu.addAction(action)

        if order is not None:
            action = QAction('remove order', self)
            action.setShortcut('Ctrl + O')
            action.setStatusTip('Removes the selected order')
            action.triggered.connect(lambda: self.remove_order(order))
            self._menu.addAction(action)

        self._menu.popup(QPoint(event.x(),event.y()))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton or event.button() == Qt.MiddleButton:
            self._menu.hide()
        super(self.__class__, self).mousePressEvent(event)

    def set_model(self, model):
        self._model = model
        self._order_dialog.set_model(model)

    def add_order(self):
        self._menu.hide()
        self._order_dialog.show()

    def remove_order(self, order):
        self._menu.hide()
        self._model.remove_item(order)
        self._model.update_windows()

    def remove_request(self, order, product_id):
        self._menu.hide()
        order.remove_request(product_id)
        if order.get_num_requests() == 0:
            self._model.remove_item(order)
        self._model.update_windows()

class OrderWidget(VizWidget):
    def __init__(self):
        super(self.__class__, self).__init__()
        self._splitter = QSplitter(Qt.Vertical, self)
        self._model = None
        self._table = OrderTable()
        self._table.setSortingEnabled(True)
        self.setWindowTitle('Orders')
        self._do_update = False
        self._splitter.addWidget(self._table)

        self._control_widget = QWidget()
        self._splitter.addWidget(self._control_widget)
        self._control_widget.setFixedHeight(24)

        self._deliver_widget = QTextEdit()
        self._deliver_widget.setReadOnly(True)
        self._splitter.addWidget(self._deliver_widget)

        self._table.itemChanged.connect(self.changed_item)

        self._send_button = QPushButton('Send orders', self._control_widget)
        self._send_button.move(20,0)
        self._send_button.clicked.connect(self.on_send)
        self._discard_button = QPushButton('Discard orders', self._control_widget)
        self._discard_button.move(140,00)
        self._discard_button.clicked.connect(self.on_discard)

    def update(self):
        self._deliver_widget.clear()
        orders = self._model.filter_items(item_kind = 'order')
        orders2 = self._model.filter_items(item_kind = 'order',
                        return_non_buffered = False,
                        return_buffered = True)
        for order in orders:
            for ss in order.to_delivered_str():
                self._deliver_widget.append(ss)

        red_brush   = QBrush(QColor(200, 100, 100))
        white_brush = QBrush(QColor(255, 255, 255))
        yellow_brush = QBrush(QColor(255, 255, 100))

        count = 0
        self._do_update = True
        self._table.setSortingEnabled(False)
        for order in orders:
            for request in order.iterate_requests():
                count = count + 1
        for order in orders2:
            for request in order.iterate_requests():
                count = count + 1

        if self._model.get_editable() or self._table.rowCount() != count:
            changed_requests = False
        else:
            changed_requests = True
        self._table.setRowCount(count)

        count = 0
        for order in orders:
            for request in order.iterate_requests():
                item = self._table.item(count, 0)
                if item is not None:
                    if request.changed:
                        for ii in range(0, self._table.columnCount()):
                           self._table.item(count, ii).setBackground(red_brush)
                    else:
                        for ii in range(0, self._table.columnCount()):
                            self._table.item(count, ii).setBackground(white_brush)

                self.set_item_text(count, 0, str(order.get_id()))
                self.set_item_text(count, 1, str(order.get_station_id()))
                self.set_item_text(count, 2, str(request.product_id))
                self.set_item_text(count, 3, str(request.requested))
                self.set_item_text(count, 4, str(request.delivered))
                self.set_item_text(count, 5, str(request.requested - request.delivered))
                count = count + 1

        for order in orders2:
            for request in order.iterate_requests():

                self.set_item_text(count, 0, str(order.get_id()), True)
                self.set_item_text(count, 1, str(order.get_station_id()), True)
                self.set_item_text(count, 2, str(request.product_id), True)
                self.set_item_text(count, 3, str(request.requested), True)
                self.set_item_text(count, 4, str(request.delivered), True)
                self.set_item_text(count, 5, str(request.requested - request.delivered), True)

                for ii in range(0, self._table.columnCount()):
                    self._table.item(count, ii).setBackground(yellow_brush)

                count = count + 1
        self._table.setSortingEnabled(True)
        self._do_update = False
        super(self.__class__, self).update()

    def set_item_text(self, column, row, text, editable = False):
        item = self._table.item(column, row)
        if item is None:
            self._table.setItem(column, row, VizTableWidgetItem(text))
            item = self._table.item(column, row)
        else:
            item.setText(text)

        if (row == 1 or row == 3) and (self._model.get_editable() or editable):
            item.setFlags(Qt.ItemIsSelectable |  Qt.ItemIsEnabled | Qt.ItemIsEditable)
        else:
            item.setFlags(Qt.ItemIsSelectable |  Qt.ItemIsEnabled)

    def set_model(self, model):
        self._model = model
        if self._model is not None:
            self._model.add_window(self)
        self._table.set_model(model)

    def on_send(self):
        if self._model is None:
            return
        self._model.accept_new_items(['order'])
        self.update()

    def on_discard(self):
        if self._model is None:
            return
        self._model.discard_new_items(['order'])
        self.update()

    def resizeEvent(self, event):
        self._splitter.resize(event.size().width(), event.size().height())

    def changed_item(self, item):
        if self._do_update:
            return

        value = 0
        order_id = self._table.item(item.row(), 0).text()
        column = item.column()
        order = self._model.get_item('order', order_id)
        if order is None:
            return
        try:
            value = int(item.text())
        except ValueError as err:
            print(err)
            self.update()
            return

        if column == 1:
            order.set_station_id(value)
        elif column == 3:
            order.set_requested_amount(self._table.item(item.row(), 2).text(), value)
        self.update()

class ProductDialog(QWidget):
    def __init__(self):
        super(self.__class__, self).__init__()

        self._shelf = None
        self._product_window = None
        self.setWindowTitle('Add product')
        self._id_text = QLineEdit(self)
        self._id_text.setText('product id: ')
        self._id_text.setReadOnly(True)
        self._count_text = QLineEdit(self)
        self._count_text.setText('product count: ')
        self._count_text.setReadOnly(True)
        self._id_textbox = QLineEdit(self)
        self._id_textbox.setText('0')
        self._count_textbox = QLineEdit(self)
        self._count_textbox.setText('0')
        self._ok_button = QPushButton('Ok', self)
        self._cancel_button = QPushButton('Cancel', self)
        self._function = None

        self._ok_button.clicked.connect(self.on_ok)
        self._cancel_button.clicked.connect(self.on_cancel)
        self.setFixedSize(320, 110)
        self._id_text.resize(140, 30)
        self._count_text.resize(140, 30)
        self._id_textbox.resize(140,30)
        self._count_textbox.resize(140,30)
        self._id_text.move(0,0)
        self._count_text.move(0,40)
        self._id_textbox.move(140,0)
        self._count_textbox.move(140,40)
        self._ok_button.move(20,80)
        self._cancel_button.move(140,80)

    def set_shelf(self, shelf):
        self._shelf = shelf

    def set_product_window(self, product_window):
        self._product_window = product_window

    def on_ok(self, event):
        self.hide()
        if self._shelf is None:
            return
        try:
            self._shelf.add_product(int(self._id_textbox.text()), int(self._count_textbox.text()))
        except ValueError:
            print('the product id and the product counts must be integer values')
        self._product_window.update()

    def on_cancel(self, event):
        self.hide()

class ProductWindow(VizWidget):
    def __init__(self):
        super(ProductWindow, self).__init__()
        self._table = ProductTable()
        self._table.setParent(self)
        self._table.move(0, 20)

    def set_model(self, model):
        self._table.set_model(model)

    def update(self):
        self._table.update()

    def resizeEvent(self, event):
        self._table.resize(event.size().width(), event.size().height() - 20)

class ProductTable(QTreeWidget):
    def __init__(self):
        super(ProductTable, self).__init__()
        self._model = None
        self.setWindowTitle('Products')

        self.setContextMenuPolicy(Qt.DefaultContextMenu)
        self._menu = QMenu()
        self._menu.setParent(self)
        self._product_dialog = ProductDialog()
        self._product_dialog.set_product_window(self)
        self.setHeaderLabels(['product ID', 'count', 'removed'])
        self.resize(400, 200)

    def set_model(self, model):
        self._model = model
        self.update()
        if self._model is not None:
            self._model.add_window(self)

    def update(self):
        if self._model == None:
            return
        expanded_items = []
        for ii in range(0, self.topLevelItemCount()):
            item = self.topLevelItem(ii)
            if item.isExpanded():
                expanded_items.append(ii)

        self.clear()

        for shelf in self._model.filter_items(item_kind = 'shelf'):
            tree_item = QTreeWidgetItem(['Shelf(' + str(shelf.get_id()) + ')'])
            self.addTopLevelItem(tree_item)
            for product in shelf.iterate_products():
                temp_item = QTreeWidgetItem([str(product[0]), str(product[1]), str(product[2])])
                tree_item.addChild(temp_item)
        for item in expanded_items:
            self.expandItem(self.topLevelItem(item))

    def show(self):
        self.update()
        return super(self.__class__, self).show()

    def contextMenuEvent(self, event):

        if not self._model.get_editable():
            return
        item = self.itemAt(event.x(),event.y())
        if item is None:
            return
        parent = item.parent()

        #find the shelf_index
        shelf_index = 0
        shelf = None
        if parent is None:
            shelf_index = self.indexOfTopLevelItem(item)
        else:
            shelf_index = self.indexOfTopLevelItem(parent)

        #get the shelf
        count = 0
        for shelf2 in self._model.filter_items(item_kind = 'shelf'):
            if count == shelf_index:
                shelf = shelf2
                break
            else:
                count = count + 1

        self._menu.clear()
        if parent is not None:

            #get the product_id
            product_id = 0
            count = 0
            for product in shelf.iterate_products():
                if count == parent.indexOfChild(item):
                    product_id = product[0]
                    break
            else:
                count = count + 1

            action = QAction('remove product', self)
            action.setShortcut('Ctrl + R')
            action.setStatusTip('Removes a product from the selected shelf')
            action.triggered.connect(lambda: self.delete_product(shelf, product_id))
            self._menu.addAction(action)

        action = QAction('add product', self)
        action.setShortcut('Ctrl + A')
        action.setStatusTip('Adds a product to the selected shelf')
        action.triggered.connect(lambda: self.add_product(shelf))
        self._menu.addAction(action)

        self._menu.popup(QPoint(event.x(),event.y()))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton or event.button() == Qt.MiddleButton:
            self._menu.hide()
        super(self.__class__, self).mousePressEvent(event)

    def add_product(self, shelf):
        self._product_dialog.set_shelf(shelf)
        self._menu.hide()
        self._product_dialog.show()

    def delete_product(self, shelf, product_id):
        shelf.delete_product(product_id)
        self._menu.hide()
        self.update()

class TaskTable(QTableWidget):
    def __init__(self):
        super(self.__class__, self).__init__()
        self._model = None
        self.setWindowTitle('Tasks')
        self.setColumnCount(6)
        self.setHorizontalHeaderLabels(['Task ID', 'Task Group',
                                        'Task Type', 'Assigned Robot',
                                        'Checkpoint History', 'Open Checkpoints'])
        self.setSortingEnabled(True)
        self.resizeColumnsToContents()
        self.resize(self.horizontalHeader().length() + 20, 200)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

    def update(self):
        tasks = self._model.filter_items(item_kind = 'task')
        red_brush   = QBrush(QColor(200, 100, 100))
        white_brush = QBrush(QColor(255, 255, 255))
        yellow_brush = QBrush(QColor(255, 255, 100))

        changed_tasks = True
        if self._model.get_editable() or self.rowCount() != len(tasks):
            changed_tasks = False

        self.setRowCount(len(tasks))

        count = 0
        self.setSortingEnabled(False)
        for task in tasks:
            task_history = ''
            task_open = ''

            for checkpoint in task.get_checkpoint_history():
                task_history = task_history + str(checkpoint) + ', '

            for checkpoint in task.get_open_checkpoints():
                task_open = task_open + str(checkpoint) + ', '

            table_item = self.item(count, 0)
            if (table_item is not None):
                if (task.get_changed() and changed_tasks):
                    for ii in range(0, self.columnCount()):
                        self.item(count, ii).setBackground(red_brush)
                else:
                    for ii in range(0, self.columnCount()):
                        self.item(count, ii).setBackground(white_brush)

            self.set_item_text(count, 0, str(task.get_id()))
            self.set_item_text(count, 1, str(task.get_task_group()))
            self.set_item_text(count, 2, str(task.get_task_type()))
            self.set_item_text(count, 3, str(task.get_robot_id()))
            self.set_item_text(count, 4, task_history)
            self.set_item_text(count, 5, task_open)
            count += 1
        self.setSortingEnabled(True)
        super(self.__class__, self).update()

    def set_item_text(self, column, row, text):
        item = self.item(column, row)
        if item is None:
            self.setItem(column, row, VizTableWidgetItem(text))
            item = self.item(column, row)
            item.setFlags( Qt.ItemIsSelectable |  Qt.ItemIsEnabled )
        else:
            item.setText(text)

    def set_model(self, model):
        self._model = model
        if self._model is not None:
            self._model.add_window(self)

class ProgramEntry(object):
    def __init__(self, program_name, short_program_name):
        self.program_name = program_name;
        self.short_program_name = short_program_name;
        self.text_field = None;

class ParserWidget(VizWidget):
    def __init__(self):
        super(self.__class__, self).__init__()
        self._splitter = QSplitter(Qt.Vertical, self)
        self._splitter.move(0, 20)
        self._button_widget = QWidget(self._splitter)
        text_splitter = QSplitter(Qt.Horizontal, self._splitter)
        self.setWindowTitle('Parser')
        self._program_tab = QTabWidget(text_splitter)
        self._program_list = []
        self._atom_text = QTextEdit(text_splitter)
        self._atom_text.setReadOnly(True)
        self._parser = None
        self._changed = False

        self._reset_grounder_button = QPushButton('reset actions', self._button_widget)
        self._reset_grounder_button.move(0,5)
        self._reset_grounder_button.resize(140,30)
        self._reset_grounder_button.clicked.connect(self.reset_actions)

        self._reset_model_button = QPushButton('reset model', self._button_widget)
        self._reset_model_button.move(140,5)
        self._reset_model_button.resize(140,30)
        self._reset_model_button.clicked.connect(self.reset_model)

        self._reset_grounder_button = QPushButton('reset grounder', self._button_widget)
        self._reset_grounder_button.move(280,5)
        self._reset_grounder_button.resize(140,30)
        self._reset_grounder_button.clicked.connect(self.reset_grounder)

        self._reset_program_button = QPushButton('reload program', self._button_widget)
        self._reset_program_button.move(420,5)
        self._reset_program_button.resize(140,30)
        self._reset_program_button.clicked.connect(self.reset_program)

        self._reset_program_button = QPushButton('delete programs', self._button_widget)
        self._reset_program_button.move(560,5)
        self._reset_program_button.resize(140,30)
        self._reset_program_button.clicked.connect(self.reset_programs)

        self._parse_program_button = QPushButton('parse programs', self._button_widget)
        self._parse_program_button.move(0,35)
        self._parse_program_button.resize(140,30)
        self._parse_program_button.clicked.connect(self.parse_program)

        self._parse_program_button = QPushButton('delete program', self._button_widget)
        self._parse_program_button.move(140,35)
        self._parse_program_button.resize(140,30)
        self._parse_program_button.clicked.connect(self.delete_program)

        self._parse_program_button = QPushButton('add empty program', self._button_widget)
        self._parse_program_button.move(280,35)
        self._parse_program_button.resize(140,30)
        self._parse_program_button.clicked.connect(self.add_empty_program)

        self._button_widget.setFixedHeight(70)

        self.move(0,0)
        self.resize(700,600)
        self._splitter.setSizes([70, 530])

    def resizeEvent(self, event):
        self._splitter.resize(event.size().width(), event.size().height())

    def set_parser(self, parser):
        if parser is self._parser:
            return
        temp = self._parser
        self._parser = parser
        if temp is not None:
            temp.set_parser_widget(None)
        if parser is not None:
            parser.set_parser_widget(self)

    def changed(self):
        self._changed = True

    def reset_actions(self):
        if self._parser is None:
            return
        self._parser.clear_model_actions()

    def reset_model(self):
        if self._parser is None:
            return
        self._parser.clear_model()

    def reset_grounder(self):
        if self._parser is None:
            return
        self.commit_programs()
        self._parser.reset_grounder()

    def reset_program(self):
        if self._parser is None:
            return
        index = self._program_tab.currentIndex()
        if index >= 0:
            self.commit_programs()
            entry = self._program_list[index]
            self._parser.load(entry.program_name)

    def reset_programs(self):
        if self._parser is None:
            return
        self._parser.reset_programs()

    def parse_program(self):
        if self._parser is None:
            return
        self.commit_programs()
        self._parser.parse()

    def delete_program(self):
        if self._parser is None:
            return
        index = self._program_tab.currentIndex()
        if index >= 0:
            entry = self._program_list[index]
            self._parser.delete_program(entry.program_name)

    def commit_programs(self):
        if not self._changed or self._parser is None:
            return
        for entry in self._program_list:
            self._parser.set_program(entry.program_name,
                                     entry.text_field.toPlainText())
    def add_empty_program(self):
        if self._parser is None:
            return
        self._parser.add_program('new', '')

    def update(self):
        if self._parser is None:
            return

        add_entrys = [];
        delete_entrys = [];

        current = self._program_tab.currentWidget()
        for entry in self._program_list:
            delete_entrys.append(entry)

        for program_name in self._parser.list_programs():
            found = False
            for entry in self._program_list:
                if entry.program_name == program_name:
                    delete_entrys.remove(entry)
                    found = True
                    break
            if not found:
                add_entrys.append(program_name)

        for entry in delete_entrys:
            self._program_list.remove(entry)
        self._program_tab.clear()

        for entry in add_entrys:
            index = entry.rfind('/') + 1
            short_program_name = entry[index :]
            count = 1
            suffix = ''
            for entry2 in self._program_list:
                if short_program_name + suffix == entry2.short_program_name:
                    count += count
                    suffix = '(' + str(count) + ')'
            short_program_name += suffix
            self._program_list.append(ProgramEntry(entry, short_program_name))

        count = 0
        for program in self._program_list:
            if program.text_field is None:
                program.text_field = QTextEdit()
                program.text_field.textChanged.connect(self.changed)
            self._program_tab.addTab(program.text_field, program.short_program_name)
            self._program_tab.setTabToolTip(count, program.program_name)
            program.text_field.setText(self._parser.get_program(program.program_name))
            count += 1

        self._atom_text.setText(self._parser.get_str_model())
        self._edited = False

        index = self._program_tab.indexOf(current)
        if index != -1:
            self._program_tab.setCurrentIndex(index)

class EnablePathWidget(QScrollArea):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setWindowTitle('Paths')
        self._model = None
        self.resize(280, 100)

        self._area = QWidget()
        self.setWidget(self._area)

        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self._checkboxes = {}
        self._colors = {}

        self._enable_all_button = QPushButton('enable all', self._area)
        self._disable_all_button = QPushButton('disable all', self._area)
        self._enable_all_button.move(5, 5)
        self._disable_all_button.move(140, 5)
        self._enable_all_button.clicked.connect(lambda: self.on_enable_all(True))
        self._disable_all_button.clicked.connect(lambda: self.on_enable_all(False))

        self._ok_button = QPushButton('Ok', self._area)
        self._cancel_button = QPushButton('Cancel', self._area)

        self._ok_button.clicked.connect(self.on_ok)
        self._cancel_button.clicked.connect(self.on_cancel)

    def update(self):
        if self._model is None:
            return
        robots = self._model.filter_items(item_kind = 'robot')
        for key in self._checkboxes:
            self._checkboxes[key].setParent(None)
        for key in self._colors:
            self._colors[key].setParent(None)
        self._checkboxes = {}
        self._colors = {}
        y_pos = 30
        for robot in robots:
            checkbox = QCheckBox("robot" + "(" + robot.get_id() + ")", self._area)
            checkbox.move(5, y_pos)
            checkbox.setChecked(robot.get_draw_path())
            checkbox.show()
            self._checkboxes[robot.get_id()] = checkbox
            
            color_text = QTextEdit(self._area)
            color_text.setHtml('<font color = ' + robot.get_color().name() + '>' + robot.get_color().name() + '</font>')
            color_text.move(125, y_pos)
            color_text.setReadOnly(True)
            color_text.resize(160, 30)
            color_text.show()
            self._colors[robot.get_id()] = color_text
            y_pos += 40

        self._ok_button.move(5, y_pos)
        self._cancel_button.move(140, y_pos)
        self._area.resize(240, y_pos + 40)
        super(self.__class__, self).update()

    def on_enable_all(self, enable):
        for key in self._checkboxes:
            self._checkboxes[key].setChecked(enable)

    def on_ok(self):
        for key in self._checkboxes:
            robot = self._model.get_item(item_kind = 'robot', ID = key)
            if robot is not None:
                robot.set_draw_path(self._checkboxes[key].isChecked())
        self.hide()

    def on_cancel(self):
        self.hide()

    def set_model(self, model):
        self._model = model
        if self._model is not None:
            self._model.add_window(self)
        self.update()

class RobotMonitor(VizWidget):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setWindowTitle('Robot Monitor')
        self._robot = None
        self._model = None
        self._robot_textbox = QLineEdit('robot: ', self)
        self._robot_textbox.move(5,5)
        self._robot_textbox.setReadOnly(True)
        self._robot_box = QComboBox(self)
        self._robot_box.move(200,5)
        self._robot_box.activated.connect(self.on_activated)

        self._actions = QLineEdit('actions: ', self)
        self._actions.setReadOnly(True)
        self._actions.move(5, 35)
        self._position = QLineEdit('position: ', self)
        self._position.setReadOnly(True)
        self._position.move(5, 65)
        self._next_action = QLineEdit('next action: ', self)
        self._next_action.setReadOnly(True)
        self._next_action.move(5, 95)
        self._next_action.resize(400, 24)

        self._robot_actions = QTextEdit(self)
        self._robot_actions.move(5, 125)
        self._robot_actions.setReadOnly(True)
        self._robot_actions.resize(400, 200)

    def update(self):
        index = self._robot_box.currentIndex()
        self._robot_box.clear()
        if self._model is not None:
            for robot in self._model.filter_items(item_kind = 'robot'):
                self._robot_box.addItem(robot.get_id())
        self._robot_box.setCurrentIndex(index)    

        if self._robot is None:
            return
        action_list = self._robot.to_occurs_str()
        join_list = []
        cc = 0
        current_step = self._model.get_current_step()
        next_action = None
        action_count = 0
        for action in action_list:
            if action is not None:
                action_count += 1
                if cc == current_step:
                    join_list.append('<font color = green>')
                    join_list.append(action)
                    join_list.append('</font>')
                else:
                    if cc > current_step and next_action is None:
                        next_action = action
                    join_list.append(action)
            cc += 1
        self._robot_textbox.setText('robot: ' + self._robot.get_id())
        self._actions.setText('actions: ' + str(action_count) + '/' + str(self._model.get_num_steps()))
        self._position.setText('position: ' + str(self._robot.get_position()[0]) + ', ' + str(self._robot.get_position()[1]))
        if next_action is not None:
            self._next_action.setText('next action: ' + next_action)
        else:
            self._next_action.setText('next action: None')
        self._robot_actions.setHtml('\n'.join(join_list))

    def on_activated(self, text):
        robot = self._model.get_item(item_kind = 'robot', ID = self._robot_box.currentText())
        self.set_robot(robot)

    def set_robot(self, robot):
        self._robot = robot
        self.update()

    def set_model(self, model):
        self._model = model
        if self._model is not None:
            self._model.add_window(self)
        self.update()

class RobotTable(VizWidget):
    def __init__(self):
        super(RobotTable, self).__init__()
        self._table = QTableWidget(self)
        self._table.move(0, 20)
        self.setWindowTitle('Robot Table')
        self._model = None
        self._table.setColumnCount(10)
        self._table.setHorizontalHeaderLabels(['Robot ID', 'Position', 
                                               'Action number', 'Action count', 
                                               'Idle count', 'Current action', 
                                               'Next action', 'Carries', 
                                               'Current Energy', 'Max Energy'])
        self._table.itemSelectionChanged.connect(self.on_selection_changed)
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table_items = {}
        self.resize(200,200)

    def update(self):
        self._table.setSortingEnabled(False)
        red_brush   = QBrush(QColor(200, 100, 100))
        green_brush = QBrush(QColor(100, 200, 100))
        white_brush = QBrush(QColor(255, 255, 255))
        blue_brush  = QBrush(QColor(155, 155, 255))
        orange_brush = QBrush(QColor(255, 128, 0))
        ignore_first = 1
        if self._model is None:
            return
        count = 0
        robots = self._model.filter_items(item_kind = 'robot')
        if(len(robots) != self._table.rowCount()):
            self._table.clearSelection()
            self._table_items = {}
            self._table.setRowCount(len(robots))
        current_step = self._model.get_current_step()
        for robot in robots:
            cc = 0
            action_count = 0
            current_action_num = 0
            idle_count = 0
            next_action = None
            current_action =  None
            for action in robot.to_occurs_str():
                if action is not None:
                    if cc == 0:
                        ignore_first = 0
                    action_count += 1
                    if cc <= current_step:
                        current_action_num += 1
                else:
                    idle_count += 1
                if cc == current_step:
                    current_action = action
                elif cc > current_step and next_action is None:
                    next_action = action
                cc += 1

            if robot.get_state() & VIZ_STATE_DELIVER:
                brush = red_brush
            elif(robot.get_current_energy() < 0 or 
                (robot.get_current_energy() > robot.get_max_energy() and 
                 robot.get_max_energy() > 0)):

                brush = orange_brush
            elif current_action is not None:
                brush = green_brush
            elif next_action is None:
                brush = blue_brush
            else:
                brush = white_brush

            self.set_item_text(count, 0, robot.get_id(), brush)
            self.set_item_text(count, 1, str(robot.get_position()[0]) + ', ' + str(robot.get_position()[1]), brush)
            self.set_item_text(count, 2, str(current_action_num), brush)
            self.set_item_text(count, 3, str(action_count), brush)
            self.set_item_text(count, 4, str(idle_count - ignore_first), brush)
            if current_action is not None:
                self.set_item_text(count, 5, current_action, brush)
            else:
                self.set_item_text(count, 5, None, brush)
            if next_action is not None:
                self.set_item_text(count, 6, next_action, brush)
            else:
                self.set_item_text(count, 6, None, brush)
            if robot.get_carries() is not None:
                self.set_item_text(count, 7, robot.get_carries().get_id(), brush)
            else:
                self.set_item_text(count, 7, "None", brush)
            self.set_item_text(count, 8, str(robot.get_current_energy()), brush)
            self.set_item_text(count, 9, str(robot.get_max_energy()), brush)
            count += 1
        self._table.setSortingEnabled(True)
        super(self.__class__, self).update()

    def on_selection_changed(self):
        robot_ids = []
        rows = []
        for index in self._table.selectedIndexes():
            row = index.row()
            if row not in rows:
                rows.append(row)
        for row in rows:
            robot_ids.append(self._table.item(row, 0).text())
        robots = self._model.filter_items(item_kind = 'robot')
        for robot in robots:
            if robot.get_id() in robot_ids:
                robot_ids.remove(robot.get_id())
                robot.set_highlighted(True)
            else:
                robot.set_highlighted(False)
        self._model.update_windows()

    def set_item_text(self, column, row, text, brush):
        if not row in self._table_items:
            self._table_items[row] = {}
        if not column in self._table_items[row]:
            self._table_items[row][column] = VizTableWidgetItem(text)
            self._table.setItem(column, row, self._table_items[row][column])
        self._table_items[row][column].setFlags( Qt.ItemIsSelectable |  Qt.ItemIsEnabled )
        self._table_items[row][column].setBackground(brush)
        self._table_items[row][column].setText(text)

    def set_model(self, model):
        self._model = model
        if self._model is not None:
            self._model.add_window(self)
        self.update()

    def resizeEvent(self, event):
        self._table.resize(event.size().width(), event.size().height() - 20)
'''
'''
        current_step = self._model.get_current_step()
        for robot in robots:
            cc = 0
            action_count = 0
            current_action_num = 0
            idle_count = 0
            next_action = None
            current_action =  None
            for action in robot.to_occurs_str():
                if action is not None:
                    if cc == 0:
                        ignore_first = 0
                    action_count += 1
                    if cc <= current_step:
                        current_action_num += 1
                else:
                    idle_count += 1
                if cc == current_step:
                    current_action = action
                elif cc > current_step and next_action is None:
                    next_action = action
                cc += 1

            if robot.get_state() & VIZ_STATE_DELIVER:
                brush = red_brush
            elif(robot.get_current_energy() < 0 or 
                (robot.get_current_energy() > robot.get_max_energy() and 
                 robot.get_max_energy() > 0)):

                brush = orange_brush
            elif current_action is not None:
                brush = green_brush
            elif next_action is None:
                brush = blue_brush
            else:
                brush = white_brush

            self.set_item_text(count, 0, robot.get_id(), brush)
            self.set_item_text(count, 1, str(robot.get_position()[0]) + ', ' + str(robot.get_position()[1]), brush)
            self.set_item_text(count, 2, str(current_action_num), brush)
            self.set_item_text(count, 3, str(action_count), brush)
            self.set_item_text(count, 4, str(idle_count - ignore_first), brush)
            if current_action is not None:
                self.set_item_text(count, 5, current_action, brush)
            else:
                self.set_item_text(count, 5, None, brush)
            if next_action is not None:
                self.set_item_text(count, 6, next_action, brush)
            else:
                self.set_item_text(count, 6, None, brush)
            if robot.get_carries() is not None:
                self.set_item_text(count, 7, robot.get_carries().get_id(), brush)
            else:
                self.set_item_text(count, 7, "None", brush)
            self.set_item_text(count, 8, str(robot.get_current_energy()), brush)
            self.set_item_text(count, 9, str(robot.get_max_energy()), brush)
            count += 1
        self._table.setSortingEnabled(True)
        super(self.__class__, self).update()    
        '''
        