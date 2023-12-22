import os.path
import re
from mapf_simulation.model import *

#import traceback
#import mapf_simulation.configuration
#from clingo.control import Control
#from clingo.ast import ProgramBuilder, parse_string


class MAPFfileParser(object):
    '''
    This class represents the file parser for reading the .map and .scen files
    and loading the content of the files into the model!
    '''

    def __init__(self):
        self._model = None
        self._model_view = None

        #SIZE OF MAP
        self._size_x = None
        self._size_y = None

        self._buckets = []  #INDEXES OF BUCKETS
        self._start = []    #START POSITIONS OF ROBOTS
        self._goals = []    #GOAL POSITIONS OF ROBOTS 
        self._heuristics = [] #GIVEN HEURISTIC

    def set_model(self, model):
        self._model = model
    
    def set_model_view(self, model_view):
        self._model_view = model_view

    def load_map(self, filename):
        '''
        This function is used for loading the .map file.  

        The function checks first whether the file exists and can be opened. 
        Then it tries to read from the file. 
        After reading the file the function analyzes the content and the syntax.
        If the test is positive then it loads the data into the model!

        :param filename: string - the complete path to the file including the name of the file

        :return:    1, if the file was loaded successfully
                    0, otherwise
        '''
        if self._model is None:
            return 0
        if self._model_view is None:
            return 0

        #CHECK IF THE FILE CAN BE OPENED
        if not os.path.isfile(filename):
            print('ERROR: CAN NOT OPEN THE FILE: ', filename)
            return 0

        #TRY TO READ THE FILE
        print('\nSTATUS: Loading the map file: ' + filename)
        try:
            ff = open(filename)
            lines = ff.read().splitlines()
            ff.close()
        except RuntimeError as error:
            print('ERROR: RUNTIME EXCEPTION WHILE LOADING THE MAP FILE: ' + filename)
            print(error)
            return 0

        #ANALYZE THE CONTENT OF THE FILE
        if not self.analyze_file_map(lines):
            print('ERROR: WRONG SYNTAX, SKIP THE .MAP FILE!')
            return 0    

        #UPDATE MODEL: IMPORT DATA 
        self._model.mapf_update_map(os.path.basename(os.path.normpath(filename)), 
                                    self._size_x,
                                    self._size_y,
                                    lines[4:])
        
        #UPDATE MODELVIEW: IMPORT DATA + RESIZE TO FIT THE WINDOW
        self._model_view.update()
        self._model_view.resize_to_fit()

        print('STATUS: MAP FILE LOADED SUCCESSFULLY')
        return 1

    def load_scene(self, filename, input_buckets):
        '''
        This function is used for loading the .scen file.  
        
        The function checks first whether the file exists and can be opened. 
        Then it tries to read from the file. 
        After reading the file the function analyzes the content and the syntax.
        If the test is positive then the function checks further if there are conflicts between the different 
        items in the map. In the case of no conflicts the parser loads the data into the model!
        
        :param filename: string - the complete path to the file including the name of the file
        :param input_buckets: string - list of buckets to be loaded, for more details about the syntax
        read *gui.py* 

        :return:    1, if the file was loaded successfully
                    0, otherwise
        '''
        if self._model is None:
            return 0
        if self._model_view is None:
            return 0

        #CHECK IF THE FILE CAN BE OPENED
        if not os.path.isfile(filename):
            print('ERROR: CAN NOT OPEN THE FILE: ', filename)
            return 0

        #TRY TO READ THE FILE
        print('\nSTATUS: Loading the scene file: ' + filename)
        try:
            ff = open(filename)
            lines = ff.read().splitlines()
            ff.close()
        except RuntimeError as error:
            print('ERROR: RUNTIME EXCEPTION WHILE LOADING THE SCENE FILE: ' + filename)
            print(error)
            return 0

        #ANALYZE THE CONTENT OF THE FILE
        if not self.analyze_file_scene(lines):
            print('ERROR: WRONG SYNTAX, SKIP THE .SCEN FILE!')
            return 0    

        #CHECK FOR COLLISIONS BETWEEN OBSTACLES IN THE MAP AND START/GOAL POSITIONS OF THE ROBOTS
        if self.check_collisions(self._start, self._goals):
            print('ERROR: COLLISIONS OCCURED IN THE MAP, TWO NODES ARE TRYING TO OCCUPY THE SAME SPOT, SKIP THE .SCEN FILE')
            return 0    
        
        #UPDATE MODEL: IMPORT DATA 
        self._model.mapf_update_scene(os.path.basename(os.path.normpath(filename)),
                                      self._buckets, self._start, self._goals, self._heuristics, input_buckets)
        
        #UPDATE MODELVIEW: IMPORT DATA + RESIZE TO FIT THE WINDOW
        self._model_view.update()
        self._model_view.resize_to_fit()

        print('STATUS: SCENE FILE LOADED SUCCESSFULLY')
        return 1

    def analyze_file_map(self, lines):
        '''
        The function checks the syntax of the .map file. For more details about the correct syntax of the .map file look here:
        https://movingai.com/benchmarks/formats.html

        :param lines: list of strings - where every string represents a line from the .map file
        :return: **True**, if the content of the .map file is okay  
        **False**, otherwise
        '''

        print('STATUS: CHECKING THE MAP FILE SYNTAX')
        line1 = "type octile"
        line2 = 'height'
        line3 = 'width'
        line4 = 'map'

        valid_char = '.G@OTSW'
        height=width=-1

        #line counter
        count = 1

        #FOR EVERY LINE IN THE FILE
        for line in lines:

            #REMOVE WHITESPACES FROM THE END OF THE LINE
            line = line.rstrip()

            #CHECK LINE 1: SHOULD BE EQUAL TO 'type octile'
            if count == 1:
                if line1 != line:
                    print('ERROR: WRONG SYNTAX IN THE MAP FILE, LINE 1 IS NOT OK! (SHOULD BE EQUAL TO \"type octile\")')
                    return False

            #CHECK LINE 2: SHOULD START WITH 'height' and FOLLOW BY AN INTEGER BIGGER THAN 0
            if count == 2:
                words = line.split()
                if len(words) != 2:
                    print('ERROR: WRONG SYNTAX IN THE MAP FILE, LINE 2 IS NOT OK! (LESS OR MORE THAN TWO WORDS)')
                    return False

                if words[0] !=  line2:
                    print('ERROR: WRONG SYNTAX IN THE MAP FILE, LINE 2 IS NOT OK! (SHOULD START WITH \"height\")')
                    return False

                try:
                    height= int(words[1])

                except ValueError:
                    print('ERROR: WRONG SYNTAX IN THE MAP FILE, LINE 2 IS NOT OK! (HEIGHT SHOULD BE AN INTEGER NUMBER)')
                    return False

                if height<1:
                    print('ERROR: WRONG VALUE IN THE MAP FILE, LINE 2 IS NOT OK! (HEIGHT SHOULD BE AN INTEGER NUMBER BIGGER THAN 0)')
                    return False

            #CHECK LINE 3: SHOULD START WITH 'width' and FOLLOW BY AN INTEGER BIGGER THAN 0
            if count == 3:
                words = line.split()
                if len(words) != 2:
                    print('ERROR: WRONG SYNTAX IN THE MAP FILE, LINE 3 IS NOT OK! (LESS OR MORE THAN TWO WORDS)')
                    return False

                if words[0] !=  line3:
                    print('ERROR: WRONG SYNTAX IN THE MAP FILE, LINE 3 IS NOT OK! (SHOULD START WITH \"width\")')
                    return False

                try:
                    width= int(words[1])

                except ValueError:
                    print('ERROR: WRONG SYNTAX IN THE MAP FILE, LINE 3 IS NOT OK! (WIDTH SHOULD BE AN INTEGER NUMBER)')
                    return False

                if width<1:
                    print('ERROR: WRONG VALUE IN THE MAP FILE, LINE 3 IS NOT OK! (WIDTH SHOULD BE AN INTEGER NUMBER BIGGER THAN 0)')
                    return False

            #CHECK LINE 4: SHOULD BE EQUAL TO 'map'
            if count == 4:
                if line4 != line:
                    print('ERROR: WRONG SYNTAX IN THE MAP FILE, LINE 4 IS NOT OK! (SHOULD BE EQUAL TO \"map\")')
                    return False

            #CHECK EVERY NEXT LINE (AFTER LINE 4): 
            #   LENGTH OF THE LINE SHOULD BE EQUAL TO PARAMETER "width" FROM LINE 3
            #   AND SHOULD CONTAIN ONLY VALID CHARACTERS
            if count > 4:

                if len(line) != width:
                    print('ERROR: WRONG VALUE IN THE MAP FILE, LINE ' + str(count) + ' IS NOT OK! (LENGTH OF THE LINE IS NOT EQUAL ' + 
                          'TO THE WIDTH FROM LINE 3. LENGTH OF THE LINE SHOULD BE ' + str(width) + ' CHARACTERS)')
                    return False

                if not bool(re.match('^['+valid_char+']+$', line)):
                    print('ERROR: WRONG VALUE IN THE MAP FILE, LINE ' + str(count) + ' IS NOT OK! (INVALID CHARACTER, ONLY VALID CHARACTERS: ' 
                          + valid_char+ ' )')
                    return False

            count = count + 1

        #CHECK OF THE NUMBER OF LINES IS EQUAL TO PARAMETER "length" FROM LINE 2
        if len(lines)-4 != height:
            print('ERROR: WRONG VALUE IN THE MAP FILE, LINE 2 IS NOT OK! (NUMBER OF LINES IN THE FILE SHOULD BE EQUAL TO THE ' 
                  + 'NUMBER GIVEN BY THE ATTRIBUTE \"height\" IN THE LINE 2). NUMBER OF LINES SHOULD BE: ' + str(height))
            return False

        self._size_x = width
        self._size_y = height
        
        return True

    def analyze_file_scene(self, lines):
        '''
        The function checks the content of the .scen file. For more details about the correct syntax of the .scen file look here:
        https://movingai.com/benchmarks/formats.html

        :param lines: list of strings - where every string represents a line from the .scen file
        :return: **True**, if the content of the .scen file is okay  
        **False**, otherwise
        '''

        print('STATUS: CHECKING THE SCENE FILE SYNTAX')
        self._buckets = []
        self._start = []
        self._goals = []
        self._heuristics = []

        map_filename  = self._model.mapf_get_filename_map()
        map_width     = self._model.mapf_get_map_width()
        map_height    = self._model.mapf_get_map_height()

        line1 = "version"

        #line counter
        count = 1

        #FOR EVERY LINE IN THE FILE
        for line in lines:
            #REMOVE WHITESPACES FROM THE END OF THE LINE
            line = line.rstrip()

            #CHECK LINE 1: SHOULD BE EQUAL TO 'version x.x'
            if count == 1:
                words = line.split()
                if len(words) != 2:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE 1 IS NOT OK! (SHOULD BE EQUAL TO \"version x.x\")')
                    return False

                if words[0] !=  line1:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE 1 IS NOT OK! (SHOULD BE EQUAL TO \"version x.x\")')
                    return False

            #CHECK EVERY OTHER LINE: SHOULD BE MADE OF 9 FIELDS
            #   bucket number / map file name / map width / map height / start x / start y / goal x / goal y / heuristics
            if count > 1:
                #split line by whitespaces in nine fields
                fields = line.split()
                
                #CHECK IF THE LINE IS MADE OF EXACTLY 9 FIELDS
                if len(fields) != 9:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (NUMBER OF FIELDS SHOULD BE EQUAL TO 9)')
                    return False

                #CHECK FIELD 1: bucket should be an integer
                try:
                    bucket= int(fields[0])
                except ValueError:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (FIRST ELEMENT IN THE LINE REPRESENTS THE BUCKET NUMBER'
                          + ' AND SHOULD BE AN INTEGER NUMBER >= 0)')
                    return False

                if bucket<0:
                    print('ERROR: WRONG VALUE IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (FIRST ELEMENT IN THE LINE REPRESENTS THE BUCKET NUMBER'
                          + ' AND SHOULD BE AN INTEGER NUMBER >= 0)')
                    return False

                #CHECK FIELD 2: map file name
                if fields[1] != map_filename:
                    print('ERROR: WRONG VALUE IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (SECOND ELEMENT IN THE LINE REPRESENTS THE NAME OF THE MAP FILE'
                          + ' AND SHOULD BE EQUAL TO THE NAME OF THE LOADED MAP FILE)')
                    print("INPUT VALUE FROM LINE " + str(count)+ ": ", fields[1])
                    print("LOADED MAP FILE: " + self._model.mapf_get_filename_map())
                    return False
                
                #CHECK FIELD 3: map width
                if fields[2] != str(map_width):
                    print('ERROR: WRONG VALUE IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (THIRD ELEMENT IN THE LINE REPRESENTS THE MAP WIDTH'
                          + ' AND SHOULD BE EQUAL TO THE WIDTH OF THE LOADED MAP FILE)')
                    print("INPUT VALUE FROM LINE " + str(count)+ ": ", fields[2])
                    print("WIDTH OF THE LOADED MAP FILE: ", map_width)
                    return False

                #CHECK FIELD 4: map height
                if fields[3] != str(map_height):
                    print('ERROR: WRONG VALUE IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 4 IN THE LINE REPRESENTS THE MAP HEIGHT'
                          + ' AND SHOULD BE EQUAL TO THE HEIGHT OF THE LOADED MAP FILE)')
                    print("INPUT VALUE FROM LINE " + str(count)+ ": ", fields[3])
                    print("WIDTH OF THE LOADED MAP FILE: ", map_height)
                    return False

                #CHECK FIELD 5: start x
                try:
                    start_x= int(fields[4])

                except ValueError:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 5 IN THE LINE REPRESENTS THE START X COORDINATE OF THE AGENT'
                          + ' AND SHOULD BE AN INTEGER NUMBER IN THE RANGE OF (0 : MAP WIDTH-1))')
                    return False

                if (start_x<0 or start_x >= map_width):
                    print('ERROR: WRONG VALUE IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 5 IN THE LINE REPRESENTS THE START X COORDINATE OF THE AGENT'
                          + ' AND SHOULD BE AN INTEGER NUMBER IN THE RANGE OF (0 : MAP WIDTH-1))')
                    print("INPUT START X:", start_x)
                    print("MAP WIDTH", map_width)
                    return False

                #CHECK FIELD 6: start y
                try:
                    start_y= int(fields[5])

                except ValueError:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 6 IN THE LINE REPRESENTS THE START Y COORDINATE OF THE AGENT'
                          + ' AND SHOULD BE AN INTEGER NUMBER IN THE RANGE OF (0 : MAP HEIGHT-1))')
                    return False

                if (start_y<0 or start_y >= map_height):
                    print('ERROR: WRONG VALUE IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 5 IN THE LINE REPRESENTS THE START Y COORDINATE OF THE AGENT'
                          + ' AND SHOULD BE AN INTEGER NUMBER IN THE RANGE OF (0 : MAP HEIGHT-1))')
                    print("INPUT START Y:", start_y)
                    print("MAP HEIGHT", map_height)
                    return False

                #CHECK FIELD 7: goal x
                try:
                    goal_x= int(fields[6])

                except ValueError:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 7 IN THE LINE REPRESENTS THE TARGET X COORDINATE OF THE AGENT'
                          + ' AND SHOULD BE AN INTEGER NUMBER IN THE RANGE OF (0 : MAP WIDTH-1))')
                    return False

                if (goal_x<0 or goal_x >= map_width):
                    print('ERROR: WRONG VALUE IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 7 IN THE LINE REPRESENTS THE TARGET X COORDINATE OF THE AGENT'
                          + ' AND SHOULD BE AN INTEGER NUMBER IN THE RANGE OF (0 : MAP WIDTH-1))')
                    print("INPUT GOAL X:", goal_x)
                    print("MAP WIDTH", map_width)
                    return False

                #CHECK FIELD 8: goal y
                try:
                    goal_y= int(fields[7])

                except ValueError:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 8 IN THE LINE REPRESENTS THE TARGET Y COORDINATE OF THE AGENT'
                          + ' AND SHOULD BE AN INTEGER NUMBER IN THE RANGE OF (0 : MAP HEIGHT-1))')
                    return False

                if (goal_y<0 or goal_y >= map_height):
                    print('ERROR: WRONG VALUE IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 8 IN THE LINE REPRESENTS THE TARGET Y COORDINATE OF THE AGENT'
                          + ' AND SHOULD BE AN INTEGER NUMBER IN THE RANGE OF (0 : MAP HEIGHT-1))')
                    print("INPUT GOAL Y:", goal_y)
                    print("MAP HEIGHT", map_height)
                    return -False

                #CHECK FIELD 9: heuristics [=optimal length]
                try:
                    heuristic= float(fields[8])

                except ValueError:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 9 IN THE LINE REPRESENTS THE OPTIMAL HEURISTIC AND SHOULD BE A NUMBER >=0)')
                    return False

                if heuristic<0:
                    print('ERROR: WRONG SYNTAX IN THE SCENE FILE, LINE ' + str(count) 
                          + ' IS NOT OK! (ELEMENT 9 IN THE LINE REPRESENTS THE OPTIMAL HEURISTIC AND SHOULD BE A NUMBER >=0)')
                    return -False

                #SAVE LINE
                self._buckets.append(bucket)
                self._start.append((start_x+1, start_y+1))      # +1 because in the scene file counting starts by 0
                self._goals.append((goal_x+1, goal_y+1))        # but in the model counting starts by 1
                self._heuristics.append(heuristic)

            count = count + 1
            
        return True
    
    def check_collisions(self, robot_nodes, goal_nodes):
        '''
        This function checks if there are conflicts between the different 
        items in the map. 
        
        A conflict occurs in the following cases:
        -   two or more agents are trying to occupy the same spot
        -   two or more robots have the same target
        -   the start position of an agent collides with a blocked node in the map
        -   the target spot of an agent collides with a blocked node in the map

        :param robot_nodes: list of (x,y) - where every entry (x,y) represents the start position for one robot
        :param goal_nodes: list of (x,y) - where every entry (x,y) represents the end position for one robot
        :return: **True**, if no conflict was found  
        **False**, if a conflict was found
        '''

        #Note: in the model position counting stars by (1,1) 
        #      and in the scene file by (0,0)
        blocked_nodes = self._model.get_blocked_nodes()
        
        #1. TEST IF TWO AGENTS TRY TO OCCUPY THE SAME SPOT
        newlist = []
        line = 2
        for node in robot_nodes:
            
            if node not in newlist:
                newlist.append(node)        
            else:
                #Attention: node[0]-1, because in the model position counting stars by (1,1) 
                #                      and in the scene file by (0,0)
                print('Collision occured in the map between two agents, start positions are equal, line: ', line,
                       "\nConflict at position: ", str(node[0]-1) + " " + str(node[1]-1),
                       "\nTwo agents cannot have the same start position!")
                return True    
            
            line=line+1

        #2. TEST IF TWO GOAL NODES OCCUPY THE SAME SPOT
        newlist = []
        line = 2
        for node in goal_nodes:
            
            if node not in newlist:
                newlist.append(node)        
            else:
                print('Collision occured in the map, two goal positions are equal, line: ', line,
                       "\nConflict at goal position: ", str(node[0]-1) + " " + str(node[1]-1),
                       "\nTwo agents cannot have same goal position!")
                return True    
            
            line=line+1
          
        #3. TEST IF A ROBOT START POSITION COLLIDES WITH A BLOCKED NODE IN THE MAP
        line = 2
        for node in robot_nodes:
            if node in blocked_nodes:
                print('Collision occured in the map, start position of an agent is trying to occupy a blocked spot, '+
                      "line: " + str(line) + " in the .scen file"
                      "\nConflict at position: x: " + str(node[0]-1) + ", y:  " + str(node[1]-1) + 
                      ", position is already occupied!")
                return True    
            line=line+1
        
        #4. TEST IF A ROBOT GOAL POSITION COLLIDES WITH A BLOCKED NODE IN THE MAP
        line = 2
        for node in goal_nodes:
            if node in blocked_nodes:
                print('Collision occured in the map, goal position of an agent is a blocked spot, '+
                      "line: " + str(line) + " in the .scen file"
                      "\nConflict at position: x: " + str(node[0]-1) + ", y:  " + str(node[1]-1) + 
                      ", position is already occupied!")
                return True    
            line=line+1
          
        return False


'''
class AspParser(object):
    def __init__(self):
        self._model = Model()
        self._control = Control()
        self._model_view = None
        self._solver = None
        self._programs = {}
        self._str_model = ''
        self._parser_widget = None
        self.reset_programs()

    def set_model_view(self, model_view):
        self._model_view = model_view

    def set_solver(self, solver):
        self._solver = solver

    def set_program(self, program_name, program):
        self._programs[program_name] = program

    def add_program(self, program_name, program):
        if program_name in self._programs:
            self._programs[program_name] += program
        else:
            self._programs[program_name] = program
        if self._parser_widget is not None:
            self._parser_widget.update()

    def delete_program(self, program_name):
        if program_name in self._programs:
            del self._programs[program_name]
        if self._parser_widget is not None:
            self._parser_widget.update()

    def set_parser_widget(self, parser_widget):
        if self._parser_widget is parser_widget:
            return
        temp = self._parser_widget
        self._parser_widget = parser_widget
        if temp is not None:
            temp.set_parser(None)
        if parser_widget is not None:
            parser_widget.set_parser(self)
            parser_widget.update()

    def on_model(self, m):
        for x in m.symbols(atoms=True):
            self.on_atom(x)
            self._str_model += str(x) + '\n'
        self.done_instance()

    def on_atom(self, atom):
        if atom is None:
            return
        if len(atom.arguments) < 2:
            return
        obj = atom.arguments[0]
        value = atom.arguments[1]

        if atom.name == 'occurs' and len(atom.arguments) == 3:
            self._on_occurs_atom(obj, value, atom.arguments[2].number)
        elif atom.name == 'init' and len(atom.arguments) == 2:
            self._on_init_atom(obj, value)

    def _on_occurs_atom(self, obj, action, time_step):
        try:
            if obj.name == 'object' and action.name == 'action':

                kind = str(obj.arguments[0])
                ID = str(obj.arguments[1])

                action_name = str(action.arguments[0])
                action_value = action.arguments[1]

                item = self._model.get_item(kind, ID, True, True)
                if item is not None:
                    item.set_action(action, time_step)
                if time_step > self._model.get_num_steps(): 
                    self._model.set_num_steps(time_step)
                self._model.set_editable(False)
        except:
            print('invalid occurs format, expecting: occurs(object([object], [objectID]), action([action], [arguments]), [time step])')

    def _on_init_atom(self, obj, value):
        try:
            if (obj.name == 'object' and value.name == 'value'
                    and len(obj.arguments) == 2
                    and len(value.arguments) == 2):

                kind = str(obj.arguments[0])
                ID = str(obj.arguments[1])

                value_name = str(value.arguments[0])
                value_value = value.arguments[1]
                item = self._model.get_item(kind, ID, True, self._model.get_editable())
                if item is not None:
                    result = item.parse_init_value(value_name,
                                                    value_value)
                    if result == 0: 
                        return

                if kind == 'node' and value_name == 'at':                  #init nodes
                    self._model.add_node(value_value.arguments[0].number, 
                                         value_value.arguments[1].number, ID)
                    return

                elif kind == 'highway' and value_name == 'at':             #init highways
                    self._model.add_highway(value_value.arguments[0].number, 
                                            value_value.arguments[1].number)
                    return

                elif kind == 'product' and value_name == 'on':              #init products
                    if value_value.arguments is None:
                        shelf = self._model.get_item('shelf', value_value, True, True)
                        shelf.set_product_amount(ID, 0)
                        return
                    else:
                        shelf = self._model.get_item('shelf', value_value.arguments[0], True, True)
                        shelf.set_product_amount(ID, value_value.arguments[1].number)
                        return

                self._model.add_init('init(' + str(obj) + ', ' + str(value) + ')')

        except Exception as e:
            if mapf_simulation.ll_config.get('features', 'debug'):
                traceback.print_exc()
            print(('invalid init: init(' + str(obj) + ', ' + str(value) + ')'))

    def done_instance(self, enable_auto_solve = True):
        self._model.accept_new_items()
        self._model.update_windows()
        if (self._solver is not None
            and mapf_simulation.configuration.config.get('visualizer', 'auto_solve') 
            and enable_auto_solve):
            self._solver.set_model(self._model)
            self._solver.solve()

        if (self._model_view is not None):
            self._model_view.update()
            self._model_view.resize_to_fit()

        if self._parser_widget is not None:
            self._parser_widget.update()

    def clear_model(self):
        self._model.clear()

    def clear_model_actions(self, restart = True):
        self._model.clear_actions()
        if restart:
            self._model.restart()

    def reset_programs(self):
        self._programs = {}
        str_load_files = mapf_simulation.configuration.ll_config.get('features', 'load_files')
        try:
            str_load_files = str_load_files.replace(' ', '')
            files = str_load_files.split(',')
            for file_name in files:
                if file_name != '':
                    if os.path.isfile(file_name):
                        ff = open(file_name)
                        self._programs[file_name] = ff.read()
                        ff.close()
        except RuntimeError as error:
            print(error)
            print('file parsing failed')
            return -1
        if self._parser_widget is not None:
            self._parser_widget.update()

    def reset_grounder(self):
        self._str_model = ''
        self._control = Control()
        if self._parser_widget is not None:
            self._parser_widget.update()

    def load(self, file_name):
        if not os.path.isfile(file_name):
            print('can not open file: ', file_name)
            return -1

        print('load file: ' + file_name)
        try:
            ff = open(file_name)
            self._programs[file_name] = ff.read()
            ff.close()
            if self._parser_widget is not None:
                self._parser_widget.update()
        except RuntimeError as error:
            print(error)
            print('file loading failed')
            return -2
        return 0

    def parse(self):
        if self._control is None:        
            return
        try:
            with ProgramBuilder(self._control) as bb:
                for key in self._programs:
                    parse_string(self._programs[key], lambda stm: bb.add(stm))
            self._control.ground([('base', [])])
            result = self._control.solve(on_model=self.on_model)
            print(result)
        except RuntimeError as error:
            print(error)
            return -2
        return 0

    def parse_file(self, file_name, clear = False, clear_actions = False, clear_grounder = True):
        if not os.path.isfile(file_name):
            print('can not open file: ', file_name)
            return -1

        if clear:
            self.reset_programs()
            self.clear_model()

        if clear_actions:
            self.reset_programs()
            self.clear_model_actions()

        if (clear or clear_actions) and self._model_view is not None:
            self._model_view.stop_timer()

        if clear_grounder:
            self.reset_grounder()
        if self.load(file_name) < 0:
            return -1
        if self.parse() < 0:
            return -2
        return 0

    def load_instance(self, file_name, create_png = False):
        result = self.parse_file(file_name, clear = True)
        if result < 0:
            return result

        if (self._model_view is not None 
            and (mapf_simulation.configuration.config.get('visualizer', 'create_pngs') or create_png)):

            rect = self._model_view.sceneRect()
            position  = self._model_view.mapFromScene(QPoint(rect.x(), rect.y()))
            position2 = self._model_view.mapFromScene(QPoint(rect.x() + rect.width(), 
                                                            rect.y() + rect.height()))
            pixmap = self._model_view.grab(QRect(position.x(), position.y(), 
                                                position2.x() - position.x(), 
                                                position2.y() - position.y()))
            pixmap.save(file_name[0 : file_name.rfind('.')] + '.png')
        self._model.update_windows()
        return 0

    def list_programs(self):
        for key in self._programs:
            yield key

    def get_model(self):
        return self._model

    def get_program(self, program_name):
        if program_name in self._programs:
            return self._programs[program_name]
        return None

    def get_program_count(self):
        return len(self._programs)

    def get_str_model(self):
        return self._str_model

'''