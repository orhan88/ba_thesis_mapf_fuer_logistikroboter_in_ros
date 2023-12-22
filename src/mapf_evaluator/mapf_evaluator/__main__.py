import sys

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mapf_interfaces.msg import StatisticInfo

class Algorithm():
    '''
    A class to represent an Algorithm
    '''

    def __init__(self, name, color, markerfacecolor):    
        """

        :param name: string - Name of the algorithm

        :param color: string - Color in which the algorithm will be plotted. 
        >This parameter will be used by the function "pyplot.plot(color= ...)" 
        to plot this algorithm by the specified color.  
        >Examples: "r" for red, "g" for green"...  
        >More information: https://matplotlib.org/stable/gallery/color/named_colors.html
        
        :param markerfacecolor: string - Color for the markerpoints  
        >This parameter will be used by the function "pyplot.plot(markerfacecolor= ...)" 
        to plot the markerpoints for this algorithm by the specified color.  
        >Examples: "r" for red, "b" for blue"...  
        >More information: https://matplotlib.org/stable/gallery/color/named_colors.html
        """
        
        self.name = name
        '''**string** - The name of the algorithm'''
        self.color = color
        '''**string** - Color in which the algorithm will be plotted'''
        self.markerfacecolor = markerfacecolor
        '''**string** - Color in which the markerpoints for this algorithm will be plotted'''
        self.msg = []
        '''**list** - List for saving the incoming ros2 messages of type "mapf_interfaces/StatisticInfo.msg".  
        Every entry stores the statistical information about the algorithm at a certain step.  
        By plotting the data for the algorithm the information from every message will be read.'''
        #self.alg_number = 0

    def add_msg(self, msg):
        """
        Saves the message in a list

        :param msg: mapf_interfaces/StatisticInfo.msg  
        """
        self.msg.append(msg)


class Evaluator(Node):
    '''
    This class represents the following ros2 node: **mapf_evaluator_node**.  

    The node mapf_evaluator_node subscribes to the folllowing topic: **/mapf_statistics_info**  
    Every time a message on the topic is received, the following function will be called: **callback(self, msg)**

    The evaluator is executed by the function **run()**. This function creates a matplotlib.pyplot figure and 
    then runs a while loop as long as the node mapf_evaluator_node is runnning.  
    Every loop cycle the ros2 node is spinned once and the figure is updated with new data
    '''

    def __init__(self):        
        super().__init__('mapf_evaluator_node')
        self.subscriber = self.create_subscription(StatisticInfo, 'mapf_statistics_info', self.callback, 10)
        '''**rclpy.subscription.Subscription** - ros2 subscriber to the topic */mapf_statistics_info*'''
        self.algorithms = []  
        '''**list** - list of saved algorithms. Items are objects of the class "Algorithm" '''
        #list of saved steps
        #self.step = []  
        #'''list - list of saved steps. Items are type of the class "Algorithm" '''
        #number defines how many algorithms stored, max 4.
        self.number = 0
        '''**int** - describes the number of saved algorithms'''

        #color combinations: (line / marker)
        #first:  -> green / blue
        #second: -> red   / yello
        #third:  -> black / white
        #last:   -> cyan  / magenta
        self.colors = ["g", "r", "k", "c"]
        '''
        **list** - list of colors for plotting the data.

        Per default the first algorithm will be plotted in green, second in red, third in black and the last in cyan'''
        self.markerfacecolors = ["b", "y", "w", "m"]
        '''
        **list** - list of colors for plotting the markerpoints
        
        Per default the markerpoints for the first algorithm will be plotted in blue, for the second in yellow, third in white and the last in magenta
        '''
        
    def callback(self, msg):
        '''
        This function will be called every time a message on the topic /mapf_statistics_info is received.  
        It saves the received message by finding the right algorithm from self.algorithms and then saving it in the right spot.  
        
        If no message for the algorithm has been saved so far, then it creates an instance of the class "Algorithm".
        This instance will be saved in self.algorithms, so next messages can be added later here.

        For reasons of clarity the max number of 4 different algorithms can be saved and plotted by the evaluator.
        If the number exceeds 4, then the first added algorithm will be deleted from self.algorithms
        
        :param msg: mapf_interfaces/StatisticInfo.msg - message from the topic /mapf_statistics_info
        '''
        self.get_logger().info('New callback received. Algorithm name: "%s", STEP: %s, SoC: %f, Runtime: %f ' % (msg.name_algorithm, msg.step, msg.sum_of_cost, msg.runtime))
        #add new algorithm to the list if no message received for this algorithm
        if self.check_new_algorithm(msg.name_algorithm):
            self.algorithms.append(Algorithm(msg.name_algorithm, self.colors[self.number%4], self.markerfacecolors[self.number%4]))
            self.number += 1
            #dont store more than four algorithms
            if self.number > 4:
                #remove first element from the list
                self.algorithms.pop(0)
        self.save_msg(msg)
        #self.print_all_algorithms()

    def save_msg(self, msg):
        '''
        This function saves the received msg in self.algorithm. It iterates the list of saved algorithms till it finds the algorithm with the right name. 
        After it saves the message for the right algorithm

        :param msg: mapf_interfaces/StatisticInfo.msg - message from the topic /mapf_statistics_info
        '''
        #save the message for the correct algorithm 
        for algorithm in self.algorithms:
            if msg.name_algorithm == algorithm.name:
                algorithm.msg.append(msg)
                return

    def check_new_algorithm(self, name):
        '''
        This function checks whether there is already an instance of the class Algorithm saved in self._algorithms with the given name
        
        :param name: string - name of the algorithm
        
        :return: True, if an instance exists with the given name  
        False, if there is no instance with the given name
        '''
        for algorithm in self.algorithms:
            if algorithm.name == name:
                return False
        return True

    def print_all_algorithms(self):
        '''
        Help function for printing all the saved data
        '''
        print(len(self.algorithms))
        for algorithm in self.algorithms:
            print(algorithm.name)
            print(algorithm.color)
            print(algorithm.markerfacecolor)
            print(len(algorithm.msg))

    def run(self):
        '''
        Executes the evaluator

        Creates a matplotlib.pyplot figure and then runs a while loop as long as the ros2 node mapf_evaluator_node is runnning.  
        Every loop cycle the ros2 node is spinned once and the figure is updated with new data.
        '''
        plt.ion()
        plt.show()
        plt.figure(num='Algorithm evaluation')        

        while(rclpy.ok()):
        
            plt.clf()
            #plot "sum of cost" over "step" for every algorithm
            for algorithm in self.algorithms:
                step = []
                sof = []
                runtime =[]
                #for every message saved for this algorithm
                for msg in algorithm.msg:
                    #get step, sum_of_cost and runtime from msg
                    step.append(msg.step)
                    sof.append(msg.sum_of_cost)
                    runtime.append(msg.runtime)

                #print(step)    
                #print(sof)    
                #plotting sum of cost over step
                plot1 = plt.subplot(2, 1, 1)
                plot1.plot(step, sof, label = algorithm.name, color=algorithm.color, linestyle='dashed', linewidth = 3,
                        marker='o', markerfacecolor=algorithm.markerfacecolor, markersize=12)
                plot1.legend()
                # naming the x axis    
                plt.xlabel('STEP')
                # naming the y axis
                plt.ylabel('SUM OF COST')
                #plt.title('Sum of cost')
                plot1.set_xlim(left=0)
                plot1.set_ylim(bottom=0)
                plot1.autoscale(True)
                
                plot2 = plt.subplot(2, 1, 2)
                plot2.plot(step, runtime, label = algorithm.name, color=algorithm.color, linestyle='dashed', linewidth = 3,
                        marker='o', markerfacecolor=algorithm.markerfacecolor, markersize=12)
                plot2.legend()
                plot2.set_xlim(left=0)
                plot2.set_ylim(bottom=0)
                plot2.autoscale(True)
                # naming the x axis    
                plt.xlabel('STEP')
                # naming the y axis
                plt.ylabel('RUNTIME')
            
            #update figure
            plt.draw()
            plt.pause(0.1)
            
            #update ROS            
            rclpy.spin_once(self, timeout_sec=0.0)
            
def main(args = None):
    '''
        Main function: creates the Evaluator and executes it.
    '''
    rclpy.init(args=args)

    evaluator = Evaluator()
    evaluator.run()

    evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
