#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "PBS.h"
#include "Instance.h"


#include "rclcpp/rclcpp.hpp"
#include "mapf_interfaces/msg/mapf_instance.hpp"
#include "mapf_interfaces/msg/mapf_solution.hpp"


//using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std::chrono_literals;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PathPlanner : public rclcpp::Node
{
  public:
    PathPlanner()
    : Node("mapf_planner_pbs_node")
    {
      init_ros();
    }

    /**
     * This function runs the Path Planner Node
    */
    void run()
    {      
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(shared_from_this());
      
      //timeout = 1 seconds
      std::chrono::seconds seconds{1};
      std::chrono::nanoseconds timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(seconds);

      while(rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "Waiting for incoming message, topic: /mapf_instance_info[MAPFInstance.msg]");

        if (message_received)
        {
          solve();
          message_received = false;
        }
        executor.spin_once(timeout);
      }

      RCLCPP_INFO(this->get_logger(),"rclcpp::ok() not OK");

    }
    
    /**
     * This function solves the path planning problem using the "Priority Based Search" algorithm
     * and sends the solution to the topic /mapf_instance_solution
     * as message of type "mapf_interfaces/MAPFSolution"
    */
    void solve()
    {
      //Save ID 
      id = instance_msg->id;

      /**
       *  Create an instance from received message of type "mapf_interfaces/MAPFInstance"
      */
      RCLCPP_INFO(this->get_logger(), "Instance message received, STEP = '%d'", instance_msg->step);
      RCLCPP_INFO(this->get_logger(), "Create a custom instance for the algorithm from the MAPFInstance.msg");
      Instance instance(instance_msg);


      /**
       *  Solve the instance 
      */
      RCLCPP_INFO(this->get_logger(), "Start path finding algorithm");
      PBS pbs(instance, false, 1);
      srand(0);
      pbs.solve(60.0); //runtime limit = 60seconds
      

      /**
       * Create and publish "mapf_interfaces/MAPFSolution" if solution found
      */
      if (pbs.solution_found)
      {

        /**
         * Create a message of type "mapf_interfaces/MAPFSolution"
        */
        auto message = mapf_interfaces::msg::MAPFSolution();
        
        /**
         *  Set:
         *  id, step and algorithm name 
        */
        message.id = id;
        message.step = instance_msg->step;
        message.name_algorithm = "priority based search with A*";

        
        /**
         *  Set:
         *  mapf_interfaces/PathInfo for every robot in paths[]
        */
        int number_robots = instance_msg->map.number_robots;
        for (int i = 0; i < number_robots; i++)
        {
          auto temp = mapf_interfaces::msg::PathInfo();
          temp.step = instance_msg->step;
          temp.robot_id = i;
          temp.path = pbs.getPath(i);
          message.paths.push_back(temp);
        }


        /**
         *  Set:
         *  mapf_interfaces/StatisticInfo.msg
        */
        message.statistics.name_algorithm = message.name_algorithm;
        message.statistics.step = message.step;
        message.statistics.runtime = pbs.runtime;
        message.statistics.sum_of_cost = pbs.solution_cost;
        message.statistics.highlevel_expanded_nodes = pbs.num_HL_expanded;
        message.statistics.highlevel_generated_nodes = pbs.num_HL_generated;
        message.statistics.lowlevel_expanded_nodes = pbs.num_LL_expanded;
        message.statistics.lowlevel_generated_nodes = pbs.num_LL_generated;
        
        /**
         * Publish the solution
        */
        RCLCPP_INFO(this->get_logger(), "publish MAPFSolution.msg, ALGORITHM: '%s', STEP: '%d'", message.name_algorithm, message.step);
        publisher_solution_->publish(message);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "No solution found");
      }

      /**
       * Clear data
      */
      pbs.clearSearchEngines();    
    }

  private:

    rclcpp::Subscription<mapf_interfaces::msg::MAPFInstance>::SharedPtr subscription_instance_;
    rclcpp::Publisher<mapf_interfaces::msg::MAPFSolution>::SharedPtr publisher_solution_;
    
    int id;
    bool message_received = false;
    mapf_interfaces::msg::MAPFInstance::SharedPtr instance_msg;
    
    /**
     * This function initialises subscribers and publishers
    */
    void init_ros()
    {
      //create subscriber to /mapf_instance_info
      subscription_instance_ = this->create_subscription<mapf_interfaces::msg::MAPFInstance>(
                        "mapf_instance_info", 1, std::bind(&PathPlanner::callback_instance, this, _1)
      );
      
      //create publisher to /mapf_instance_solution
      publisher_solution_ = this->create_publisher<mapf_interfaces::msg::MAPFSolution>(
                        "mapf_instance_solution", 1);
    }

    /**
     * This function will be called every time a message on the topic /mapf_instance_info
     * is received
    */
    void callback_instance(mapf_interfaces::msg::MAPFInstance::SharedPtr msg)
    {
      instance_msg = msg;
      message_received = true;
      RCLCPP_INFO(this->get_logger(), "I heard: STEP = '%d'", instance_msg->step);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<PathPlanner> planner = std::make_shared<PathPlanner>();
  planner->run();
  rclcpp::shutdown();
  
  return 0;
}




