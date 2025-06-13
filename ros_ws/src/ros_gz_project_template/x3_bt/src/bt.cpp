#include <behaviortree_cpp/bt_factory.h>
#include "nav2_msgs/srv/load_map.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "x3_interfaces/action/set_height.hpp"
#include <behaviortree_ros2/bt_action_node.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <behaviortree_ros2/bt_service_node.hpp>

using namespace BT;

class CheckAltitude : public SyncActionNode
{
    public:
        CheckAltitude(const std::string& name, const NodeConfig& config):
        SyncActionNode(name,config)
        {}
    
        static PortsList providedPorts()
        {
            return {
                InputPort<float>("cur_alt"),
                InputPort<float>("z")
            };
        }
        NodeStatus tick() override
        {
            Expected<float> cur_alt = getInput<float>("cur_alt");
            Expected<float> z = getInput<float>("z");
            if (cur_alt.value() == z.value())
            {
                return NodeStatus::SUCCESS;
            }
            else
            {
                return NodeStatus::FAILURE;
            }
        }
};

using namespace BT;

class CheckBattery : public SyncActionNode
{
    public:
        CheckBattery(const std::string& name, const NodeConfig& config):
        SyncActionNode(name,config)
        {
            battery_level = 100;
        }
    
        static PortsList providedPorts()
        {
            return {
                OutputPort<int>("battery_level")
            };
        }
        NodeStatus tick() override
        {
            //decrease battery percentage from last movement
            printf("Battery Check\n");
            battery_level -= 10;
            setOutput<int>("battery_level", battery_level);

            if (battery_level > 10)
            {
                return NodeStatus::SUCCESS;
            }
            else
            {
                return NodeStatus::FAILURE;
            }
        }
    private:
        int battery_level;

};

using namespace BT;

class GetNextWayPoint : public SyncActionNode
{
    public:
        GetNextWayPoint(const std::string& name, const NodeConfig& config):
        SyncActionNode(name,config)
        {
            wayPtNum = 0;
        }
    
        static PortsList providedPorts()
        {
            return 
            {
                OutputPort<float>("x"),
                OutputPort<float>("y"),
                OutputPort<float>("z")
            };
        }
        NodeStatus tick() override
        {
            if (wayPtNum < 4)
            {
                setOutput<float>("x", XwayPts[wayPtNum]);
                setOutput<float>("y", YwayPts[wayPtNum]);
                setOutput<float>("z", ZwayPts[wayPtNum]);
                wayPtNum++;
                return NodeStatus::SUCCESS;
            }
            else
            {
                return NodeStatus::FAILURE;
            }
        }
    private:
        int wayPtNum;
        float XwayPts[5] = {1, 1, 2, 4, 5};
        float YwayPts[5] = {0, 2, 4, 3, -2};
        float ZwayPts[5] = {1.5, 1.5, 1.5, 1.5, 1.5};
};

using namespace BT;

class GetTakeoffLocation : public SyncActionNode
{
    public:
        GetTakeoffLocation(const std::string& name, const NodeConfig& config):
        SyncActionNode(name,config)
        {}
    
        static PortsList providedPorts()
        {
            return {
                OutputPort<float>("startX"),
                OutputPort<float>("startY")
            };
        }
        NodeStatus tick() override
        {
            setOutput<float>("startX", 0);
            setOutput<float>("startY", 0);
            return NodeStatus::SUCCESS;
        }
};

using Nav2LoadMap = nav2_msgs::srv::LoadMap;
using namespace BT;


class LoadMap: public RosServiceNode<Nav2LoadMap>
{
  public:
  const std::string map_path = "/home/benjamin/Documents/seniorProject/maps/"; //hard code map path

  LoadMap(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<Nav2LoadMap>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        InputPort<float>("cur_alt")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // use input ports to set map file
    float map_height;
    getInput("cur_alt", map_height);
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << map_height;
    std::string map_name = stream.str();    
    std::replace(map_name.begin(), map_name.end(), '.', '_');
    map_name = map_path + map_name + "Map.yaml"; //get the actual path
    request->map_url = map_name;
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    if (!(response->result))
    {
        printf("Success response\n");
        return NodeStatus::SUCCESS;
    }
    else
    {
        printf("Failure response\n");
        return NodeStatus::FAILURE;
    }
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    printf("Failure\n");
    //RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }
};

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleSetNavGoal = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using namespace BT;

class SetNavGoal: public RosActionNode<NavigateToPose>
{
public:
  SetNavGoal(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<NavigateToPose>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        InputPort<float>("x"),
        InputPort<float>("y")
    });
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    printf("Set Goal\n");

    // get "x and y" from the Input port
    getInput("x", goal.pose.pose.position.x);
    getInput("y", goal.pose.pose.position.y);
    //set map
    goal.pose.header.frame_id = "map";
    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    uint16_t result = wr.result->error_code; //get result code
    if (result == nav2_msgs::action::NavigateToPose::Result::NONE)
    {
        printf("Nav Goal Success\n");
        return NodeStatus::SUCCESS;
    }
    else
    {
        return NodeStatus::FAILURE;
    }
  }
  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    //printf("Nav Goal Feedback\n");
    geometry_msgs::msg::PoseStamped current_pose = feedback->current_pose;
    return NodeStatus::RUNNING;
  }
};

using SetHeight = x3_interfaces::action::SetHeight;
using GoalHandleSetAltitude = rclcpp_action::ServerGoalHandle<SetHeight>;
using namespace BT;

class SetAltitude: public RosActionNode<SetHeight>
{
public:
  SetAltitude(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<SetHeight>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        InputPort<float>("z"),
        OutputPort<float>("cur_alt")
    });
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    // get "order" from the Input port
    getInput("z", goal.target_height);
    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    float altitude;
    altitude = wr.result->final_height;
    if(altitude >= 0)
    {
        setOutput<float>("cur_alt", altitude);
        return NodeStatus::SUCCESS;
    }
    else
    {
        return NodeStatus::FAILURE;
    }
  }
  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    //RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    float current_height;
    current_height = feedback->current_height;
    //RCLCPP_INFO(node_->get_logger(), "Current altitude: %.2f", current_height);
    return NodeStatus::RUNNING;
  }
};

using namespace BT;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;
    printf("Starting BT\n");
    factory.registerNodeType<CheckAltitude>("CheckAltitude");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<GetTakeoffLocation>("GetTakeoffLocation");
    factory.registerNodeType<GetNextWayPoint>("GetNextWayPoint");
    printf("Nodes Registered BT\n");
    //Load Map Node & Params
    auto ldMapNode = std::make_shared<rclcpp::Node>("load_map_client");
    RosNodeParams ldMapParams; 
    ldMapParams.nh = ldMapNode;
    ldMapParams.default_port_value = "/map_server/load_map";
    factory.registerNodeType<LoadMap>("LoadMap", ldMapParams);
    printf("load map Registered BT\n");

    //Set Nav Goal Node & Params
    auto navGoalNode = std::make_shared<rclcpp::Node>("set_nav_goal_client");
    RosNodeParams navGoalParams; 
    navGoalParams.nh = navGoalNode;
    navGoalParams.default_port_value = "/navigate_to_pose";
    factory.registerNodeType<SetNavGoal>("SetNavGoal", navGoalParams);
    printf("nav goal Registered BT\n");


    //Set Altitude Node & Params
    auto setAltNode = std::make_shared<rclcpp::Node>("set_altitude_client");
    RosNodeParams setAltParams; 
    setAltParams.nh = setAltNode;
    setAltParams.default_port_value = "/set_height";
    factory.registerNodeType<SetAltitude>("SetAltitude", setAltParams);
    printf("set altitude Registered BT\n");



    auto tree = factory.createTreeFromFile("/home/benjamin/Documents/seniorProject/ros_ws/src/ros_gz_project_template/x3_bt/src/Route_BT.xml");
    printf("Tree Created BT\n");

    tree.tickWhileRunning();
    return 0;
}