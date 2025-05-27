#include <behaviortree_ros2/bt_service_node.hpp>
#include "nav2_msgs/srv/load_map.hpp"

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
    std::string map_name;
    float map_height;
    getInput("cur_alt", map_height);
    map_name = std::to_string(map_height);
    std::replace(map_name.begin(), map_name.end(), '.', '_');
    map_name = map_path + map_name + "Map.yaml" //get the actual path
    request->map_url = map_name;
    // must return true if we are ready to send the request
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    if (response->result)
    {
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
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }
};