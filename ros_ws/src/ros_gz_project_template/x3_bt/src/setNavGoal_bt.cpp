#include <behaviortree_ros2/bt_action_node.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose
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

    // get "x and y" from the Input port
    getInput("x", pose.pose.position.x);
    getInput("y", pose.pose.position.x);
    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus(const WrappedResult& wr) override
  {
    rclcpp_action::ResultCode result = wr->result.code; //get result code
    if (result == rclcpp_action::ResultCode::SUCCEEDED)
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
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
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
    geometry_msgs::msg::PoseStamped current_pose = feedback->current_pose;
    return NodeStatus::RUNNING;
  }
};