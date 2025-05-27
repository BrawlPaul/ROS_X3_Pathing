#include <behaviortree_cpp/action_node.h>

using namespace BT;

class CheckBattery : public SyncActionNode
{
    int battery_level;
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
};
