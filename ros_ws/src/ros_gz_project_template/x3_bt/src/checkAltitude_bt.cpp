#include <behaviortree_cpp/action_node.h>

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
