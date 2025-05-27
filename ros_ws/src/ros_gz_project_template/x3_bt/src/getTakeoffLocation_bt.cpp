#include <behaviortree_cpp/action_node.h>

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
