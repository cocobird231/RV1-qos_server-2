#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/qos2.h"

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string recordFilePath = "qosserver/qos/record.json";

private:
    void _getParams()
    {
        this->get_parameter("recordFilePath", this->recordFilePath);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("recordFilePath", this->recordFilePath);
        this->_getParams();
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("qosserver_params_node");
    auto server = std::make_shared<vehicle_interfaces::QoSServer>(params->nodeName, params->qosService, params->recordFilePath);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
