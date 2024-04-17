#include "vehicle_interfaces/interactive_subscription.h"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/timesync.h"
#include "vehicle_interfaces/qos2.h"

#include "vehicle_interfaces/msg/wheel_state.hpp"
#include "vehicle_interfaces/msg/distance.hpp"

#define NODE_NAME "qossubtest_0_node"
#define TOPIC_NAME_0 "topic0"
#define TOPIC_NAME_1 "topic1"

using namespace std::chrono_literals;

class SampleSubscriber : public vehicle_interfaces::TimeSyncNode, public vehicle_interfaces::QoSNode
{
private:
    std::shared_ptr<vehicle_interfaces::MultiInteractiveSubscription<vehicle_interfaces::msg::WheelState> > sub0_;
    std::shared_ptr<vehicle_interfaces::MultiInteractiveSubscription<vehicle_interfaces::msg::Distance> > sub1_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
    vehicle_interfaces::unique_thread execTh_;
    const std::string nodeName_;

    std::atomic<bool> exitF_;

private:
    void _topic0Callback(const vehicle_interfaces::msg::WheelState::SharedPtr msg)
    {
        static int64_t cnt = 0;
        RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_topic0Callback] Heard!");
    }

    void _topic1Callback(const vehicle_interfaces::msg::Distance::SharedPtr msg)
    {
        static int64_t cnt = 0;
        RCLCPP_INFO(this->get_logger(), "[SampleSubscriber::_topic1Callback] Heard!");
    }

public:
    SampleSubscriber(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::TimeSyncNode(gParams->nodeName, 
                                            gParams->timesyncService, 
                                            gParams->timesyncPeriod_ms, 
                                            gParams->timesyncAccuracy_ms, 
                                            gParams->timesyncWaitService), 
        vehicle_interfaces::QoSNode(gParams->nodeName, gParams->qosService, gParams->qosDirPath), 
        rclcpp::Node(gParams->nodeName), 
        nodeName_(gParams->nodeName)
    {
        this->sub0_ = std::make_shared<vehicle_interfaces::MultiInteractiveSubscription<vehicle_interfaces::msg::WheelState> >(this->nodeName_ + "_sub_0", TOPIC_NAME_0, 10, std::bind(&SampleSubscriber::_topic0Callback, this, std::placeholders::_1));
        this->sub1_ = std::make_shared<vehicle_interfaces::MultiInteractiveSubscription<vehicle_interfaces::msg::Distance> >(this->nodeName_ + "_sub_1", TOPIC_NAME_1, 10, std::bind(&SampleSubscriber::_topic1Callback, this, std::placeholders::_1));

        this->exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->exec_->add_node(this->sub0_);
        this->exec_->add_node(this->sub1_);
        this->execTh_ = vehicle_interfaces::make_unique_thread(vehicle_interfaces::SpinExecutor, this->exec_, this->nodeName_, 1000);

        /*
         * Add following codes for QoSNode support
         */
        this->addQoSNodeCommand(this->sub0_, true);
        this->addQoSNodeCommand(this->sub1_, true);

        RCLCPP_INFO(this->get_logger(), "[SampleSubscriber] Constructed.");
    }

    ~SampleSubscriber()
    {
        this->close();
    }

    void close()
    {
        if (this->exitF_)
            return;
        this->exitF_ = true;
        this->exec_->cancel();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("qossubtest_params_node");
    params->nodeName = NODE_NAME;
    params->timesyncService = "";
    params->safetyService = "";
    auto timeSyncSub = std::make_shared<SampleSubscriber>(params);
    rclcpp::spin(timeSyncSub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}