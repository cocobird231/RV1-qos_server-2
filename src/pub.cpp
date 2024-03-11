#include <random>

#include "vehicle_interfaces/interactive_publisher.h"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/timesync.h"
#include "vehicle_interfaces/qos2.h"

#include "vehicle_interfaces/msg/wheel_state.hpp"
#include "vehicle_interfaces/msg/distance.hpp"

#define NODE_NAME "qospubtest_0_node"
#define TOPIC_NAME_0 "topic0"
#define TOPIC_NAME_1 "topic1"

using namespace std::chrono_literals;

class SamplePublisher : public vehicle_interfaces::TimeSyncNode, public vehicle_interfaces::QoSNode
{
private:
    std::shared_ptr<vehicle_interfaces::MultiInteractivePublisher<vehicle_interfaces::msg::WheelState> > pub0_;
    std::shared_ptr<vehicle_interfaces::MultiInteractivePublisher<vehicle_interfaces::msg::Distance> > pub1_;
    rclcpp::executors::SingleThreadedExecutor* exec_;
    std::thread* execTh_;
    rclcpp::TimerBase::SharedPtr timer0_;
    rclcpp::TimerBase::SharedPtr timer1_;
    const std::string nodeName_;

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

    std::atomic<bool> exitF_;

private:
    void _timer0Callback()
    {
        static uint64_t cnt = 0;
        auto msg = vehicle_interfaces::msg::WheelState();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = cnt;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = 20;

        msg.gear = vehicle_interfaces::msg::WheelState::GEAR_NEUTRAL;
        msg.steering = cnt % 512;
        msg.pedal_throttle = cnt % 256;
        msg.pedal_brake = cnt % 128;
        msg.pedal_clutch = cnt % 64;
        msg.button = cnt % 32;
        msg.func = cnt % 16;

        RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_timer0Callback] Publishing");
        this->pub0_->publish(msg);
    }

    void _timer1Callback()
    {
        static uint64_t cnt = 0;
        auto msg = vehicle_interfaces::msg::Distance();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_ULTRASONIC;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = cnt;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = 50;

        msg.unit_type = vehicle_interfaces::msg::Distance::UNIT_METER;
        msg.min = 0;
        msg.max = 10;

        static std::uniform_real_distribution<> uniDistrib{0.0, 1.0};
        msg.distance = uniDistrib(gen_) * 10.0;

        RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_timer1Callback] Publishing");
        this->pub1_->publish(msg);
    }

public:
    SamplePublisher(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::TimeSyncNode(gParams->nodeName, 
                                            gParams->timesyncService, 
                                            gParams->timesyncPeriod_ms, 
                                            gParams->timesyncAccuracy_ms, 
                                            gParams->timesyncWaitService), 
        vehicle_interfaces::QoSNode(gParams->nodeName, gParams->qosService, gParams->qosDirPath), 
        rclcpp::Node(gParams->nodeName), 
        nodeName_(gParams->nodeName), 
        exec_(nullptr), 
        execTh_(nullptr), 
        exitF_(false)
    {
        this->pub0_ = std::make_shared<vehicle_interfaces::MultiInteractivePublisher<vehicle_interfaces::msg::WheelState> >(this->nodeName_ + "_pub_0", TOPIC_NAME_0, 10);
        this->pub1_ = std::make_shared<vehicle_interfaces::MultiInteractivePublisher<vehicle_interfaces::msg::Distance> >(this->nodeName_ + "_pub_1", TOPIC_NAME_1, 10);

        this->exec_ = new rclcpp::executors::SingleThreadedExecutor();
        this->exec_->add_node(this->pub0_);
        this->exec_->add_node(this->pub1_);
        this->execTh_ = new std::thread(vehicle_interfaces::SpinExecutor, this->exec_, this->nodeName_, 1000);

        this->timer0_ = this->create_wall_timer(100ms, std::bind(&SamplePublisher::_timer0Callback, this));
        this->timer1_ = this->create_wall_timer(150ms, std::bind(&SamplePublisher::_timer1Callback, this));
        /*
         * Add following codes for QoSNode support
         */
        this->addQoSNodeCommand(this->pub0_, true);
        this->addQoSNodeCommand(this->pub1_, true);

        RCLCPP_INFO(this->get_logger(), "[SamplePublisher] Constructed.");
    }

    ~SamplePublisher()
    {
        this->close();
    }

    void close()
    {
        if (this->exitF_)
            return;
        if (this->exec_ != nullptr)
        {
            this->exec_->cancel();
            if (this->execTh_ != nullptr)
            {
                this->execTh_->join();
                delete this->execTh_;
            }
            delete this->exec_;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("qospubtest_params_node");
    params->nodeName = NODE_NAME;
    params->timesyncService = "";
    params->safetyService = "";
    auto timeSyncPub = std::make_shared<SamplePublisher>(params);
    std::this_thread::sleep_for(5s);
    rclcpp::spin(timeSyncPub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}