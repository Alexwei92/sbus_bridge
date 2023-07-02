#include "sbus_bridge.h"
#include "sbus_bridge/SbusRosMessage.h"

namespace sbus_bridge
{

SBusBridge::SBusBridge(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      destructor_invoked_(false)
{
    if (!loadParameters())
    {
        ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
        ros::shutdown();
        return;
    }

    // Publisher
    received_sbus_msg_pub_ = nh_.advertise<sbus_bridge::SbusRosMessage>("received_sbus_message", 1);

    // Start serial port with receiver thread
    if (!setUpSBusSerialPort(port_name_, true))
    {
        ros::shutdown();
        return;
    }
}

SBusBridge::~SBusBridge()
{
    destructor_invoked_ = true;

    // Stop SBus receiver thread
    stopReceiverThread();

    // Close serial port
    disconnectSerialPort();
}

void SBusBridge::handleReceivedSbusMessage(const SBusMsg &received_sbus_msg)
{
    received_sbus_msg_pub_.publish(received_sbus_msg.toRosMessage());
}

bool SBusBridge::loadParameters()
{
    try 
    {
        pnh_.getParam("port_name", port_name_);
    } 
    catch (...) 
    {
        return false;
    }
    return true;
}

} // namespace sbus_bridge


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sbus_bridge");
    sbus_bridge::SBusBridge sbus_bridge;

    ros::spin();
    return 0;
}
