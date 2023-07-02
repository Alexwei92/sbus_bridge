#include "sbus_bridge.h"
#include "sbus_bridge/SbusRosMessage.h"

namespace sbus_bridge
{

SBusBridge::SBusBridge(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      stop_watchdog_thread_(false),
      time_last_rc_msg_received_(),
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

    // Start watchdog thread
    try
    {
        watchdog_thread_ = std::thread(&SBusBridge::watchdogThread, this);
    }
    catch (...)
    {
        ROS_ERROR("[%s] Could not successfully start watchdog thread.",
            pnh_.getNamespace().c_str());
        ros::shutdown();
        return;
    }
}

SBusBridge::~SBusBridge()
{
    destructor_invoked_ = true;

    // Stop watchdog thread
    stop_watchdog_thread_ = true;
    // Wait for watchdog thread to finish
    watchdog_thread_.join();

    // Stop SBus receiver thread
    stopReceiverThread();

    // Close serial port
    disconnectSerialPort();
}

void SBusBridge::watchdogThread()
{
    ros::Rate watchdog_rate(50.0);
    while (ros::ok() && !stop_watchdog_thread_)
    {
        watchdog_rate.sleep();

        const ros::Time time_now = ros::Time::now();

        if (time_now - time_last_rc_msg_received_ > ros::Duration(rc_timeout_)) 
        {
            ROS_WARN(
                "[%s] Remote control was active but no message from it was received "
                "within timeout (%f s).",
                pnh_.getNamespace().c_str(), rc_timeout_);
        }
    }
}

void SBusBridge::handleReceivedSbusMessage(const SBusMsg &received_sbus_msg)
{
    time_last_rc_msg_received_ = ros::Time::now();
    received_sbus_msg_pub_.publish(received_sbus_msg.toRosMessage());
}

bool SBusBridge::loadParameters()
{
    try 
    {
        pnh_.getParam("port_name", port_name_);
        pnh_.getParam("rc_timeout", rc_timeout_);
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
