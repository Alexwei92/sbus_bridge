#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sbus_serial_port.h"
#include "sbus_msg.h"

namespace sbus_bridge {

class SBusBridge : public SBusSerialPort
{
public:
    SBusBridge(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    SBusBridge() : SBusBridge(ros::NodeHandle(), ros::NodeHandle("~")) {}
    virtual ~SBusBridge();

private:
    void watchdogThread();

    void handleReceivedSbusMessage(const SBusMsg &received_sbus_msg) override;

    bool loadParameters();
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Publishers
    ros::Publisher received_sbus_msg_pub_;

    std::atomic_bool destructor_invoked_;

    // Watchdog
    std::thread watchdog_thread_;
    std::atomic_bool stop_watchdog_thread_;
    ros::Time time_last_rc_msg_received_;

    // Parameters
    std::string port_name_;
    double rc_timeout_;
};

} // namespace sbus_bridge