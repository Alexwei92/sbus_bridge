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
    void handleReceivedSbusMessage(const SBusMsg &received_sbus_msg) override;

    bool loadParameters();
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Publishers
    ros::Publisher received_sbus_msg_pub_;

    std::atomic_bool destructor_invoked_;

    // Parameters
    std::string port_name_;
};

} // namespace sbus_bridge