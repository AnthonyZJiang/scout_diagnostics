#pragma once

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <scout_msgs/ScoutStatus.h>
#include <scout_msgs/ScoutMotorState.h>
#include <scout_msgs/ScoutDriverState.h>
#include <scout_msgs/ScoutLightState.h>
#include <scout_msgs/ScoutBmsStatus.h>
#include "scout_diagnostics/ScoutDiagnosticsConfig.h"


class ScoutDiagnostics
{

public:
    ScoutDiagnostics(ros::NodeHandle nh, ros::NodeHandle priv_nh);
    ~ScoutDiagnostics();

private:
    ros::Timer timer;
    ros::Subscriber scout_status_sub;
    ros::Subscriber scout_bms_status_sub;
    diagnostic_updater::Updater diagnostics;
    dynamic_reconfigure::Server<scout_diagnostics::ScoutDiagnosticsConfig> dynamic_reconfigure_server;

    scout_msgs::ScoutStatus scout_status;
    scout_msgs::ScoutBmsStatus scout_bms_status;
    bool scout_status_updated = false;
    bool scout_bms_status_updated = false;

    bool include_velocity_states = true;
    bool include_motor_driver_states = false;
    bool include_light_states = false;
    bool include_bms_states = false;

    void periodicUpdate(ros::TimerEvent const &event);
    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void scoutStatusCallback(const scout_msgs::ScoutStatus::ConstPtr &msg);
    void scoutBmsStatusCallback(const scout_msgs::ScoutBmsStatus::ConstPtr &msg);
    void reconfigCallback(scout_diagnostics::ScoutDiagnosticsConfig &config, uint32_t level);

    std::string getFaultMsg();
    std::string getBaseStateMsg();
    std::string getControlMode();
    std::string getDriverStateMsg(scout_msgs::ScoutDriverState &driver_state);
    std::string getBMSAlarmMsg();
    std::string getLightMode(const uint8_t &mode);
};