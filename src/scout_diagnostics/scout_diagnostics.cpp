#include "scout_diagnostics/scout_diagnostics.h"

ScoutDiagnostics::ScoutDiagnostics(ros::NodeHandle nh, ros::NodeHandle priv_nh)
    : diagnostics(nh, priv_nh, ros::this_node::getName()), dynamic_reconfigure_server(priv_nh), node(nh)
{
    diagnostics.setHardwareID("scout");
    diagnostics.add("status", this, &ScoutDiagnostics::updateDiagnostics);

    dynamic_reconfigure::Server<scout_diagnostics::ScoutDiagnosticsConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigCallback(config, level); };
    dynamic_reconfigure_server.setCallback(cb);

    scout_status_sub = nh.subscribe("scout_status", 1, &ScoutDiagnostics::scoutStatusCallback, this);
    if (include_bms_states)
    {
        scout_bms_status_sub = nh.subscribe("BMS_status", 1, &ScoutDiagnostics::scoutBmsStatusCallback, this);
    }

    timer = nh.createTimer(ros::Duration(.1), &ScoutDiagnostics::periodicUpdate, this);
    timer.start();
}

ScoutDiagnostics::~ScoutDiagnostics()
{
}

void ScoutDiagnostics::updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (!scout_status_updated)
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Not connected.");
        return;
    }
    if (scout_status.fault_code > 0 || scout_status.base_state > 0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Error state (%d): %s.", scout_status.fault_code, getFaultMsg());
    }
    else if (scout_bms_status_updated && scout_bms_status.SOC < 20.0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Low battery %.2f%%.", scout_bms_status.SOC);
    }
    else if (scout_bms_status_updated && scout_bms_status.SOC < 10.0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical battery %.2f%%.", scout_bms_status.SOC);
    }
    else if (scout_bms_status_updated && (scout_bms_status.Warning_Status_1>0 || scout_bms_status.Warning_Status_2>0 || scout_bms_status.Alarm_Status_1>0 || scout_bms_status.Alarm_Status_2>0))
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "BMS alarm.");
    }
    else
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK.");
    }

    stat.add("Battery Voltage", scout_status.battery_voltage);
    stat.add("Base State", getBaseStateMsg());
    stat.add("Control Mode", getControlMode());
    if (include_velocity_states)
    {
        stat.add("Linear Velocity", scout_status.linear_velocity);
        stat.add("Angular Velocity", scout_status.angular_velocity);
        stat.add("Lateral Velocity", scout_status.lateral_velocity);
    }
    if (include_motor_driver_states)
    {
        stat.addf("Motor 1 State", "Current: %f, RPM: %f, Temperature: %f", scout_status.motor_states[0].current, scout_status.motor_states[0].rpm, scout_status.motor_states[0].temperature);
        stat.addf("Motor 2 State", "Current: %f, RPM: %f, Temperature: %f", scout_status.motor_states[1].current, scout_status.motor_states[1].rpm, scout_status.motor_states[1].temperature);
        stat.addf("Motor 3 State", "Current: %f, RPM: %f, Temperature: %f", scout_status.motor_states[2].current, scout_status.motor_states[2].rpm, scout_status.motor_states[2].temperature);
        stat.addf("Motor 4 State", "Current: %f, RPM: %f, Temperature: %f", scout_status.motor_states[3].current, scout_status.motor_states[3].rpm, scout_status.motor_states[3].temperature);
        stat.addf("Driver 1 State", "Voltage: %f, Temperature: %f", scout_status.driver_states[0].driver_voltage, scout_status.driver_states[0].driver_temperature);
        stat.addf("Driver 2 State", "Voltage: %f, Temperature: %f", scout_status.driver_states[1].driver_voltage, scout_status.driver_states[1].driver_temperature, scout_status.driver_states[1].driver_state);
        stat.addf("Driver 3 State", "Voltage: %f, Temperature: %f", scout_status.driver_states[2].driver_voltage, scout_status.driver_states[2].driver_temperature, scout_status.driver_states[2].driver_state);
        stat.addf("Driver 4 State", "Voltage: %f, Temperature: %f", scout_status.driver_states[3].driver_voltage, scout_status.driver_states[3].driver_temperature, scout_status.driver_states[3].driver_state);
        if (scout_status.driver_states[0].driver_state > 0)
        {
            stat.add("Driver 1 Error", getDriverStateMsg(scout_status.driver_states[0]));
        }
        if (scout_status.driver_states[1].driver_state > 0)
        {
            stat.add("Driver 2 Error", getDriverStateMsg(scout_status.driver_states[1]));
        }
        if (scout_status.driver_states[2].driver_state > 0)
        {
            stat.add("Driver 3 Error", getDriverStateMsg(scout_status.driver_states[2]));
        }
        if (scout_status.driver_states[3].driver_state > 0)
        {
            stat.add("Driver 4 Error", getDriverStateMsg(scout_status.driver_states[3]));
        }
    }
    if (include_light_states)
    {
        stat.add("Light Control Enabled", scout_status.light_control_enabled ? "True": "False");
        stat.add("Front Light Mode", getLightMode(scout_status.front_light_state.mode));
        stat.addf("Front Light Brightness (%)", "%d", scout_status.front_light_state.custom_value);
        stat.add("Rear Light Mode", getLightMode(scout_status.rear_light_state.mode));
        stat.addf("Rear Light Brightness (%)", "%d", scout_status.rear_light_state.custom_value);
    }
    if (include_bms_states && scout_bms_status_updated)
    {
        stat.add("BMS Charge (%)", scout_bms_status.SOC);
        stat.add("BMS Health (%)", scout_bms_status.SOH);
        stat.add("BMS Battery Voltage", scout_bms_status.battery_voltage);
        stat.add("BMS Battery Current", scout_bms_status.battery_current);
        stat.add("BMS Battery Temperature", scout_bms_status.battery_temperature);
        if (scout_bms_status.Alarm_Status_1 > 0 || scout_bms_status.Warning_Status_1 > 0 || scout_bms_status.Alarm_Status_2 > 0 || scout_bms_status.Warning_Status_2 > 0)
        {
            stat.add("BMS Alarm", getBMSAlarmMsg());
        }
    }
    scout_status_updated = false;
    scout_bms_status_updated = false;
}

void ScoutDiagnostics::periodicUpdate(const ros::TimerEvent &event)
{
    diagnostics.update();
}

void ScoutDiagnostics::scoutStatusCallback(const scout_msgs::ScoutStatus::ConstPtr &msg)
{
    scout_status = *msg;
    scout_status_updated = true;
}

void ScoutDiagnostics::scoutBmsStatusCallback(const scout_msgs::ScoutBmsStatus::ConstPtr &msg)
{
    scout_bms_status = *msg;
    scout_bms_status_updated = true;
}

void ScoutDiagnostics::reconfigCallback(scout_diagnostics::ScoutDiagnosticsConfig &config, uint32_t level)
{
    include_velocity_states = config.include_velocity_states;
    include_motor_driver_states = config.include_motor_driver_states;
    include_light_states = config.include_light_states;
    if (include_bms_states != config.include_bms_states)
    {
        if (include_bms_states)
        {
            scout_bms_status_sub.shutdown();
        }
        else
        {
            scout_bms_status_sub = node.subscribe("BMS_status", 1, &ScoutDiagnostics::scoutBmsStatusCallback, this);
        }
    }
    include_bms_states = config.include_bms_states;
}

// bit [0], Battery undervoltage fault (0: No failure 1: Failure) Protection voltage is 22V (The battery version with BMS, the protection power is 10%)|
// bit [1), Battery undervoltage fault[2] (0: No failure 1: Failure) Alarm voltage is 24V (The battery version with BMS, the warning power is 15%)
// bit [2], RC transmitter disconnection protection (0: Normal 1: RC transmitter disconnected)
// bit [3], No.1 motor communication failure (0: No failure 1: Failure)
// bit [4], No.2 motor communication failure (0: No failure 1: Failure)
// bit [5], No.3 motor communication failure (0: No failure 1: Failure)
// bit [6], No.4 motor communication failure (0: No failure 1: Failure)
// bit [7], Reserved, default 0
// ref: https://www.generationrobots.com/media/agilex/SCOUT2.0_UserManual_v2.0_EN
std::string ScoutDiagnostics::getFaultMsg()
{
    std::string msg;
    if (scout_status.fault_code & 1)
    {
        msg += "critical battery 10%, ";
    }
    if (scout_status.fault_code & (1 << 1))
    {
        msg += "low battery 15%, ";
    }
    if (scout_status.fault_code & (1 << 2))
    {
        msg += "RC transmitter disconnected, ";
    }
    if (scout_status.fault_code & (1 << 3))
    {
        msg += "motor 1 communication failure, ";
    }
    if (scout_status.fault_code & (1 << 4))
    {
        msg += "motor 2 communication failure, ";
    }
    if (scout_status.fault_code & (1 << 5))
    {
        msg += "motor 3 communication failure, ";
    }
    if (scout_status.fault_code & (1 << 6))
    {
        msg += "motor 4 communication failure, ";
    }
    return msg;
}

std::string ScoutDiagnostics::getBaseStateMsg()
{
    std::string msg;
    switch (scout_status.base_state)
    {
        case 0:
            msg = "Normal";
            break;
        case 1:
            msg = "Emergency stop";
            break;
        case 2:
            msg = "System exception";
            break;
    }
    return msg;
}

std::string ScoutDiagnostics::getControlMode()
{
    std::string msg;
    switch (scout_status.control_mode)
    {
        case 0:
            msg = "Standby";
            break;
        case 1:
            msg = "CAN command";
            break;
        case 2:
            msg = "Serial port";
            break;
        case 3:
            msg = "RC";
            break;
    }
    return msg;
}

// bit [0], Whether the power supply voltage is too low (0:Normal 1: Too low)
// bit [1], Whether the motor is overheated (0:Normal 1:Overheated)
// bit [2], Whether the drive is over current (0:Normal 1:Over current)
// bit [3], Whether the drive is overheated (0:Normal 1:Overheated)
// bit [4], Sensor status (0:Normal 1:Abnormal)
// bit [5], Drive error status (0:Normal 1: Error)
// bit [6], Drive enable status (0:Normal 1:Disability)
// bit [7], Reserved
std::string ScoutDiagnostics::getDriverStateMsg(scout_msgs::ScoutDriverState &driver_state)
{
    std::string msg;
    if (driver_state.driver_state & 1)
    {
        msg += "low voltage, ";
    }
    if (driver_state.driver_state & (1 << 1))
    {
        msg += "motor overheated, ";
    }
    if (driver_state.driver_state & (1 << 2))
    {
        msg += "drive overcurrent, ";
    }
    if (driver_state.driver_state & (1 << 3))
    {
        msg += "drive overheated, ";
    }
    if (driver_state.driver_state & (1 << 4))
    {
        msg += "sensor abnormal, ";
    }
    if (driver_state.driver_state & (1 << 5))
    {
        msg += "drive error, ";
    }
    if (driver_state.driver_state & (1 << 6))
    {
        msg += "drive disabled, ";
    }
    return msg;
}

// Alarm Status 1
//  BIT1: Overvoltage 
//  BIT2: Undervoltage 
//  BIT3: High temperature
//  BIT4: Low temperature 
//  BIT7: Discharge overcurrent
// Alarm Status 2
//  BITO: Charge overcurrent
// Warning Status 3
//  BIT1: Overvoltage 
//  BIT2: Undervoltage 
//  BIT3: High temperature
//  BIT4: Low temperature 
//  BIT7: Discharge overcurrent
// Warning Status 4
//  BITO: Charge overcurrent
// ref: https://static.generation-robots.com/media/user-manual-agilex-robotics-hunter-2.0-de.pdf
std::string ScoutDiagnostics::getBMSAlarmMsg()
{
    std::string msg;
    if (scout_bms_status.Alarm_Status_1 & (1 << 1) || scout_bms_status.Warning_Status_1 & (1 << 1))
    {
        msg += "overvoltage, ";
    }
    if (scout_bms_status.Alarm_Status_1 & (1 << 2) || scout_bms_status.Warning_Status_1 & (1 << 2))
    {
        msg += "undervoltage, ";
    }
    if (scout_bms_status.Alarm_Status_1 & (1 << 3) || scout_bms_status.Warning_Status_1 & (1 << 3))
    {
        msg += "high_temerature, ";
    }
    if (scout_bms_status.Alarm_Status_1 & (1 << 4) || scout_bms_status.Warning_Status_1 & (1 << 4))
    {
        msg += "low_temperature, ";
    }
    if (scout_bms_status.Alarm_Status_1 & (1 << 7) || scout_bms_status.Warning_Status_1 & (1 << 7))
    {
        msg += "discharge_overcurrent, ";
    }
    if (scout_bms_status.Alarm_Status_2 & 1 || scout_bms_status.Warning_Status_2 & 1)
    {
        msg += "charge_overcurrent, ";
    }
    return msg;
}

std::string ScoutDiagnostics::getLightMode(const uint8_t &mode)
{
    std::string msg;
    if (mode == 0)
    {
        msg = "NC";
    }
    else if (mode == 1)
    {
        msg = "NO";
    }
    else if (mode == 2)
    {
        msg = "Blink mode";
    }
    else if (mode == 3)
    {
        msg = "User-defined brightness";
    }
    return msg;
}