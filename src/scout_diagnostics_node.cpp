#include <ros/ros.h>
#include "scout_diagnostics/scout_diagnostics.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scout_diagnostics");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    ScoutDiagnostics scout_diagnostics(nh, priv_nh);

    ros::spin();
}