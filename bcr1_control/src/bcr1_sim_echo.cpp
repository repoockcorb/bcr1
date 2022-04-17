#include <ros/ros.h>
#include <bcr1_control/armCmd.h>
#include <bcr1_control/bcr1Telemetry.h>

ros::Publisher telem_pub;

void cmdCallback(const bcr1_control::armCmd::ConstPtr &msg){
    /*
    COMMAND
        float32[6] effort # 0-255 PWM forArduino
        float32[6] angle # deg
        #uint32 msg_ctr # count sent msgs to detect missed messages

    TELEMETRY
        float32[6] angle # degrees
    */

    static bcr1_control::bcr1Telemetry telem;

    for(int i=0; i<6; i++){
        telem.angle[i]=msg->angle[i];

    }

    telem_pub.publish(telem);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "bcr1_sim_echo");

    ros::NodeHandle n;

    ros::Subscriber cmd_sub=n.subscribe("/arduino/armCmd", 10, cmdCallback);

    telem_pub=n.advertise<bcr1_control::bcr1Telemetry>("/arduino/bcr1Telemetry", 10);

    ros::spin();
}

