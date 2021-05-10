/*************************************************************************************************************
 * This suspension node listens for packets on the CANBUS that contain suspension data. That is the data that
 * contains a voltage value representing how far each suspension sensor is compressed/decompressed. One for
 * each wheel.
 ************************************************************************************************************/

#include "ros/ros.h"
#include <fsae_electric_vehicle/suspension.h>
#include "CANController.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Suspension");
  
    ros::NodeHandle n;

    ros::Publisher suspension_msg = n.advertise<fsae_electric_vehicle::suspension>("suspension", 1000);
  
    fsae_electric_vehicle::suspension suspension; // Constructor
  
    // Start CANBUS Header
    CANController can;
    can.start("can0");

    std::cout << "Suspension Init!" << std::endl;

    ros::Rate loop_rate(5);

    while (ros::ok()) {
        auto data = can.getData(0x02051881, 0x1FFFFFFF);
        if (data.has_value()) {
            suspension.frontLeft = data->data[0];
            suspension.frontRight = data->data[1];
            suspension.rearLeft = data->data[2];
            suspension.rearRight = data->data[3];
        }

        suspension_msg.publish(suspension);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
