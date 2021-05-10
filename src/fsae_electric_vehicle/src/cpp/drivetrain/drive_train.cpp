/**********************************************************************************************************************************
* Im unsure what sensor data this drive_train node is supposed to be collecting but it can be repurposed for any analog sensor
**********************************************************************************************************************************/

#include "ros/ros.h"
#include <fsae_electric_vehicle/drive_train.h>
#include "CANController.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "drivetrain");

  ros::NodeHandle n;

  ros::Publisher drivetrain_msg = n.advertise<fsae_electric_vehicle::drive_train>("drivetrain_voltage", 1000);

  fsae_electric_vehicle::drive_train drivetrain; // Constructor

  // Start CANBUS Header
  CANController can;
  can.start("can0");

  std::cout << "Drive_Train Init!" << std::endl;

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    auto data = can.getData(0x02051881, 0x1FFFFFFF);
    if (data.has_value())
      drivetrain.voltage = data->data[0];

    drivetrain_msg.publish(drivetrain);
    ros::spinOnce();
    loop_rate.sleep();
  }
}