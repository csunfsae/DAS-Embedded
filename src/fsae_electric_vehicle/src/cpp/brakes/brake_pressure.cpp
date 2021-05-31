/*********************************************************************************************************************
* Brake pressure node listens for packets on the CANBUS with brake pressure data
*********************************************************************************************************************/

#include "ros/ros.h"
#include <fsae_electric_vehicle/brake_pressure.h>
#include <CANController.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "brake_pressure");

  ros::NodeHandle n;

  ros::Publisher brake_pressure_msg = n.advertise<fsae_electric_vehicle::brake_pressure>("brake_pressure", 1000);

  fsae_electric_vehicle::brake_pressure brake_pressure; // Constructor

  // Start CANBUS Header
  CANController can;
  can.start("can0");

  std::cout << "Brake_Pressure Init!" << std::endl;

  ros::Rate loop_rate(50);
  float lastVal = 0;

  while (ros::ok()) {
    auto data = can.getData(0x02051884, 0x1FFFFFFF);
    if (data.has_value())
      std::memcpy(&lastVal, data->data, 1);

    brake_pressure.pressure = lastVal;
    brake_pressure_msg.publish(brake_pressure);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
