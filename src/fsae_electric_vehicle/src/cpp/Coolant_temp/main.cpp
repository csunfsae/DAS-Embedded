#include "ros/ros.h"
#include <fsae_electric_vehicle/coolant.h>
#include <CANController.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "Coolant");

  ros::NodeHandle n;

  ros::Publisher coolant_msg = n.advertise<fsae_electric_vehicle::coolant>("coolant_temperature", 1000);

  fsae_electric_vehicle::coolant temperature;

  std::cout << "Coolant Init!" << std::endl;

  // Start CANBUS header
  CANController can;
  can.start("can0");

  ros::Rate loop_rate(5);
  float lastVal = 0;

  while (ros::ok()) {
    auto data = can.getData(0x0A080F03, 0x1FFFFFFF);
    if (data.has_value())
      std::memcpy(&lastVal, data->data, 1);

    temperature.temperature = lastVal;//was lastVal
    //float x = 21.0;
    //fsae_electric_vehicle::coolant temperature2 = temperature.x;
    coolant_msg.publish(temperature);//was temperature
    ros::spinOnce();
    loop_rate.sleep();
  }
}
