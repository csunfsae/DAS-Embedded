#include "ros/ros.h"
#include <fsae_electric_vehicle/suspension.h>
#include "CANController.h"

int main(int argc, char **argv) {
  std::cout << "Starting Suspension" << std::endl;

  ros::init(argc, argv, "Suspension");
  std::cout << "initialized suspension node" << std::endl;
  
  ros::NodeHandle n;
  std::cout << "After node handle calling ros::start()" << std::endl;

  ros::Publisher suspension_msg = n.advertise<fsae_electric_vehicle::suspension>("suspension", 1000);
  std::cout << "After publisher" << std::endl;
  
  fsae_electric_vehicle::suspension suspension; //constructor
 
  ros::Rate loop_rate(5);
  std::cout << "listening" << std::endl;
  
  CANController can;
  can.start("can0");

  float lastVal = 0;

  while (ros::ok()) {
    auto data = can.getData(0x02051881, 0x1FFFFFFF);
    if (data.has_value()) {
      std::memcpy(&lastVal, data->data, 4);
    }
    lastVal += .1;
    suspension.frontLeft = 0;
    suspension.frontRight = 22;
    suspension.rearLeft = 33;
    suspension.rearRight = 44;
    suspension_msg.publish(suspension);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
