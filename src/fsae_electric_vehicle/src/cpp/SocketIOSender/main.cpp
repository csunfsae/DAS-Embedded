/********************************************************************************************************************
 * This is the networking code that sends the vehicle data to the server using SocketIO
 *******************************************************************************************************************/
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "fsae_electric_vehicle/serial.h"
#include "fsae_electric_vehicle/speedometer.h"
#include "fsae_electric_vehicle/brake_pressure.h"
#include "fsae_electric_vehicle/coolant.h"
#include "fsae_electric_vehicle/drive_train.h"
#include "fsae_electric_vehicle/suspension.h"
#include "fsae_electric_vehicle/gps.h"
//#include "/home/nvidia/Desktop/formulaEmbedded/src/fsae_electric_vehicle/socket.io-client-cpp/src/sio_client.h" //Change to directory where the header file is at or it won't compile. Seth & Faizan you can add your include that is specific to your computers directory, just make sure all of the other includes for sio_client.h are commented out.
#include <sio_client.h>
#include <json.hpp>
//#include "/home/btc54/Desktop/formulaEmbedded/src/fsae_electric_vehicle/socket.io-client-cpp/src/sio_client.h" 
#include <string>
#include <mutex>
#include <fstream>
//#include <socket.io-client-cpp/src/sio_client.h>
//#include "fsae_electric_vehicle/sio_client.h"

  using namespace nlohmann;

  float speedVal, driveTrain, coolantTemp, brakePressure;
  
  float suspFrontLeft, suspFrontRight, suspRearLeft, suspRearRight;
  
  float gpsHours, gpsMinutes, gpsSeconds, gpsFix, gpsLat, gpsLon, gpsSpeed, gpsHeading, gpsLapEndTime, gpsCurrentLapTimer;
  
  json carLocation;
  
  std::mutex dataMutex;

  void speedCallback(const fsae_electric_vehicle::speedometer::ConstPtr& msg) {
 	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&speedVal, &msg->speed, sizeof(speedVal)+1);
 }
  
  void gpsCallback(const fsae_electric_vehicle::gps::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock{dataMutex};
  	memcpy(&gpsHours, &msg->hours, sizeof(speedVal)+1);
  	memcpy(&gpsMinutes, &msg->minutes, sizeof(speedVal)+1);
  	memcpy(&gpsSeconds, &msg->seconds, sizeof(speedVal)+1);
  	memcpy(&gpsFix, &msg->fix, sizeof(speedVal)+1);
	  //memcpy(&gpsLat, &msg->latitude, sizeof(speedVal)+1);
	  //memcpy(&gpsLon, &msg->longitude, sizeof(speedVal)+1);
	  memcpy(&gpsSpeed, &msg->speed, sizeof(speedVal)+1);
	  memcpy(&gpsHeading, &msg->heading, sizeof(speedVal)+1);
    memcpy(&gpsLapEndTime, &msg->lapEndTime, sizeof(speedVal)+1);
    memcpy(&gpsCurrentLapTimer, &msg->currentLapTimer, sizeof(speedVal)+1);
 }

 void brakeCallback(const fsae_electric_vehicle::brake_pressure::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&brakePressure, &msg->pressure, sizeof(speedVal)+1);
 }

 void coolantCallback(const fsae_electric_vehicle::coolant::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&coolantTemp, &msg->temperature, sizeof(speedVal)+1);
 }

 void batteryCallback(const fsae_electric_vehicle::drive_train::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&driveTrain, &msg->voltage, sizeof(speedVal)+1);
 }
 
 void suspensionCallback(const fsae_electric_vehicle::suspension::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock{dataMutex};
 	memcpy(&suspFrontLeft, &msg->frontLeft, sizeof(speedVal)+1);
 	memcpy(&suspFrontRight, &msg->frontRight, sizeof(speedVal)+1);
 	memcpy(&suspRearLeft, &msg->rearLeft, sizeof(speedVal)+1);
 	memcpy(&suspRearRight, &msg->rearRight, sizeof(speedVal)+1);
 }
 
sio::message::ptr createObject(json o)
{
    sio::message::ptr object = sio::object_message::create();

    for (json::iterator it = o.begin(); it != o.end(); ++it)
    {
        auto key = it.key();
        auto v = it.value();

        if (v.is_boolean())
        {
            object->get_map()[key] = sio::bool_message::create(v.get<bool>());
        }
        else if (v.is_number_float())
        {
            object->get_map()[key] = sio::double_message::create(v.get<double>());
        }
        else if (v.is_string())
        {
            object->get_map()[key] = sio::string_message::create(v.get<std::string>());
        }
        else if (v.is_object())
        {
            json childObject = v;
            object->get_map()[key] = createObject(childObject);
        }
    }
    return object;
}

int main(int argc, char **argv) {

  sio::client h;
  h.connect("http://64.227.48.74:2222");
  
  //sio::message::list speedData(speedVal);
  //sio::message::list speedData(std::to_string(speedVal));


  ros::init(argc, argv, "SocketIOSender");
  ros::NodeHandle n;
  

  ros::Subscriber speed = n.subscribe<fsae_electric_vehicle::speedometer>("speedometer", 1000, speedCallback);
  ros::Subscriber gps = n.subscribe<fsae_electric_vehicle::gps>("gps_lap_timer", 1000, gpsCallback);
  ros::Subscriber bat = n.subscribe<fsae_electric_vehicle::drive_train>("drivetrain_voltage", 1000, batteryCallback);
  ros::Subscriber cool = n.subscribe<fsae_electric_vehicle::coolant>("coolant_temperature", 1000, coolantCallback);
  ros::Subscriber brake = n.subscribe<fsae_electric_vehicle::brake_pressure>("brake_pressure", 1000, brakeCallback);
  ros::Subscriber susp = n.subscribe<fsae_electric_vehicle::suspension>("suspension", 1000, suspensionCallback);
  

  
  ros::Rate loop_rate{30};

  while (ros::ok()) {
    ros::spinOnce();
    
    carLocation["latitude"] = 314.11;
    carLocation["longitude"] = 1115.321;
    
    //std::ifstream ifs("carLocation.json");
    //json parseLocation = json::parse(ifs);


    h.socket()->emit("speedometer", sio::double_message::create(speedVal));
    h.socket()->emit("driveTrain", sio::double_message::create(brakePressure));
    h.socket()->emit("coolantTemp", sio::double_message::create(coolantTemp));
    h.socket()->emit("brakePressure", sio::double_message::create(driveTrain));
    
    h.socket()->emit("suspFrontLeft", sio::double_message::create(suspFrontLeft));
    h.socket()->emit("suspFrontRight", sio::double_message::create(suspFrontRight));
    h.socket()->emit("suspRearLeft", sio::double_message::create(suspRearLeft));
    h.socket()->emit("suspRearRight", sio::double_message::create(suspRearRight));
    
    h.socket()->emit("gpsHours", sio::double_message::create(gpsHours));
    h.socket()->emit("gpsMinutes", sio::double_message::create(gpsMinutes));
    h.socket()->emit("gpsSeconds", sio::double_message::create(gpsSeconds));
    h.socket()->emit("gpsFix", sio::double_message::create(gpsFix));
    //h.socket()->emit("gpsLat", sio::double_message::create(gpsLat));
    //h.socket()->emit("gpsLon", sio::double_message::create(gpsLon));
    h.socket()->emit("gpsSpeed", sio::double_message::create(gpsSpeed));
    h.socket()->emit("gpsHeading", sio::double_message::create(gpsHeading));
    h.socket()->emit("gpsEndLapTime", sio::double_message::create(gpsLapEndTime));
    h.socket()->emit("gpsCurrentLapTimer", sio::double_message::create(gpsCurrentLapTimer));

    sio::message::ptr object = createObject(carLocation);
    h.socket()->emit("carLocation", object);
    
    loop_rate.sleep();
  }
}
