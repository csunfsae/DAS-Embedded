/********************************************************************************************************************
 * This is the networking code that sends the vehicle data to the server using SocketIO
 *******************************************************************************************************************/

#include "sioSender.h"
#include <iostream>
#include <fstream>

#define OUTPUT_DATA_TO_TXT_FILE true

int main(int argc, char **argv)
{
  sio::client h;
  h.connect("http://64.227.48.74:2222");

  float speedVal = -1.0f, brakePressure = -1.0f, coolantTemp = -1.0f, driveTrain = -1.0f;
  float suspensionFrontLeft = -1.0f, suspensionFrontRight = -1.0f, suspensionRearLeft = -1.0f, suspensionRearRight = -1.0f;  
  float gpsHours = -1.0f, gpsMinutes = -1.0f, gpsSeconds = -1.0f, gpsFix = -1.0f, gpsLat = -1.0f, gpsLon = -1.0f,
        gpsSpeed = -1.0f, gpsHeading = -1.0f, gpsLapEndTime = -1.0f, gpsCurrentLapTimer = -1.0f;
  
  //sio::message::list speedData(speedVal);
  //sio::message::list speedData(std::to_string(speedVal));

  ros::init(argc, argv, "SocketIOSender");
  ros::NodeHandle n;

  ros::Subscriber speed = n.subscribe<fsae_electric_vehicle::speedometer> ("speedometer", 1000, boost::bind(speedCallback, _1, speedVal));
  ros::Subscriber gps = n.subscribe<fsae_electric_vehicle::gps> ("gps_lap_timer", 1000, boost::bind(gpsCallback, _1, gpsHours, gpsMinutes, gpsSeconds, gpsFix, gpsSpeed, gpsHeading, gpsLapEndTime, gpsCurrentLapTimer));
  ros::Subscriber bat = n.subscribe<fsae_electric_vehicle::drive_train> ("drivetrain_voltage", 1000, boost::bind(batteryCallback, _1, driveTrain));
  ros::Subscriber cool = n.subscribe<fsae_electric_vehicle::coolant> ("coolant_temperature", 1000, boost::bind(coolantCallback, _1, coolantTemp));
  ros::Subscriber brake = n.subscribe<fsae_electric_vehicle::brake_pressure> ("brake_pressure", 1000, boost::bind(brakeCallback, _1, brakePressure));
  ros::Subscriber susp = n.subscribe<fsae_electric_vehicle::suspension>("suspension", 1000, boost::bind(suspensionCallback, _1, suspensionFrontLeft, suspensionFrontRight, suspensionRearLeft, suspensionRearRight));

  static std::ofstream collectedData;

  if (OUTPUT_DATA_TO_TXT_FILE) {
    collectedData.open("collectedData.txt", std::ios::out);

    if (!collectedData.is_open()) {
      ROS_WARN("Could not open file for data logging");
    }
  }

  ros::Rate loop_rate{40};

  while (ros::ok()) {
    ros::spinOnce();
    
    carLocation["latitude"] = 314.11;
    carLocation["longitude"] = 115.321;
    
    //std::ifstream ifs("carLocation.json");
    //json parseLocation = json::parse(ifs);

    // Check each float to see if its == -1. If it is -1 then send it to server, else dont send it.
    if (speedVal != -1.0f) {
      h.socket()->emit("speedometer", sio::double_message::create(speedVal));
      logToFile("Speed", speedVal, collectedData);
    }

    if (brakePressure != -1.0f) {
      h.socket()->emit("brakePressure", sio::double_message::create(brakePressure));
      logToFile("Brake Pressure", brakePressure, collectedData);
    }

    if (coolantTemp != -1.0f) {
      h.socket()->emit("coolantTemp", sio::double_message::create(coolantTemp));
      logToFile("Coolant Temp", coolantTemp, collectedData);
    }

    if (driveTrain != -1.0f) {
      h.socket()->emit("driveTrain", sio::double_message::create(driveTrain));
      logToFile("Drive Train", driveTrain, collectedData);
    }

    if (suspensionFrontLeft != -1.0f) {
      h.socket()->emit("suspensionFrontLeft", sio::double_message::create(suspensionFrontLeft));  
      logToFile("Suspension FL", suspensionFrontLeft, collectedData);
    }

    if (suspensionFrontRight != -1.0f) {
      h.socket()->emit("suspensionFrontRight", sio::double_message::create(suspensionFrontRight));  
      logToFile("Suspension FR", suspensionFrontRight, collectedData);
    }

    if (suspensionRearLeft != -1.0f) {
      h.socket()->emit("suspensionRearLeft", sio::double_message::create(suspensionRearLeft));  
      logToFile("SuspensionRearLeft", suspensionRearLeft, collectedData);
    }

    if (suspensionRearRight != -1.0f) {
      h.socket()->emit("suspensionRearRight", sio::double_message::create(suspensionRearRight));
      logToFile("SuspensionRearRight", suspensionRearRight, collectedData);
    }

    if (gpsHours != -1.0f) {
      h.socket()->emit("gpsHours", sio::double_message::create(gpsHours));
      logToFile("Hours", gpsHours, collectedData);
    }
    if (gpsMinutes != -1.0f) {
      h.socket()->emit("gpsMinutes", sio::double_message::create(gpsMinutes));
      logToFile("Minutes", gpsMinutes, collectedData);
    }

    if (gpsSeconds != -1.0f) {
      h.socket()->emit("gpsSeconds", sio::double_message::create(gpsSeconds));
      logToFile("Seconds", gpsSeconds, collectedData);
    }

    if (gpsFix != -1.0f) {
      h.socket()->emit("gpsFix", sio::double_message::create(gpsFix));
      logToFile("Fix", gpsFix, collectedData);
    }

    //if (gpsLat != -1.0f)
      //h.socket()->emit("gpsLat", sio::double_message::create(gpsLat));

    //if (gpsLon != -1.0f)
      //h.socket()->emit("gpsLon", sio::double_message::create(gpsLon));

    if (gpsSpeed != -1.0f) {
      h.socket()->emit("gpsSpeed", sio::double_message::create(gpsSpeed));
      logToFile("Speed ", gpsSpeed, collectedData);
    }

    if (gpsHeading != -1.0f) {
      h.socket()->emit("gpsHeading", sio::double_message::create(gpsHeading));
      logToFile("Heading", gpsHeading, collectedData);
    }

    if (gpsLapEndTime != -1.0f) {
      h.socket()->emit("gpsEndLapTime", sio::double_message::create(gpsLapEndTime));
      logToFile("Lap Time", gpsLapEndTime, collectedData);
    }

    if (gpsCurrentLapTimer != -1.0f) {
      h.socket()->emit("gpsCurrentLapTimer", sio::double_message::create(gpsCurrentLapTimer));
      logToFile("Current Lap Time", gpsCurrentLapTimer, collectedData);
    }


    sio::message::ptr object = createObject(carLocation);
    h.socket()->emit("carLocation", object);

    // Set all floats to be -1 to indicate that they've been sent to the server.
    ResetValues(speedVal, driveTrain, coolantTemp, brakePressure,
                suspensionFrontLeft, suspensionFrontRight, suspensionRearLeft, suspensionRearRight,
                gpsHours, gpsMinutes, gpsSeconds, gpsFix, gpsSpeed, gpsHeading, gpsLapEndTime, gpsCurrentLapTimer);
    
    loop_rate.sleep();
  }

  collectedData.close();
}



void speedCallback(const fsae_electric_vehicle::speedometer::ConstPtr& msg, float& speedVal) {
  std::lock_guard<std::mutex> lock{dataMutex};
  memcpy(&speedVal, &msg->speed, sizeof(speedVal)+1);
}



void brakeCallback(const fsae_electric_vehicle::brake_pressure::ConstPtr& msg, float& brakePressure) { 
  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&brakePressure, &msg->pressure, sizeof(brakePressure)+1);
}



void coolantCallback(const fsae_electric_vehicle::coolant::ConstPtr& msg, float& coolantTemp) {
  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&coolantTemp, &msg->temperature, sizeof(coolantTemp)+1);
}



void batteryCallback(const fsae_electric_vehicle::drive_train::ConstPtr& msg, float& driveTrain) {
  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&driveTrain, &msg->voltage, sizeof(driveTrain)+1);
}



void gpsCallback(const fsae_electric_vehicle::gps::ConstPtr& msg, float& gpsHours, float& gpsMinutes, float& gpsSeconds,
                  float& gpsFix, float& gpsSpeed, float& gpsHeading, float& gpsLapEndTime, float& gpsCurrentLapTimer) {
  
  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&gpsHours, &msg->hours, sizeof(gpsHours)+1);
	memcpy(&gpsMinutes, &msg->minutes, sizeof(gpsMinutes)+1);
	memcpy(&gpsSeconds, &msg->seconds, sizeof(gpsSeconds)+1);
	memcpy(&gpsFix, &msg->fix, sizeof(gpsFix)+1);
  //memcpy(&gpsLat, &msg->latitude, sizeof(gpsLat)+1);
  //memcpy(&gpsLon, &msg->longitude, sizeof(gpsLon)+1);
  memcpy(&gpsSpeed, &msg->speed, sizeof(gpsSpeed)+1);
  memcpy(&gpsHeading, &msg->heading, sizeof(gpsHeading)+1);
  memcpy(&gpsLapEndTime, &msg->lapEndTime, sizeof(gpsLapEndTime)+1);
  memcpy(&gpsCurrentLapTimer, &msg->currentLapTimer, sizeof(gpsCurrentLapTimer)+1);
}



void suspensionCallback(const fsae_electric_vehicle::suspension::ConstPtr& msg,
                        float& suspensionFrontLeft, float& suspensionFrontRight, float& suspensionRearLeft, float& suspensionRearRight) {

  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&suspensionFrontLeft, &msg->frontLeft, sizeof(suspensionFrontLeft)+1);
	memcpy(&suspensionFrontRight, &msg->frontRight, sizeof(suspensionFrontRight)+1);
	memcpy(&suspensionRearLeft, &msg->rearLeft, sizeof(suspensionRearLeft)+1);
	memcpy(&suspensionRearRight, &msg->rearRight, sizeof(suspensionRearRight)+1);
}



void ResetValues(float& speedVal, float& driveTrain, float& coolantTemp, float& brakePressure,
                 float& suspensionFrontLeft, float& suspensionFrontRight, float& suspensionRearLeft, float& suspensionRearRight,
                 float& gpsHours, float& gpsMinutes, float& gpsSeconds, float& gpsFix, float& gpsSpeed, float& gpsHeading, float& gpsLapEndTime, float& gpsCurrentLapTimer) {

  speedVal = -1.0f, driveTrain = -1.0f, coolantTemp = -1.0f, brakePressure = -1.0f;
  suspensionFrontLeft = -1.0f, suspensionFrontRight = -1.0f, suspensionRearLeft = -1.0f, suspensionRearRight = -1.0f;
  gpsHours = -1.0f, gpsMinutes = -1.0f, gpsSeconds = -1.0f, gpsFix = -1.0f, gpsSpeed = -1.0f, gpsHeading = -1.0f, gpsLapEndTime = -1.0f, gpsCurrentLapTimer = -1.0f;
}



void logToFile(std::string text, float value, std::ofstream& collectedData) {
  if (text == "Speed") {
    collectedData << "\n" << text << ": " << value << std::endl; 
  } else {
    collectedData << text << ": " << value << std::endl;
  }
}