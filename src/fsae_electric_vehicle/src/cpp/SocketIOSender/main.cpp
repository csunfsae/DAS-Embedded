/*************************************************************************************************************************************************************
 * This is the networking code that sends the vehicle data to the server using SocketIO
 * 
 * Important Note: It seems like the max number of arguments you can pass into a Callback function is 9. This is most likely due to the boost::bind() function
 * being unable to bind that many arguments together, but im unsure if that is the exact reason. If passing more than 9 arguments to a Callback function is
 * necessary then making all of those arguments global variables would be a possible solution, or maybe you can put all of the variables into a struct.
 ************************************************************************************************************************************************************/


/* I want to pass an array by reference to gpsCallback. I need to make sure that 	memcpy(&gpsTimestamp + 1 works. */

#include <iostream>
#include <fstream>
#include "sioSender.h"
#include <future>
#include <list>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <string> 

#define OUTPUT_DATA_TO_TXT_FILE true  // Do you want to log all data received by this node to a file?
#define OUTPUT_DATA_TO_SERVER false   // Do you want to send all data received by this node to the server?

  float speedVal = -1.0f, brakePressure = -1.0f, coolantTemp = -1.0f, driveTrain = -1.0f;
  float suspensionFrontLeft = -1.0f, suspensionFrontRight = -1.0f, suspensionRearLeft = -1.0f, suspensionRearRight = -1.0f;
  float gpsTimestamp[4];
  float gpsHours = -1.0f, gpsMinutes = -1.0f, gpsSeconds = -1.0f, gpsMilliseconds = 1.0f, gpsFix = -1.0f, gpsLatitude = -1.0f, gpsLongitude = -1.0f, gpsSpeed = -1.0f, gpsHeading = -1.0f, gpsLapEndTime = -1.0f, gpsCurrentLapTimer = -1.0f;
void callSignalRConnection(std::string speed) {
    std::promise<void> start_task;
    signalr::hub_connection connection = signalr::hub_connection_builder::create("https://csun-fsae.azurewebsites.net/racehub").build();

    connection.on("Speedometer", [](const std::vector<signalr::value>& m)
        {
            std::cout << m[0].as_string() << std::endl;
        });

    connection.start([&start_task](std::exception_ptr exception) {
        start_task.set_value();
        });

    start_task.get_future().get();

    std::promise<void> send_task;

    std::vector<signalr::value> args{ speed };
    connection.invoke("Speedometer", args, [&send_task](const signalr::value& value, std::exception_ptr exception) {
        send_task.set_value();
        });

    send_task.get_future().get();

    std::promise<void> stop_task;
    connection.stop([&stop_task](std::exception_ptr exception) {
        stop_task.set_value();
        });

    stop_task.get_future().get();
}
int main(int argc, char **argv)
{
  //sio::message::list speedData(speedVal);
  //sio::message::list speedData(std::to_string(speedVal));

  ros::init(argc, argv, "SocketIOSender");
  ros::NodeHandle n;

  ros::Subscriber speed = n.subscribe<fsae_electric_vehicle::speedometer> ("speedometer", 1000, SpeedCallback);
  ros::Subscriber gps = n.subscribe<fsae_electric_vehicle::gps> ("gps_lap_timer", 1000, GpsCallback);
  ros::Subscriber bat = n.subscribe<fsae_electric_vehicle::drive_train> ("drivetrain_voltage", 1000, BatteryCallback);
  ros::Subscriber cool = n.subscribe<fsae_electric_vehicle::coolant> ("coolant_temperature", 1000, CoolantCallback);
  ros::Subscriber brake = n.subscribe<fsae_electric_vehicle::brake_pressure> ("brake_pressure", 1000, BrakeCallback);
  ros::Subscriber susp = n.subscribe<fsae_electric_vehicle::suspension>("suspension", 1000, SuspensionCallback);

  sio::client h;
  if (OUTPUT_DATA_TO_SERVER) {
    h.connect("http://64.227.48.74:2222");

    ROS_INFO("Connected to server!");
  }

  std::ofstream collectedData;

  if (OUTPUT_DATA_TO_TXT_FILE) {
    collectedData.open("collectedData.txt", std::ios::out);

    if (!collectedData.is_open()) {
      ROS_WARN("Could not open file for data logging");
    } else {
      ROS_INFO("File successfully opened!");
    }
  }

  ros::Rate loop_rate{40};

  while (ros::ok()) {
    ros::spinOnce();
    
    //std::ifstream ifs("carLocation.json");
    //json parseLocation = json::parse(ifs);

    // Check each float to see if its == -1. If it is -1 then send it to server, else dont send it.
    // if (speedVal != -1.0f) {
    //   if (OUTPUT_DATA_TO_SERVER)
    //     h.socket()->emit("speedometer", sio::double_message::create(speedVal));
    //   if (OUTPUT_DATA_TO_TXT_FILE)
    //     LogToFile("Speed", speedVal, collectedData);
    // }
    srand((unsigned)time(0));    

    for (int i = 0; i < 100; i++) {
        speedVal = 1 + (rand() % 50);
        

        callSignalRConnection(std::to_string(speedVal));
    }

    if (brakePressure != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("brakePressure", sio::double_message::create(brakePressure));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Speed", speedVal, collectedData);
    }

    //if (coolantTemp != -1.0f) {
    if(coolantTemp != -1.0f){
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("coolantTemp", sio::double_message::create(coolantTemp));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Coolant Temp", coolantTemp, collectedData);
    }

    if (driveTrain != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("driveTrain", sio::double_message::create(driveTrain));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Drive Train", driveTrain, collectedData);
    }

    if (suspensionFrontLeft != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("suspensionFrontLeft", sio::double_message::create(suspensionFrontLeft));  
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Suspension FL", suspensionFrontLeft, collectedData);
    }

    if (suspensionFrontRight != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("suspensionFrontRight", sio::double_message::create(suspensionFrontRight));  
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Suspension FR", suspensionFrontRight, collectedData);
    }

    if (suspensionRearLeft != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("suspensionRearLeft", sio::double_message::create(suspensionRearLeft));  
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("SuspensionRearLeft", suspensionRearLeft, collectedData);
    }

    if (suspensionRearRight != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("suspensionRearRight", sio::double_message::create(suspensionRearRight));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("SuspensionRearRight", suspensionRearRight, collectedData);
    }

    if (gpsHours != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("gpsHours", sio::double_message::create(gpsHours));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Hours", gpsHours, collectedData);
    }
    if (gpsMinutes != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("gpsMinutes", sio::double_message::create(gpsMinutes));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Minutes", gpsMinutes, collectedData);
    }

    if (gpsSeconds != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("gpsSeconds", sio::double_message::create(gpsSeconds));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Seconds", gpsSeconds, collectedData);
    }

    if (gpsFix != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("gpsFix", sio::double_message::create(gpsFix));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Fix", gpsFix, collectedData);
    }

    if (gpsSpeed != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("gpsSpeed", sio::double_message::create(gpsSpeed));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Speed ", gpsSpeed, collectedData);
    }

    if (gpsHeading != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("gpsHeading", sio::double_message::create(gpsHeading));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Heading", gpsHeading, collectedData);
    }

    if (gpsLapEndTime != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("gpsEndLapTime", sio::double_message::create(gpsLapEndTime));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Lap Time", gpsLapEndTime, collectedData);
    }

    if (gpsCurrentLapTimer != -1.0f) {
      if (OUTPUT_DATA_TO_SERVER)
        h.socket()->emit("gpsCurrentLapTimer", sio::double_message::create(gpsCurrentLapTimer));
      if (OUTPUT_DATA_TO_TXT_FILE)
        LogToFile("Current Lap Time", gpsCurrentLapTimer, collectedData);
    }

    sio::message::ptr object = createObject(carLocation);
    if (OUTPUT_DATA_TO_SERVER) {
      carLocation["latitude"] = gpsLatitude;
      carLocation["longitude"] = gpsLongitude;
      h.socket()->emit("carLocation", object);
    }
    if (OUTPUT_DATA_TO_TXT_FILE) {
      LogToFile("Car Latitude", gpsLatitude, collectedData);
      LogToFile("Car Longitude", gpsLongitude, collectedData);
    }

    // Set all floats to be -1 to indicate that they've been sent to the server.
    ResetValues(speedVal, driveTrain, coolantTemp, brakePressure,
                suspensionFrontLeft, suspensionFrontRight, suspensionRearLeft, suspensionRearRight, gpsLatitude, gpsLongitude,
                gpsHours, gpsMinutes, gpsSeconds, gpsFix, gpsSpeed, gpsHeading, gpsLapEndTime, gpsCurrentLapTimer);
    
    loop_rate.sleep();
  }

  collectedData.close();
}



void SpeedCallback(const fsae_electric_vehicle::speedometer::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock{dataMutex};
  memcpy(&speedVal, &msg->speed, sizeof(speedVal)+1);
}



void BrakeCallback(const fsae_electric_vehicle::brake_pressure::ConstPtr& msg) { 
  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&brakePressure, &msg->pressure, sizeof(brakePressure)+1);
}



void CoolantCallback(const fsae_electric_vehicle::coolant::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&coolantTemp, &msg->temperature, sizeof(coolantTemp)+1);
}



void BatteryCallback(const fsae_electric_vehicle::drive_train::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&driveTrain, &msg->voltage, sizeof(driveTrain)+1);
}



void GpsCallback(const fsae_electric_vehicle::gps::ConstPtr& msg) {
  
  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&gpsTimestamp, &msg->timestamp[0], sizeof(float)+1);
	memcpy(&gpsTimestamp + 1, &msg->timestamp[1], sizeof(float)+1);
	memcpy(&gpsTimestamp + 2, &msg->timestamp[2], sizeof(float)+1);
  memcpy(&gpsTimestamp + 3, &msg->timestamp[3], sizeof(float)+1); // Make a timestamp array so i dont have to pass in 4 separate values.
	memcpy(&gpsFix, &msg->fix, sizeof(gpsFix)+1);
  //memcpy(&gpsLatitude, &msg->latitude, sizeof(gpsLatitude)+1);
  //memcpy(&gpsLongitude, &msg->longitude, sizeof(gpsLongitude)+1);
  memcpy(&gpsSpeed, &msg->speed, sizeof(gpsSpeed)+1);
  memcpy(&gpsHeading, &msg->heading, sizeof(gpsHeading)+1);
  memcpy(&gpsLapEndTime, &msg->lapEndTime, sizeof(gpsLapEndTime)+1);
  memcpy(&gpsCurrentLapTimer, &msg->currentLapTimer, sizeof(gpsCurrentLapTimer)+1);
}



void SuspensionCallback(const fsae_electric_vehicle::suspension::ConstPtr& msg) {

  std::lock_guard<std::mutex> lock{dataMutex};
	memcpy(&suspensionFrontLeft, &msg->frontLeft, sizeof(suspensionFrontLeft)+1);
	memcpy(&suspensionFrontRight, &msg->frontRight, sizeof(suspensionFrontRight)+1);
	memcpy(&suspensionRearLeft, &msg->rearLeft, sizeof(suspensionRearLeft)+1);
	memcpy(&suspensionRearRight, &msg->rearRight, sizeof(suspensionRearRight)+1);
}



void ResetValues(float& speedVal, float& driveTrain, float& coolantTemp, float& brakePressure,
                 float& suspensionFrontLeft, float& suspensionFrontRight, float& suspensionRearLeft, float& suspensionRearRight, float& gpsLatitude, float& gpsLongitude,
                 float& gpsHours, float& gpsMinutes, float& gpsSeconds, float& gpsFix, float& gpsSpeed, float& gpsHeading, float& gpsLapEndTime, float& gpsCurrentLapTimer) {

  speedVal = -1.0f, driveTrain = -1.0f, coolantTemp = -1.0f, brakePressure = -1.0f;
  suspensionFrontLeft = -1.0f, suspensionFrontRight = -1.0f, suspensionRearLeft = -1.0f, suspensionRearRight = -1.0f;
  gpsHours = -1.0f, gpsMinutes = -1.0f, gpsSeconds = -1.0f, gpsFix = -1.0f, gpsSpeed = -1.0f, gpsHeading = -1.0f, gpsLapEndTime = -1.0f, gpsCurrentLapTimer = -1.0f;
}



void LogToFile(std::string text, float value, std::ofstream& collectedData) {
  if (text == "Speed") {
    collectedData << "\n" << text << ": " << value << std::endl; 
  } else {
    collectedData << text << ": " << value << std::endl;
  }
}

