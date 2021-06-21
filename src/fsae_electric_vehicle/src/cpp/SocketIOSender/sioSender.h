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
//#include <socket.io-client-cpp/src/sio_client.h>
//#include "fsae_electric_vehicle/sio_client.h"

using namespace nlohmann;

std::mutex dataMutex;

json carLocation;

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

sio::message::ptr createObject(json );
void SuspensionCallback(const fsae_electric_vehicle::suspension::ConstPtr&, float&, float&, float&, float&);
void BatteryCallback(const fsae_electric_vehicle::drive_train::ConstPtr&, float&);
void CoolantCallback(const fsae_electric_vehicle::coolant::ConstPtr&, float&);
void BrakeCallback(const fsae_electric_vehicle::brake_pressure::ConstPtr&, float&);
void GpsCallback(const fsae_electric_vehicle::gps::ConstPtr&, float (&gpsTimestamp)[4], float&, float&, float&, float&, float&, float&, float&);
void SpeedCallback(const fsae_electric_vehicle::speedometer::ConstPtr&, float&);
void ResetValues(float&, float&, float&, float&, float&, float&, float&, float&, float&, float&, float&, float&, float&, float&, float&, float&, float&, float&);
void LogToFile(std::string, float, std::ofstream&);