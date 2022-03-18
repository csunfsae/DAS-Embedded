/******************************************************************************************************************************************
 * Author: Brandon Cobb.
 * E-mail: btc5472@gmail.com
 * 
 * This code only works in the PST time zone
 * Im not using the "valid" field within the CANBUS frame
 * 
 * This programs goal is to read the CANBUS frames sent from the Adafruit GPS units and use it to calculate the number of laps, lap times,
 * & best lap time. That requires this program to know the cars ground location, the ground location of the start/finish line. This program
 * will know when the car has crossed the start/finish line by drawing a line between its current GPS position and its previous GPS
 * location and then checking to see if that line intersects with the start/finish line.
 * getData() runs in less than 0.00006 seconds
 * 
 * I was trying to pass gpsLatitude and gpsLongitude into the gpsCallback function but boost::bind wont take more than 9 params. So to solve
 * that i was thinking of removing Hours, minutes, & seconds from the data field and merging that into one value somehow. I should merge
 * thsoe values by puttin them into an array.
 * 
 * What Ive done since last commit:
 * 
 * 
 * 
*******************************************************************************************************************************************/

/* TIMING CODE
auto startTime = std::chrono::high_resolution_clock::now();
auto endTime = std::chrono::high_resolution_clock::now();
std::chrono::duration<float> duration = endTime - startTime;
std::cout << "Duration: " << duration.count() << std::endl;
*/

#include <cassert>
#include <math.h>
#include <array>
//#include <thread>
#include <chrono>
#include "RaceTrack.h"
//#include "Lap.h"

#ifdef _WIN32	// If in a Windows environment
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define _CRT_SECURE_NO_WARNINGS // For MSVC
#define LOW_SPEED_DATA_LOGGING 	true

float truncate(float coordinate, int start, int numSplit);
static void FillRosMessageWithProcessedData(fsae_electric_vehicle::gps*, RaceTrack&);
static void FillRosMessageWithFrameOneData(fsae_electric_vehicle::gps*, std::optional<CANData>);
static void FillRosMessageWithFrameTwoData(fsae_electric_vehicle::gps*, point, bool&);
static point InterpretLatLon(std::optional<CANData>);
static float ConvertToSeconds(char*);
static void DisplayTime(const uint8_t, const float);
static void WaitForGPSFix(CANController*,
							std::pair<std::optional<CANData>, std::optional<CANData>>*,
							std::pair<std::optional<CANData>, std::optional<CANData>>*);
static validFrame GetGPSData(CANController*,
								std::pair<std::optional<CANData>, std::optional<CANData>>*,
								std::pair<std::optional<CANData>, std::optional<CANData>>*);
static void WaitForCrossingStartLine(CANController&, RaceTrack&,
										std::pair<std::optional<CANData>, std::optional<CANData>>&,
										std::pair<std::optional<CANData>, std::optional<CANData>>&);
static void EstablishStartLineSpeedMethod(CANController&, RaceTrack&,
											std::pair<std::optional<CANData>, std::optional<CANData>>&,
											std::pair<std::optional<CANData>, std::optional<CANData>>&);


int main(int argc, char **argv)
{
	std::cout << "In cout gps_lap_timer main function\n" << std::endl;
	ROS_INFO_STREAM("In gps_lap_timer main function\n");

	// Stuff to make this a ROS node
	ros::init(argc, argv, "gps_lap_timer");
	ros::NodeHandle n;
  	ros::Publisher gps_lap_timer_pub = n.advertise<fsae_electric_vehicle::gps>("gps_lap_timer", 1000);
  	fsae_electric_vehicle::gps gps_lap_timer; // Struct used for sending GPS data to the sioSender.cpp ROS node

    // Start the CABUS header on the Jetson/Quasar board
	CANController can;
	if (can.start("can0") == -1) {
		ROS_FATAL("Could not start the CANBUS header!\n");
		exit(-1);
	}

	// 2 CANBUS frames will need to be received to get all the data from one GPRMC sentence
    // Frame 1 (.first) includes hour, minute, seconds, fix, speed, and angle
    // Frame 2 (.second) includes Latitude and Longitude
	// One pair contains two CANBUS frames of GPS data from one GPRMC sentence. Each GPS unit emits one GPRMC sentence at a fixed frequency
	// Store frame with lower id on the left (first) the one with higher id that contains lat & lon data on on the right (second)
	std::pair<std::optional<CANData>, std::optional<CANData>> gpsUnitOneData; 
	std::pair<std::optional<CANData>, std::optional<CANData>> gpsUnitTwoData;

	RaceTrack RaceTrack;

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();



	ros::Rate loop_rate(40); // This means that loop rate can be up to 40 times per second
	validFrame updatedFrames = noFrames;
	auto startLowSpeedTimer = std::chrono::high_resolution_clock::now();
	auto currentTime = std::chrono::high_resolution_clock::now();
	std::chrono::duration<float> lowSpeedDuration = currentTime - startLowSpeedTimer;
	while (ros::ok()) {
		ROS_DEBUG_ONCE("ROS is ok! Entered the main while loop");

		// Read CANBUS frames from the CANBUS header
		updatedFrames = GetGPSData(&can, &gpsUnitOneData, &gpsUnitTwoData);

		// Print out frame
		std::cout << "UnitOneData:" << std::endl;
		std::cout << "VALID:" << gpsUnitOneData.first->valid << std::endl;
		// printf("Hours: %u \n", gpsUnitOneData.first->data[GPS_HOURS]);
		// printf("Mins: %u \n", gpsUnitOneData.first->data[GPS_MINUTES]);
		// printf("Secs: %u \n", gpsUnitOneData.first->data[GPS_SECONDS]);
		// ROS_INFO("Hours: %u \n", gpsUnitOneData.first->data[GPS_HOURS]);
		// ROS_INFO("Mins: %u \n", gpsUnitOneData.first->data[GPS_MINUTES]);
		// ROS_INFO("Secs: %u \n", gpsUnitOneData.first->data[GPS_SECONDS]);
		// printf("FIX: %u \n", gpsUnitOneData.first->data[GPS_FIX]);
		// ROS_INFO("FIX: %u \n", gpsUnitOneData.first->data[GPS_FIX]);
		// ROS_INFO("ID: %u \n", gpsUnitOneData.first->data[GPS_ONE_FRAME_ONE_ID]);
		// printf("Speed: %u \n", gpsUnitOneData.first->data[GPS_SPEED]);
		ROS_INFO("Speed: %u \n", gpsUnitOneData.first->data[GPS_SPEED]);
		// Car will stop logging data after low speed for 5 seconds
		// If lowSpeedTimer > 10s, then wait to cross start line and start timer once crossed
		if (!LOW_SPEED_DATA_LOGGING) { 																					// If Low Speed Data Logging is turned off
			ROS_INFO_ONCE("Low Speed Data Logging is off. Nothing will be sent to server if car is moving very slow.");
			if (RaceTrack.GetStartLine().p0.x != 0.0f && RaceTrack.GetStartLine().p0.y != 0.0f && 							// If startLine exists
				RaceTrack.GetStartLine().p1.x != 0.0f && RaceTrack.GetStartLine().p1.y != 0.0f) {
					
				currentTime = std::chrono::high_resolution_clock::now();
				lowSpeedDuration = currentTime - startLowSpeedTimer;														// Update the duration

				if (lowSpeedDuration.count() > 5.0f) {																		// If duration > 5s
					startLowSpeedTimer = std::chrono::high_resolution_clock::now();												// Restart timer
					WaitForCrossingStartLine(can, RaceTrack, gpsUnitOneData, gpsUnitTwoData);									// Wait to cross startLine
		
				} else {																									// Else if duration < 5
					// If speed > 2 then reset timer. If speed < 2 then update duration
					if (gpsUnitOneData.first->data[GPS_SPEED] > 2) {															// If speed > 2, reset timer and duration
						startLowSpeedTimer = std::chrono::high_resolution_clock::now();
						lowSpeedDuration = currentTime - startLowSpeedTimer;
					} else if (gpsUnitOneData.first->data[GPS_SPEED] < 2)														// Else if speed < 2, update duration
						lowSpeedDuration = currentTime - startLowSpeedTimer;
				}
			}
		}
		
		//RaceTrack.EstablishStartLine(gpsUnitOneData);

		// If GPS doesnt have fix then wait for fix
		// If startLine hasnt been created then create one with speed method
		switch (updatedFrames) {
			case noFrames: // Do nothing
				break;

			case unitOneFrameOne:
				if (!gpsUnitOneData.first->data[GPS_FIX])
					WaitForGPSFix(&can, &gpsUnitOneData, &gpsUnitTwoData);
				break;

			case unitOneFrameOneAndTwo:
				if (!gpsUnitOneData.first->data[GPS_FIX])
					WaitForGPSFix(&can, &gpsUnitOneData, &gpsUnitTwoData);

				if ((RaceTrack.GetStartLine()).p0.x == 0.0f && (RaceTrack.GetStartLine()).p0.y == 0.0f &&
					(RaceTrack.GetStartLine()).p1.x == 0.0f && (RaceTrack.GetStartLine()).p1.y == 0.0f) {
					RaceTrack.EstablishStartLine(gpsUnitOneData);
					//EstablishStartLineSpeedMethod(can, RaceTrack, gpsUnitOneData, gpsUnitTwoData);
				}
				break;

			case unitTwoFrameOne:
				if (!gpsUnitTwoData.first->data[GPS_FIX])
					WaitForGPSFix(&can, &gpsUnitOneData, &gpsUnitTwoData);
				break;

			case unitTwoFrameOneAndTwo:
				if (!gpsUnitTwoData.first->data[GPS_FIX])
					WaitForGPSFix(&can, &gpsUnitOneData, &gpsUnitTwoData);

				if (RaceTrack.GetStartLine().p0.x == 0.0f && RaceTrack.GetStartLine().p0.y == 0.0f &&
					RaceTrack.GetStartLine().p1.x == 0.0f && RaceTrack.GetStartLine().p1.y == 0.0f) {
					EstablishStartLineSpeedMethod(can, RaceTrack, gpsUnitOneData, gpsUnitTwoData);
				}
				break;
		}

		// Store data in the ROS Message depending on which frame is recieved
		// Its ok if we have the first frame but not the second because we can verify that we still have fix.
		// Its not ok if we have the 2nd frame and not the first because we cannot verify that we still have fix.
		switch (updatedFrames) {
			case noFrames: // Do nothing
				break;

			case unitOneFrameOne:
				if (gpsUnitOneData.first->data[GPS_FIX])
					FillRosMessageWithFrameOneData(&gps_lap_timer, gpsUnitOneData.first);
				break;

			case unitOneFrameOneAndTwo:
				if (gpsUnitOneData.first->data[GPS_FIX]) {
					point carCoordinates = InterpretLatLon(gpsUnitTwoData.second);
					RaceTrack.UpdateCarCoordinates(carCoordinates);
					FillRosMessageWithFrameOneData(&gps_lap_timer, gpsUnitOneData.first);
					FillRosMessageWithFrameTwoData(&gps_lap_timer, carCoordinates, gpsUnitTwoData.second->valid); // Might be able to combine all FillRosMessage() funcs together
					// Sends current lap time & lap end time. Data points that can be derived by the server: lap num, best lap, best lap time
					FillRosMessageWithProcessedData(&gps_lap_timer, RaceTrack);
				}
				break;

			case unitTwoFrameOne:
				if(gpsUnitTwoData.first->data[GPS_FIX])
					FillRosMessageWithFrameOneData(&gps_lap_timer, gpsUnitTwoData.first);
				break;

			case unitTwoFrameOneAndTwo:
				if (gpsUnitTwoData.first->data[GPS_FIX]) {
					point carCoordinates = InterpretLatLon(gpsUnitTwoData.second);
					RaceTrack.UpdateCarCoordinates(carCoordinates);
					FillRosMessageWithFrameOneData(&gps_lap_timer, gpsUnitOneData.first);
					FillRosMessageWithFrameTwoData(&gps_lap_timer, carCoordinates, gpsUnitTwoData.second->valid);
					// Sends current lap time & lap end time. Data points that can be derived by the server: lap num, best lap, best lap time
					FillRosMessageWithProcessedData(&gps_lap_timer, RaceTrack);
				}
				break;
		}

		// Send ROS message filled with GPS data to sioSender.cpp
		if (updatedFrames != noFrames) 
			gps_lap_timer_pub.publish(gps_lap_timer);

		//ros::spinOnce();
   		loop_rate.sleep();
  	}
}  

// Responsible for truncating Latitude and Longitude values from DDM to DD format
float truncate(float coordinate, int start, int numSplit){
	std::string output = std::to_string(coordinate).substr(start, numSplit+1);
	
	if(output.find('.') == std::string::npos || output.back() == '.'){
		output.pop_back();
	}
	
	return stof(output);


}

// Read CANBUS frames from the CANBUS header and store them in the gpsUnitOneData or gpsUnitTwoData pairs. Return true means one pair is full
static validFrame GetGPSData(CANController* can,
						std::pair<std::optional<CANData>, std::optional<CANData>>* gpsUnitOneData,
						std::pair<std::optional<CANData>, std::optional<CANData>>* gpsUnitTwoData)
	{
	// Look for frame one (id = 0x35) from GPS unit one
	std::optional<CANData> canData = can->getData(GPS_ONE_FRAME_ONE_ID, 0x1FFFFFFF);
	// if(canData.has_value()){
	// 	CANData tempcanData = canData;
	// 	for(int i= 0; i<8;i++){
	// 		ROS_INFO("canData: %u\n", tempcanData.data[i]);
	// 	}
	// }
	//for(int i= 0; i<8;i++){
			//ROS_INFO("canData: %u\n", canData.data[i]);
	//}
	// 
	if (canData.has_value() && canData->valid) { 	// If frame has been read
		gpsUnitOneData->first = canData;			// Store frame in gpsUnitOneData
		ROS_INFO("GPS1 F 1 read");
		canData.reset();							// Clear the data from canData

		std::optional<CANData> canData = can->getData(GPS_ONE_FRAME_TWO_ID, 0x1FFFFFFF); // Look for frame two from GPS unit one

		if (canData.has_value()) {					// If frame has been read
			gpsUnitOneData->second = canData;		// Store frame in gpsUnitTwoData
			ROS_DEBUG("GPS1 F 2 read");
			return unitOneFrameOneAndTwo;
		}

		ROS_WARN("First CANBUS frame for GPS 1 was read, but not the second frame containing latitude & longitude.");

		return unitOneFrameOne;
	}

	// Look for frame one (id = 0x37) from GPS unit two
	std::optional<CANData> canData2 = can->getData(GPS_TWO_FRAME_ONE_ID, 0x1FFFFFFF);
	//
	if (canData.has_value() && canData->valid) {	// If frame has been read
		gpsUnitTwoData->first = canData2;			// Store frame in gpsUnitTwoData
		ROS_DEBUG("GPS2 F 1 read");

		canData.reset();							// Clear the data from canData

		std::optional<CANData> canData2 = can->getData(GPS_TWO_FRAME_TWO_ID, 0x1FFFFFFF); // Look for frame two from GPS unit two

		if (canData.has_value()) {					// If frame has been read
			gpsUnitTwoData->second = canData2;		// Store frame in gpsUnitTwoData
			ROS_DEBUG("GPS2 F 2 read");
			return unitTwoFrameOneAndTwo;
		}

		ROS_WARN("First CANBUS frame from GPS was read, but not the second frame containing latitude & longitude.");

		return unitTwoFrameOne;
	}

	return noFrames;
}



// Doesnt return until a GPS fix is found
static void WaitForGPSFix(CANController* can,
						std::pair<std::optional<CANData>, std::optional<CANData>>* gpsUnitOneData,
						std::pair<std::optional<CANData>, std::optional<CANData>>* gpsUnitTwoData)
	{
	do {
		std::optional<CANData> canData = can->getData(GPS_ONE_FRAME_ONE_ID, 0x1FFFFFFF); // 2nd param is ID mask
		if (canData->data[GPS_FIX] == 1) //canData.has_value() && 
			break; // No need to copy frames into gpsUnitOneData

		std::optional<CANData> canData2 = can->getData(GPS_TWO_FRAME_ONE_ID, 0x1FFFFFFF);
		if (canData2->data[GPS_FIX] == 1) //canData2.has_value() && 
			break; // No need to copy frames into gpsUnitTwoData

		ROS_INFO_THROTTLE(3, "Waiting for GPS fix!\n");
		ROS_WARN_DELAYED_THROTTLE(120, "Taking a long time to find a GPS fix.");

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	} while (true);

	gpsUnitOneData->first->valid = 0;
	gpsUnitTwoData->first->valid = 0;

	ROS_INFO("Found FIX!");
}



static void EstablishStartLineSpeedMethod(CANController& can, RaceTrack& RaceTrack,
											std::pair<std::optional<CANData>, std::optional<CANData>>& gpsUnitOneData,
											std::pair<std::optional<CANData>, std::optional<CANData>>& gpsUnitTwoData) 
	{
	validFrame updatedFrames = noFrames;
	std::chrono::_V2::steady_clock::time_point startOfTimer = std::chrono::steady_clock::now();
	std::chrono::_V2::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
	std::chrono::duration<float> durationOfTimer = currentTime - startOfTimer;

	ROS_INFO("Waiting to establish a start line using the speed method.");
	
	// Wait to Establish Startline until the car travels over 20mph for 20 seconds
	while (durationOfTimer.count() < 2.0f) {
		updatedFrames = GetGPSData(&can, &gpsUnitOneData, &gpsUnitTwoData);

		if (updatedFrames >= unitTwoFrameOne && gpsUnitTwoData.first->data[GPS_SPEED] < 5) {
			startOfTimer = std::chrono::steady_clock::now();
		} else if (updatedFrames >= unitOneFrameOne && gpsUnitOneData.first->data[GPS_SPEED] < 5)
			startOfTimer = std::chrono::steady_clock::now();

		currentTime = std::chrono::steady_clock::now();
		durationOfTimer = currentTime - startOfTimer;

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	ROS_INFO("Car has traveled over 5mph for 2 seconds. Establishing start line now");

	// Wait for both frames from a single GPS unit to be received
	do {
		updatedFrames = GetGPSData(&can, &gpsUnitOneData, &gpsUnitTwoData);

		ROS_WARN_DELAYED_THROTTLE(15, "Trying to read two CANBUS packets with GPS data... Taking longer than expected.");

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	} while (updatedFrames != unitOneFrameOneAndTwo && updatedFrames != unitTwoFrameOneAndTwo);

	ROS_DEBUG("At least one pair is full");

	// Establish startline using just one of the pairs
	if (updatedFrames == unitOneFrameOneAndTwo) {
		//std::cout << "In If" << std::endl;
		RaceTrack.EstablishStartLine(gpsUnitOneData);
		RaceTrack.StartTimer();
		// These lines below are commented out because ther is no startLine object/field in ROS message
		//gps_lap_timer.startLine = RaceTrack.getStartLilne();	// Put startLine in ROS message
		//gps_lap_timer_pub.publish(gps_lap_timer);
	}
	else if (updatedFrames = unitTwoFrameOneAndTwo) {
		//std::cout << "In If 2" << std::endl;
		RaceTrack.EstablishStartLine(gpsUnitTwoData);
		RaceTrack.StartTimer();
		//gps_lap_timer.startLine = RaceTrack.getStartLilne();	// Put startLine in ROS message
		//gps_lap_timer_pub.publish(gps_lap_timer);
	}
	else
		ROS_ERROR("Cannot Establish a Start Line due to at least 1 out of 2 CANBUS frames missing!");
}



static point InterpretLatLon(std::optional<CANData> latLonFrame) {
	int32_t latitude;	// latitude format: DDMM.mmmm		This represents Degrees and Minutes. No seconds
	int32_t longitude;	// longitude format: DDDMM.mmmm
	int latDegrees, lonDegrees;
	float latMin, lonMin;
	point carCoordinates;

	// Pull the latitude & longitude data from the frame	
	for (int i = 0; i < 4; i++) {
		latitude = latitude << 8 | latLonFrame->data[i];
	}
	for (int i = 0; i < 4; i++) {
		longitude = longitude << 8 | latLonFrame->data[i];
	}

	// Interpret the lat & lon data 
	float flatitude = static_cast<float>(latitude / 10000);
	float flongitude = static_cast<float>(longitude / 10000);
	
	latDegrees = truncate(flatitude, 0, 2);
	latMin = truncate(flatitude, 2, 6);
	
	lonDegrees = truncate(flongitude, 0, 3);
	lonMin = truncate(flongitude, 3, 6);
	
	flatitude = latDegrees + (latMin / 60);
	flongitude = lonDegrees + (lonMin / 60);

	carCoordinates.x = flatitude;
	carCoordinates.y = flongitude;

	return carCoordinates;
}



// Partially fills the struct representing a ROS message with data from first GPS frame
static void FillRosMessageWithFrameOneData(fsae_electric_vehicle::gps* gps_lap_timer, std::optional<CANData> frameOneData) { // This just needs an optional candata param not a pair
	uint16_t heading;
	
	gps_lap_timer->timestamp[0] = frameOneData->timestamp;		// Time data (HOURS portion of timestamp) Convert to float for all timestamp fields
	gps_lap_timer->timestamp[1] = frameOneData->timestamp;		// Time data ( Minutes portion of timestamp)
	gps_lap_timer->timestamp[2] = frameOneData->timestamp;		// Time data
	gps_lap_timer->timestamp[3] = frameOneData->timestamp;		// Time data
	gps_lap_timer->fix = frameOneData->data[GPS_FIX];
	gps_lap_timer->speed = frameOneData->data[GPS_SPEED];

 	heading = frameOneData->data[GPS_HEADING_1];
	heading = heading << 8;
	gps_lap_timer->heading = heading | frameOneData->data[GPS_HEADING_2];

	frameOneData->valid = false; // Mark frame as invalid
}



// Partially fills the struct representing a ROS message with data from second GPS frame
static void FillRosMessageWithFrameTwoData(fsae_electric_vehicle::gps* gps_lap_timer, point coordinates, bool& frameValid) {
	// Put lat & lon into the struct representing a ROS message
	gps_lap_timer->latitude = coordinates.x;
	gps_lap_timer->longitude = coordinates.y;

	// Mark frame as invalid
	frameValid = false;
}



// Partially fills a struct representing a ROS message with lapDuration and currentLapTimer
static void FillRosMessageWithProcessedData(fsae_electric_vehicle::gps* gps_lap_timer, RaceTrack& RaceTrack) {
	auto lapDuration = RaceTrack.GetLapDuration();
	float fLapDuration = std::chrono::duration<float>(lapDuration).count();
	if (fLapDuration != 0.0f) {
		gps_lap_timer->lapEndTime =fLapDuration;
	}

	// Get currentLapTimer and send it
	auto currentLapTimer = RaceTrack.GetCurrentLapTimer();
	gps_lap_timer->currentLapTimer = std::chrono::duration<float>(currentLapTimer).count();
}



static void WaitForCrossingStartLine(CANController& can, RaceTrack& RaceTrack,
									std::pair<std::optional<CANData>, std::optional<CANData>>& gpsUnitOneData,
									std::pair<std::optional<CANData>, std::optional<CANData>>& gpsUnitTwoData)
{
	validFrame updatedFrames;
	bool crossedStartLine = false;
	point carCoordinates;

	ROS_INFO("Waiting until the car crosses the startLine again to start the lap timer.");

	while (!crossedStartLine) {
		updatedFrames = GetGPSData(&can, &gpsUnitOneData, &gpsUnitTwoData);

		if (updatedFrames == unitOneFrameOneAndTwo && gpsUnitOneData.first->data[GPS_FIX]) {
			carCoordinates = InterpretLatLon(gpsUnitTwoData.second);
			RaceTrack.UpdateCarCoordinates(carCoordinates);
			crossedStartLine = RaceTrack.WasStartLineCrossed();
			ROS_INFO("StartLine has now been crossed, starting lap timer.");
		} else if (updatedFrames == unitTwoFrameOneAndTwo && gpsUnitTwoData.first->data[GPS_FIX]) {
			carCoordinates = InterpretLatLon(gpsUnitTwoData.second);
			RaceTrack.UpdateCarCoordinates(carCoordinates);
			crossedStartLine = RaceTrack.WasStartLineCrossed();
			ROS_INFO("StartLine has now been crossed, starting lap timer.");
		} else {
			ROS_WARN("While waiting for car to cross startLine, the 2nd CANBUS frame containing lat & lon couldnt be read");
		}
	}
}


/************************************************************ TODO **********************************************************************/

/* This program needs to read all CANBUS frames that pertain to the GPS units. That means reading ALL CANBUS frames that it sees
and then filtering out the ones that it wants based on the frame ID.
can.getData() reads frames from an std::vector<> called m_dataMap which is defined in CANController.h. That is a vector of CANData
objects. Each object represents a CANBUS frame with a specific ID. When a new CANBUS frame is read from the header, that frame
will replace the frame/object if it already exists in the std::vector<> m_dataMap. So there will be no two elements within m_dataMap
that contain a CANBUS frame with the same ID, or in other words old CANBUS frames are immediately overwritten in favor of new frames.
Also whenever CANController::getData() is called, if the sought after frame is found then it is returned and marked as invalid within
std::vector<> m_dataMap. So if that same frame is searched for again, it will not be returned because it is marked as invalid.
Can the this function call "can.getData(0x37, 0x1FFFFFFF);" be used to filter out several message
IDs at once or do I have to call that function separately for each can frame i want to filter out? */

// Vehicle data should be stored locally on the TX2 if there is no connection to the server

/* There may be an issue where the frames stored in gpsUnitOneData pair may be from different GPRMC sentences. For example the 
 first frame may be read & stored in the gpsUnitOneData pair successfully but then for whatever reason the second frame may not
 be read or stored successfully. At that point the gpsUnitOneData will only have the first frame stored and the second member of
 the pair will be empty. The data from the 2nd GPS unit will then try to be read and stored successfully. Then we will expect to 
 receive and store frame one again from the first GPS unit, but if that doesnt happen successfully for whatever reason then it
 would be possible to read and store the 2nd frame from GPS unit one and then at that point we would have gpsUnitOneData.first be
 from an old GPRMC sentence that is invalid now. */

/* Does the gps give all of its sentences the same timestamp? I want to know if all 4 of the CANBUS frames ill be sending to the TX2 in
one iteration of the while loop in the teensy code will all have the same timestamp. */

// GPS can lose fix at any time. Make sure that fact doesnt cause bugs in the code

// Make as many function arguments as possible const

// Pass arguments by reference and not by pointer when possible in all functions

/* Other stuff to send to sioSender.cpp:
	start line
	maybe startHeading
*/

// Can the private functions in RaceTrack object be defined as const? LineIntersect() const {}

/*
A more accurate startLine crossing time can be obtained by adding the timestamp of the previous car position (carCoordinates.p1) and the 
timestamp of the current car position (carCoordinates.p0) and dividing by two. Or better yet, find the intersection point of the startLine
and the line created between the two points in the carCoordinates variable, and use that intersection point to estimate when the car crossed
the startLine.
*/

/* If no start line is defined then the car should just record all the data as one lap. Then later the user can go into the system and define
a startline and the DAS should figure out how many laps were done.*/

/* Maybe instead of having GetGpsData() return a bool, I can put a timestamp in the timestamp field in the CANBUS frame and then just compare
the timestamps to make sure they arent more than  */


/******************************************************* Unused code *************************************************************

/*************************************************** GPS Utility Functions *********************************************

static float ConvertToSeconds(uint64_t time) {
	if (time == nullptr)
		return 0.0f;

	float timeHours = atof_(time);
	
	uint16_t hm = (uint16_t)(timeHours / 100);
	uint16_t hours = (uint16_t)(ft / 10000);
	uint16_t minutes = (hm - (hours * 100) + (hours * 60));
	float seconds = ft - (hm * 100) + (minutes * 60);

	return seconds;
}



// Convert float seconds to MM::SS.SS format.
static void DisplayTime(const uint8_t n, const float ft) {
	assert(ft > 0.);

	char s1[16];

	memset(&s1[10], 0, 6);
	uint16_t m = (uint16_t)ft / 60;
	float fs = ft - (m * 60);
	sprintf(s1, "%.02d:%05.2f ", m, fs);

	if (n) { // Prepend lap number.
		char s2[6];
		sprintf(s2, "%d: ", n);
		Prepend(s1, s2);
	}

	std::cout << s1;
}*/
