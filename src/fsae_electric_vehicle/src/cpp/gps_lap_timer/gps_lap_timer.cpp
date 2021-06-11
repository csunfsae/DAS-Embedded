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
 * I need to auto generate the startLine when car drives over 15mph for 10 seconds.
 * 
 * 
 * What Ive done since last commit:
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
#include <thread>
#include <chrono>
#include "RaceTrack.h"
#include "Lap.h"

#ifdef _WIN32	// If in a Windows environment
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define _CRT_SECURE_NO_WARNINGS // For MSVC

static void FillRosMessageWithProcessedData(fsae_electric_vehicle::gps*, RaceTrack&);
static void FillRosMessageWithFrameOneData(fsae_electric_vehicle::gps*, std::optional<CANData>);
static void FillRosMessageWithFrameTwoData(fsae_electric_vehicle::gps*, point, bool&);
static point InterpretLatLon(std::optional<CANData>);
static void Run(float, char *[]);
static float ConvertToSeconds(char*);
static void DisplayTime(const uint8_t, const float);
static void waitForGPSFix(CANController*,
							std::pair<std::optional<CANData>, std::optional<CANData>>*,
							std::pair<std::optional<CANData>, std::optional<CANData>>*);
static validFrame GetGPSData(CANController*,
							std::pair<std::optional<CANData>, std::optional<CANData>>*,
							std::pair<std::optional<CANData>, std::optional<CANData>>*);



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

	// 2 CANBUS frames will need to be received in order to get all the data from one GPRMC sentence
    // Frame 1 includes hour, minute, seconds, fix, speed, and angle
    // Frame 2 includes Latitude and Longitude
	// One pair contains two CANBUS frames of GPS data from one GPRMC sentence. Each GPS unit emits one GPRMC sentence at a fixed frequency
	// Store frame with lower id on the left (first) the one with higher id that contains lat & lon data on on the right (second)
	std::pair<std::optional<CANData>, std::optional<CANData>> gpsUnitOneData; 
	std::pair<std::optional<CANData>, std::optional<CANData>> gpsUnitTwoData;

	RaceTrack RaceTrack;

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
		ros::console::notifyLoggerLevelsChanged();

	waitForGPSFix(&can, &gpsUnitOneData, &gpsUnitTwoData);

	// If button is pressed to make start line. This is set to true because there is no button to make the startline yet
	if (true) {
		// Fill up one GPSUpdateCycle struct with data from one gps cycle. Not data split from 2 cycles of the GPS while loop on Teensy
		// While either pair is not full
		ROS_INFO("Establishing the Start Line");

		validFrame updatedFrames = noFrames;
		do {
			updatedFrames = GetGPSData(&can, &gpsUnitOneData, &gpsUnitTwoData);

			ROS_WARN_DELAYED_THROTTLE(15, "Trying to read two CANBUS packets with GPS data... Taking longer than expected.");

			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		} while (updatedFrames != unitOneFrameOneAndTwo && updatedFrames != unitTwoFrameOneAndTwo);
		//!(updatedFrames != unitOneFrameOneAndTwo) || !(updatedFrames == unitTwoFrameOneAndTwo)

		ROS_INFO("At least one pair is full");

		// Print out frame
		std::cout << "UnitOneData:" << std::endl;
		std::cout << "VALID:" << gpsUnitOneData.first->valid << std::endl;
		printf("Hours: %u \n", gpsUnitOneData.first->data[GPS_HOURS]);
		printf("Mins: %u \n", gpsUnitOneData.first->data[GPS_MINUTES]);
		printf("Secs: %u \n", gpsUnitOneData.first->data[GPS_SECONDS]);
		printf("FIX: %u \n", gpsUnitOneData.first->data[GPS_FIX]);
		printf("Speed: %u \n", gpsUnitOneData.first->data[GPS_SPEED]);

		// Establish startline using just one of the pairs
		if (updatedFrames == unitOneFrameOneAndTwo) {
			//std::cout << "In If" << std::endl;
			RaceTrack.EstablishStartLine(gpsUnitOneData);
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

		//std::cout << "AFTER If" << std::endl;
	}

	auto steadyBool = std::chrono::high_resolution_clock::is_steady;
	if (steadyBool) {
		ROS_INFO("Its a steady clock\n");
	} else {
		ROS_INFO("Its a non-Steady clock");
	}

	ros::Rate loop_rate(40); // This means that loop rate can be up to 30 times per second
  	
	validFrame updatedFrames = noFrames;
	while (ros::ok()) {
		ROS_INFO_ONCE("ROS is ok! Entered the main while loop");

		// Read CANBUS frames from the CANBUS header
		updatedFrames = GetGPSData(&can, &gpsUnitOneData, &gpsUnitTwoData);

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

		ros::spinOnce();
   		loop_rate.sleep();
  	}
}



// Partially fills a struct representing a ROS message with currentLapTimer and lapEndTime
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



// Doesnt return until a GPS fix is found
static void waitForGPSFix(CANController* can,
						std::pair<std::optional<CANData>, std::optional<CANData>>* gpsUnitOneData,
						std::pair<std::optional<CANData>, std::optional<CANData>>* gpsUnitTwoData)
	{
	do {
		std::optional<CANData> canData = can->getData(GPS_ONE_FRAME_ONE_ID, 0x1FFFFFFF); // 2nd param is ID mask
		if (canData.has_value() && canData->data[GPS_FIX] == 1)
			break; // No need to copy frames into gpsUnitOneData

		std::optional<CANData> canData2 = can->getData(GPS_TWO_FRAME_ONE_ID, 0x1FFFFFFF);
		if (canData2.has_value() && canData2->data[GPS_FIX] == 1)
			break; // No need to copy frames into gpsUnitTwoData

		ROS_INFO_THROTTLE(3, "Waiting for GPS fix!\n");
		ROS_WARN_DELAYED_THROTTLE(120, "Taking a long time to find a GPS fix.");

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	} while (true);

	gpsUnitOneData->first->valid = 0;
	gpsUnitTwoData->first->valid = 0;

	ROS_INFO("Found FIX!");
}



// Read CANBUS frames from the CANBUS header and store them in the gpsUnitOneData or gpsUnitTwoData pairs. Return true means one pair is full
static validFrame GetGPSData(CANController* can,
						std::pair<std::optional<CANData>, std::optional<CANData>>* gpsUnitOneData,
						std::pair<std::optional<CANData>, std::optional<CANData>>* gpsUnitTwoData)
	{
	// Look for frame one (id = 0x35) from GPS unit one
	std::optional<CANData> canData = can->getData(GPS_ONE_FRAME_ONE_ID, 0x1FFFFFFF);
	
	if (canData.has_value() && canData->valid) { 	// If frame has been read
		gpsUnitOneData->first = canData;			// Store frame in gpsUnitOneData
		ROS_DEBUG("GPS1 F 1 read");

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

		ROS_WARN("First CANBUSframe for GPS 1 and GPS 2 were read, but not the second frame for GPS 1 and 2 containing latitude & longitude.");

		return unitTwoFrameOne;
	}

	return noFrames;
}



// Partially fills the struct representing a ROS message with data from first GPS frame. Marks frame as invalid
static void FillRosMessageWithFrameOneData(fsae_electric_vehicle::gps* gps_lap_timer, std::optional<CANData> frameOneData) { // This just needs an optional candata param not a pair
	gps_lap_timer->hours = frameOneData->data[0];		// Time data
	gps_lap_timer->minutes = frameOneData->data[1];		// Time data
	gps_lap_timer->seconds = frameOneData->data[2];		// Time data
	gps_lap_timer->fix = frameOneData->data[3];
	gps_lap_timer->speed = frameOneData->data[4];
	gps_lap_timer->heading = frameOneData->data[5] & frameOneData->data[6]; // This may cause gps_lap_timer.heading to be read wrong since its not a uint16_t

	frameOneData->valid = false; // Mark frame as invalid
}



// Partially fills the struct representing a ROS message with data from second GPS frame. Marks frame as invalid
static void FillRosMessageWithFrameTwoData(fsae_electric_vehicle::gps* gps_lap_timer, point coordinates, bool& frameValid) {
	// Put lat & lon into the struct representing a ROS message
	gps_lap_timer->latitude = coordinates.x;
	gps_lap_timer->longitude = coordinates.y;

	// Mark frame as invalid
	frameValid = false;
}



static point InterpretLatLon(std::optional<CANData> latLonFrame) {
	int32_t latitude;	// latitude format: DDMM.mmmm		This represents Degrees and Minutes. No seconds
	int32_t longitude;	// longitude format: DDDMM.mmmm
	point carCoordinates;

	// Pull the latitude & longitude data from the frame	
	for (int i = 0; i < 4; i++) {
		latitude = latLonFrame->data[i];
		latitude = latitude << 8;
	}
	for (int i = 4; i < 8; i++) {
		longitude = latLonFrame->data[i];
		longitude = longitude << 8;
	}

	// Interpret the lat & lon data 
	float flatitude = static_cast<float>(latitude / 10000);
	float flongitude = static_cast<float>(longitude / 10000);

	carCoordinates.x = flatitude;
	carCoordinates.y = flongitude;

	return carCoordinates;
}



/***************************************************** TODO **********************************************************************/
// Do I need to create an object called RacingSession that will hold various different races. Races will be made up of laps.
// Should be able to start & end racing session

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

// Figure out a way to trigger new lap as accurately as possible

// I was wondering if i should send data to sioSender or process the gps data before establishing startline

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

// Establish the startLine when the car is driving over 20mph for 5 seconds. That way no physical button is needed for the driver to push.

// Can the private functions in RaceTrack object be defined as const? LineIntersect() const {}

/*
A more accurate startLine crossing time can be obtained by adding the timestamp of the previous car position (carCoordinates.p1) and the 
timestamp of the current car position (carCoordinates.p0) and dividing by two. Or better yet, find the intersection point of the startLine
and the line created between the two points in the carCoordinates variable, and use that intersection point to estimate when the car crossed
the startLine.
*/

/* If no start line is defined then the car will just record all the data as one lap. Then later the user can go into the system and define
a startline and the DAS should figure out how many laps were done.*/


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