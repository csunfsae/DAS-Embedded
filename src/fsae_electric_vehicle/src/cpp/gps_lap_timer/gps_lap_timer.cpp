/******************************************************************************************************************************************
 *  This code only works in the PST time zone
 * Im not using the "valid" field within the CANBUS frame
 * 
 * This programs goal is to read the CANBUS frames sent from the Adafruit GPS units and use it to calculate the number of laps, lap times,
 * & best lap time. That requires this program to know the cars ground location, the ground location of the start/finish line. This program
 * will know when the car has crossed the start/finish line by drawing a line between its current GPS position and its previous GPS
 * location and then checking to see if that line intersects with the start/finish line.
 * getData() runs in less than 0.00006 seconds
 * 
 * 
 * 
 * 
 * What Ive done since last commit:
 * Added validFrames Enum to make frames as valid or not. Not using the valid indicator in the frame itself
 * Added code to change the ROS log level
*******************************************************************************************************************************************/

/* TIMING CODE
auto startTime = std::chrono::high_resolution_clock::now();
auto endTime = std::chrono::high_resolution_clock::now();
std::chrono::duration<float> duration = endTime - startTime;
std::cout << "Duration: " << duration.count() << std::endl;
*/

#include "ros/ros.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cassert>
#include <math.h>
#include <array>
#include "GPS.h" // Custom header
#include <thread>
#include <chrono>

#ifdef _WIN32	// If in a Windows environment
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define _CRT_SECURE_NO_WARNINGS // For MSVC

point_t startPoint;		// Coordinate of start/finish location.
line_t startLine;		// startLine defined by two points
uint16_t startHeading;	// Heading crossing start/finish.
line_t carCoordinates;	// Coordinates of current & previous GPS location.



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

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}

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
			EstablishStartLine(gpsUnitOneData);
		}
		else if (updatedFrames = unitTwoFrameOneAndTwo) {
			//std::cout << "In If 2" << std::endl;
			EstablishStartLine(gpsUnitTwoData);
		}
		else
			ROS_ERROR("Cannot Establish a Start Line due to at least 1 out of 2 CANBUS frames missing!");

		//std::cout << "AFTER If" << std::endl;
	}

	ros::Rate loop_rate(40); // This means that loop rate can be up to 30 times per second
  	
	validFrame updatedFrames = noFrames;
	while (ros::ok()) {
		ROS_INFO_ONCE("ROS is ok! Entered the main while loop");

		// Read CANBUS frames from the CANBUS header
		updatedFrames = GetGPSData(&can, &gpsUnitOneData, &gpsUnitTwoData);

		// Process the GPS data
		if (updatedFrames != noFrames) {
			//LapTimer(gpsUnitOneData, gpsUnitTwoData);
		}

		// Store data in the CANBUS frame depending on which frame is recieved
		// Its ok if we have the first frame but not the second because we can verify that we still have fix.
		// Its not ok if we have the 2nd frame and not the first because we cannot verify that we still have fix.
		switch (updatedFrames) {
			case noFrames: // Do nothing
				break;
			case unitOneFrameOne:
				FillRosMessageWithFrameOneData(&gps_lap_timer, gpsUnitOneData.first);
				break;
			case unitOneFrameOneAndTwo:
				FillRosMessageWithFrameOneData(&gps_lap_timer, gpsUnitOneData.first);
				FillRosMessageWithFrameTwoData(&gps_lap_timer, gpsUnitOneData.second);
				break;
			case unitTwoFrameOne:
				FillRosMessageWithFrameOneData(&gps_lap_timer, gpsUnitTwoData.first);
				break;
			case unitTwoFrameOneAndTwo:
				FillRosMessageWithFrameOneData(&gps_lap_timer, gpsUnitOneData.first);
				FillRosMessageWithFrameTwoData(&gps_lap_timer, gpsUnitTwoData.second);
				break;
		}

		/* Other stuff to send to sioSender.cpp
		start line
		lap #
		current lap time
		best lap time & its corresponding lap
		ending lap time
		maybe startHeading
		*/

		// Send ROS message filled with GPS data to sioSender.cpp
		gps_lap_timer_pub.publish(gps_lap_timer);

		ros::spinOnce();
   		loop_rate.sleep();
  	}
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

		ROS_INFO_THROTTLE(2, "Waiting for GPS fix!\n");
		ROS_WARN_DELAYED_THROTTLE(120, "Taking a long time to find a GPS fix.");

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	} while (true);

	gpsUnitOneData->first->valid = 0;
	gpsUnitTwoData->first->valid = 0;

	ROS_INFO("Found FIX!");
}



// Read CANBUS frames from the CANBUS header and store them in the gpsUnitOneData or gpsUnitTwoData pairs. Return turn true means a pair is full
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



static void EstablishStartLine(const std::pair<std::optional<CANData>, std::optional<CANData>> gpsUnitData) {
	float posTimestamp, latitudeCoordinate, longitudeCoordinate;

	// Verify that GPS has fix
	if (!gpsUnitData.first->data[GPS_FIX]) {
		ROS_ERROR("Tried to Establish StartLine but the GPS doesnt have a fix.");
		return;
	}
	// Get current coordinates of car (lat, lon)
	//startPoint.x = 
	//startPoint.y = 

	// Heading while crossing start/finish. Vehicle should be as parallel to the sides of the track as possible
	startHeading = gpsUnitData.first->data[GPS_HEADING_1];

	// Define startline
	StartLine(startPoint, (float)startHeading);

	// Set the current carCoordinates position
	carCoordinates.p0.x = startPoint.x;
	carCoordinates.p0.y = startPoint.y;

	return;
}



/*
So to create a start line we need to represent the start line as a pair of x & y coordinates. The GPS coordinates of the vehicle will
act as the x & y coordinates. Using 2 coordinates can only represent an opject in a 2d plane, but since the distances we are dealing
with are so small relative to the circumference of the earth, using 2 GPS coordinates will work just fine. Tracking the position of
a vehicle going long distances in a single direction on the surface of the Earth will not work with this program because the Earth
is not flat. #FlatEarthSociety This program doesnt prove that the Earth is round though so the flat earthers win again. So to create
the startline we need to first create a line that lies in the direction that the car is facing. That line will be called headingLine.
One (x,y) coordinate of headingLine will be the GPS coordinates of the car. The 2nd (x,y) coordinate will need to be created. Once we
have that headingLine defined then we can go ahead and create a new line called start Line that lies perpendicular to headingLine &
the intersection point will be the GPS coordinates of the car when the EstablishStartLine function is called. The start line cannot
be an infinite line or else the car will cross that infinite start line at least twice during every lap so we need to define endpoints
for the start line. The start line length will have to represent about 20-50 feet in the real world, which will translate to about
0.000110 degrees in GPS coordinates. The length of the startline should be user defineable.
*/

// Construct a startline.
static void StartLine(const point_t headingLineCoor, const float sHeading) {
	point_t headingLineCoor2;	// Second pair of coodinates that defines an infinite imaginary line (headingLine) lying in the direction the car is facing
	float m, b, temp;			// Slope & y-intercept of that line

	// Create another (x,y) coordinate that lies in the same line as headingLine
	// This creates a line in our imaginary 2d plane that points in the direction the car is facing
	// This line is used for creating a startline that is perpendicular to this one that is being created
	headingLineCoor2.x = headingLineCoor.x + PROJECTION_DISTANCE * cos(DEGTORAD(sHeading));
	headingLineCoor2.y = headingLineCoor.y + PROJECTION_DISTANCE * sin(DEGTORAD(sHeading));
	
	// Calculate the slope & y-intercept of headingLine 
	m = (headingLineCoor.y - headingLineCoor2.y) / (headingLineCoor.x - headingLineCoor2.x);
	b = headingLineCoor.y - (m * headingLineCoor.x); // I dont think i need this line because this b is immediately redefined below

	// Construct the perpendicular line (start line) slope & y-intercept.
	m = -1.0f / m;
	b = headingLineCoor.y - (m * headingLineCoor.x);

	// Define endpoints of the perpendicular line (start line)
	// I think this defines the endpoints hundreds of miles apart due to LINE_WIDTH_2 being too big. Should be < 1
	temp = headingLineCoor.x + LINE_WIDTH_2;
	startLine.p0.y = (m * temp + b);
	startLine.p0.x = temp;
	temp -= LINE_WIDTH;
	startLine.p1.y = (m * temp + b);
	startLine.p1.x = temp;
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
static void FillRosMessageWithFrameTwoData(fsae_electric_vehicle::gps* gps_lap_timer, std::optional<CANData> frameTwoData) { // This just needs an optional candata param not a pair
	int32_t latitude;	// latitude format: DDMM.MMMM		This represents Degrees and Minutes. No seconds
	int32_t longitude;	// longitude format: DDDMM.MMMM

	// Pull the latitude & longitude data from the frame	
	for (int i = 0; i < 4; i++) {
		latitude = frameTwoData->data[i];
		latitude = latitude << 8;
	}
	for (int i = 4; i < 8; i++) {
		longitude = frameTwoData->data[i];
		longitude = longitude << 8;
	}

	// Interpret the lat & lon data 
	float flatitude = static_cast<float>(latitude / 10000);
	float flongitude = static_cast<float>(longitude / 10000);

	// Put lat & lon into the struct representing a ROS message
	gps_lap_timer->latitude = latitude;
	gps_lap_timer->longitude = longitude;

	// Mark frame as invalid
	frameTwoData->valid = false;
}



static void Run(float timeStamp, char *tokens[]) {
	// Lap counters.
	uint8_t numLaps = 0;
	uint16_t hzCounter = 1;
	
	// Lap data.
	std::array<lap, 256> race;
	
	// Best lap time (lap #, time).
	std::pair<uint8_t, float> bestTime(0, 0.0f);

	// Note timestamp of startline point.
	race[numLaps].setStart(timeStamp);

	// Previous position GPS time stamp.
	float prevTimeStamp = timeStamp;

	// Confirm sentence is sequential.
	/* Doesnt seem like were going to need this function
	timeStamp = ConvertToSeconds(tokens[RMC_TIME]);
	if (!Equal(timeStamp, prevTimeStamp + GPS_UPDATE_PERIOD)) {
		error.SetError(err::ID::TIME_STAMP);
		ROS_DEBUG("GPS has no fix");
	} */

	// Get current carCoordinates position (lat, lon)
	// GeoCopy(canData.data[GPS_LATITUDE], temp, LATITUDE);

	// Ignore gps sentences for 1 second after crossing start/finish.
	if (hzCounter < GPS_UPDATE_FREQUENCY) {
		hzCounter++;

		// Prepare for next iteration.
		carCoordinates.p0.x = carCoordinates.p1.x;
		carCoordinates.p0.y = carCoordinates.p1.y;
	}
	
	/*
	// Heading sanity check & check if crossed start/finish line?
	if (Within30(startHeading, (uint16_t)atol(tokens[RMC_TRACK])) && LineIntersection(carCoordinates)) {
		point_t intersectPoint;

		// Calculate track/start line intersection point.
		IntersectPoint(carCoordinates.p0, carCoordinates.p1, &intersectPoint);

		// Overall length of this track segment.
		float totDist = Distance(carCoordinates.p0, carCoordinates.p1);
		// Length from start line intersection point to track segment end point.
		float segDist = Distance(intersectPoint, carCoordinates.p1);

		// Calculate startline crossing time for this and next lap.
		float crossTime = timeStamp - (GPS_UPDATE_PERIOD * (segDist / totDist));
		race[numLaps].setStop(crossTime);		// Record lap end time for the lap just finished
		race[numLaps + 1].setStart(crossTime);	// Record lap start time for the lap just started

		// Determine current lap stats.
		DisplayTime(numLaps + 1, race[numLaps].getTime());
		if (numLaps > 0)
			DisplayTime(bestTime.first + 1, bestTime.second);

		// Is this lap a new best?
		if (numLaps == 0 || race[numLaps].getTime() < bestTime.second) {
			// Announce new fast lap.
			std::cout << " << Fast Lap";
			bestTime = std::make_pair(numLaps, race[numLaps].getTime());
		}
		std::cout << "\n";

		// Increment counters. These probably dont work
		numLaps++;
		hzCounter = 1;
	}
	*/

	// Prepare for next iteration.
	carCoordinates.p0.x = carCoordinates.p1.x;
	carCoordinates.p0.y = carCoordinates.p1.y;
}



static float Distance(const point_t t1, const point_t t2) {
	float Lat1, Long1, Lat2, Long2;	// Coordinates in degrees.
	float dlat, dlon;				// Change in location.
	float a, d;
	float c; // Great Circle distance (radians).

	Lat1 = (float)((uint32_t)(t1.y / 100.0f));
	Lat1 = (float)(Lat1 + (t1.y - Lat1 * 100.0f) / 60.0f);
	Lat1 = (float)(Lat1 * D_TO_RADIANS);

	Long1 = (float)((uint32_t)(t1.x / 100.0f));
	Long1 = (float)(Long1 + (t1.x - Long1 * 100.0f) / 60.0f);
	Long1 = (float)(Long1 * D_TO_RADIANS);

	Lat2 = (float)((uint32_t)(t2.y / 100.0f));
	Lat2 = (float)(Lat2 + (t2.y - Lat2 * 100.0f) / 60.0f);
	Lat2 = (float)(Lat2 * D_TO_RADIANS);

	Long2 = (float)((uint32_t)(t2.x / 100.0f));
	Long2 = (float)(Long2 + (t2.x - Long2 * 100.0f) / 60.0f);
	Long2 = (float)(Long2 * D_TO_RADIANS);

	dlat = Lat2 - Lat1;
	dlon = Long2 - Long1;

	a = (float)((sin(dlat / 2.0f) * sin(dlat / 2.0f)) + cos(Lat1) * cos(Lat2) * (sin(dlon / 2.0f) * sin(dlon / 2.0f)));
	c = (float)(2.0f * atan2(sqrt(a), sqrt(1.0f - a)));

	d = (float)(EARTH_RADIUS * c);

	return d;
}



// Check if the 2 lines intersect
static bool LineIntersection(const line_t carCoordinates) {
	float z;
	int16_t s1, s2, s3, s4;

	// Quick rejection test.
	if (!(MAX(startLine.p0.x, startLine.p1.x) >= MIN(carCoordinates.p0.x, carCoordinates.p1.x) &&
		MAX(carCoordinates.p0.x, carCoordinates.p1.x) >= MIN(startLine.p0.x, startLine.p1.x) &&
		MAX(startLine.p0.y, startLine.p1.y) >= MIN(carCoordinates.p0.y, carCoordinates.p1.y) &&
		MAX(carCoordinates.p0.y, carCoordinates.p1.y) >= MIN(startLine.p0.y, startLine.p1.y)))
		return false;

	// Check to see if lines are collinear
	if ((z = ((carCoordinates.p0.x - startLine.p0.x) * (startLine.p1.y - startLine.p0.y)) - ((carCoordinates.p0.y - startLine.p0.y) * (startLine.p1.x - startLine.p0.x))) < 0.0f)
		s1 = -1; // Counterclockwise.
	else if (z > 0.0f)
		s1 = 1;  // Clockwise.
	else
		s1 = 0;  // Collinear.

	if ((z = ((carCoordinates.p1.x - startLine.p0.x) * (startLine.p1.y - startLine.p0.y)) - ((carCoordinates.p1.y - startLine.p0.y) * (startLine.p1.x - startLine.p0.x))) < 0.0f)
		s2 = -1;
	else if (z > 0.0f)
		s2 = 1;
	else
		s2 = 0;

	if ((z = ((startLine.p0.x - carCoordinates.p0.x) * (carCoordinates.p1.y - carCoordinates.p0.y)) - ((startLine.p0.y - carCoordinates.p0.y) * (carCoordinates.p1.x - carCoordinates.p0.x))) < 0.0f)
		s3 = -1;
	else if (z > 0.0f)
		s3 = 1;
	else
		s3 = 0;

	if ((z = ((startLine.p1.x - carCoordinates.p0.x) * (carCoordinates.p1.y - carCoordinates.p0.y)) - ((startLine.p1.y - carCoordinates.p0.y) * (carCoordinates.p1.x - carCoordinates.p0.x))) < 0.0f)
		s4 = -1;
	else if (z > 0.0f)
		s4 = 1;
	else
		s4 = 0;

	if ((s1 * s2 <= 0) && (s3 * s4 <= 0))
		return true;

	return false; // Line segments do not intersect.
}



// 2d lines point of intersection.
static void IntersectPoint(const point_t p1, const point_t p2, point_t* intersectionPoint) {
	float denom, numera, mua; //numerb, mub;

	// No checks because it is assumed the lines intersect.
	denom = (startLine.p1.y - startLine.p0.y) * (p2.x - p1.x) - (startLine.p1.x - startLine.p0.x) * (p2.y - p1.y); // y - Py
	numera = (startLine.p1.x - startLine.p0.x) * (p1.y - startLine.p0.y) - (startLine.p1.y - startLine.p0.y) * (p1.x - startLine.p0.x); // x - Px
	//numerb = (p2.x - p1.x)*(p1.y - startingLine1.y) - (p2.y - p1.y)*(p1.x - startingLine1.x);

	mua = numera / denom; // Calculate the slope
	//mub = numerb/denom;
	intersectionPoint->x = p1.x + mua * (p2.x - p1.x); // Point-Slope-Form y = m(x - Px) + Py
	intersectionPoint->y = p1.y + mua * (p2.y - p1.y);
}



// Determine if floats are relatively equal.
static bool Equal(float a, float b) { return fabs(a - b) <= FLT_EPSILON; }



// Check if heading and angle are within 30 degrees of each other
static bool Within30(const uint16_t a, const uint16_t h) { return ((360 - abs(a - h) % 360 < 30) || (abs(a - h) % 360 < 30)); }



/***************************************************** TODO **********************************************************************/
// Need to create an object called RacingSession that will hold various different races. Races will be made up of laps.
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

// Vehicle data should be stored locally on the Quasar if there is no connection to the server

// Figure out a way to trigger new lap as accurately as possible

// Create an std::array<CANData, 2> filled with the 2 canbus frames that are sent by one GPS unit. Maybe name it GPSUpdateCycle.

// Data that will also need to be sent to the server from this program will be lap #, lap time, best lap, best time,

// i was wondering if i should send data to sioSender or process the gps data before establishing startline

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


/******************************************************* Unused code *************************************************************
// UBX Protocol.
//UBX   SYNC  CLSS  ID    LENGTH     MSG                            CHECKSUM
//0xB5, 0x62, 0x06, 0x08, 0x06,0x00, 0x64,0x00,0x01,0x00,0x01,0x00, 0x7A,0x12 // 10Hz
//0xB5, 0x62, 0x06, 0x08, 0x06,0x00, 0xC8,0x00,0x01,0x00,0x01,0x00, 0xDE,0x6A // 5Hz
//0xB5, 0x62, 0x06, 0x08, 0x06,0x00, 0xE8,0x03,0x01,0x00,0x01,0x00, 0x01,0x39 // 1Hz
// Can also disable/enable NMEA sentences with a text command: 
// $PUBX,40,GLL,1,0,0,0,0,0*5D enables the GLL sentence on the serial port.
// $PUBX,41,1,0007,0003,19200,0*25 sets the GPS modules baud rate to 19200.
/*std::array<unsigned char, 14> gpsUpdateRate = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A }; // 5Hz update rate.
std::array<unsigned char, 16> gpsDisableGGA = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24 }; // GxGGA off
std::array<unsigned char, 16> gpsDisableGGL = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B }; // GxGLL off
std::array<unsigned char, 16> gpsDisableGSA = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 }; // GxGSA off
std::array<unsigned char, 16> gpsDisableGSV = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 }; // GxGSV off
std::array<unsigned char, 16> gpsDisableVTG = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47 }; // GxVTG off
std::array<unsigned char, 32> gpsBaud19200 = { "$PUBX,41,1,0007,0003,19200,0*25" }; // 19200 baud rate*/

/*static void gpsSetup(const PORT p)
{
	assert(p != NULL);

	SendData(p, &gpsDisableGGA[0], gpsDisableGGA.size()); // Disable GGA.
	SendData(p, &gpsDisableGGL[0], gpsDisableGGL.size()); // Disable GGL.
	SendData(p, &gpsDisableGSA[0], gpsDisableGSA.size()); // DIsable GSA.
	SendData(p, &gpsDisableGSV[0], gpsDisableGSV.size()); // Disable GSV.
	SendData(p, &gpsDisableVTG[0], gpsDisableVTG.size()); // Disable VTG.
	SendData(p, &gpsUpdateRate[0], gpsUpdateRate.size()); // Set update rate at 5Hz.
	//SendData(p, &gpsBaud19200[0], gpsBaud19200.size()); // Set 19200 baud rate.
}*/

/*
// An RMC sentence is a standardized string of chars emitted by GPS units
static bool GetRMCSentence(char* tokens[]) {
	error.Clear();

	float seconds = 0, latitude = 0, longitude = 0, speed = 0, magneticVar = 0, trueCourse = 0;
	//std::cin.get();
	CANController can; // Start the CABUS header on the Jetson/Quasar board
	can.start("can0");
	auto data = can.getData(0x34, 0x1FFFFFFF); // First param is idFilter
	if (data.has_value()) {
		//std::memcpy(&buffer, data->data, RMC_CHECKSUM + 1); //
		seconds = data->data[0]; // 'seconds' can be later converted into hours:minutes:seconds later if needed
		latitude = data->data[1];
		longitude = data->data[2];
		speed = data->data[3];
		magneticVar = data->data[4];
		trueCourse= data->data[5];
	} else {
		return false;
	}

	// RMC sentence?
	if (strncmp("$GPRMC", buffer, 6) == 0)
	{
		// Terminate sentence at eol.
		char* eol = strchr(buffer, 0x0a);
		if (eol != NULL)
#ifdef FILE_INPUT
			buffer[eol - buffer] = 0;
#else
			buffer[eol - buffer - 1] = 0;
#endif

		// Confirm crc.
		if (!Checksum(buffer))
		{
			error.SetError(err::ID::CHECKSUM);
			ROS_FATAL("CRC not confirmed!");
			ROS_DEBUG("CRC not confirmed!");
			return false;
		}

		// Parse the GPS RMC string and check for valid fix.
		if (ParseRMC(buffer, tokens) == RMC_CHECKSUM) {
			if (tokens[RMC_STATUS] == nullptr || *tokens[RMC_STATUS] != 'A') {
				error.SetError(err::ID::NO_FIX);
				ROS_DEBUG("GPS has no fix");
				return false;
			}
			else
				return true;
		}
	}

	error.SetError(err::ID::BAD_SENTENCE);
	ROS_DEBUG("Not an RMC sentence!");
	return false;
}
*/

/*
static void storeFrameData(CANData canbusFrame, fsae_electric_vehicle::gps gps_lap_timer) {
	int32_t latitude, longitude;

	switch(canbusFrame->id) {
		case 0x35 || 0x37:
			gps_lap_timer.hours = canbusFrame->data[0];
			gps_lap_timer.minutes = canbusFrame->data[1];
			gps_lap_timer.seconds = canbusFrame->data[2];
			gps_lap_timer.fix = canbusFrame->data[3];
			gps_lap_timer.speed = canbusFrame->data[4];
			gps_lap_timer.heading = canbusFrame->data[5] & canbusFrame->data[6]; // This may cause gps_lap_timer.heading to be read wrong since its not a uint16_t
			break;
		case 0x36 || 0x38:
			for (int i = 0; i < 4; i++) {
				latitude = canbusFrame->data[i];
				latitude = latitude << 8;
			}
			for (int i = 4; i < 8; i++) {
				longitude = canbusFrame->data[i];
				longitude = longitude << 8;
			}
			float flatitude = (float)(latitude / 10000);
			float flongitude = (float)(longitude / 10000);
			gps_lap_timer.latitude = latitude;
			gps_lap_timer.longitude = longitude;
			break;
		case 0x37:
			gps_lap_timer.hours = canbusFrame->data[0];
			gps_lap_timer.minutes = canbusFrame->data[1];
			gps_lap_timer.seconds = canbusFrame->data[2];
			gps_lap_timer.fix = canbusFrame->data[3];
			gps_lap_timer.speed = canbusFrame->data[4];
			gps_lap_timer.heading = canbusFrame->data[5] & canbusFrame->data[6]; // This may cause gps_lap_timer.heading to be read wrong since its not a uint16_t
			break;
		case 0x38:
			for (int i = 0; i < 4; i++) {
				latitude = canbusFrame->data[i];
				latitude = latitude << 8;
			}
			for (int i = 4; i < 8; i++) {
				longitude = canbusFrame->data[i];
				longitude = longitude << 8;
			}
			float flatitude = (float)(latitude / 10000);
			float flongitude = (float)(longitude / 10000);
			gps_lap_timer.latitude = latitude;
			gps_lap_timer.longitude = longitude;
			break;
	}
}



static bool readCanbusGPSData(CANController &can, CANData* canbusFrame) {
	// Read CANBUS frames from the CANBUS
	// If CANBUS frames are stored in the vector<CANData> m_dataMap for a long time then reading data from the CANBUS like this will cause old frames to be
	// read when newer frames are available. Idk if multiple frames with the same ID are kept in the vector simultaneously. Idk how many frames the vector
	// stores before it starts throwing frames out.
	std::optional<CANData> canData = can.getData(0x35, 0x1FFFFFFF); // First param is the CAN frame ID, 2nd is ID mask.
	std::optional<CANData> canData = can.getData(0x36, 0x1FFFFFFF); // These will always return data because the GPS has a fix
	std::optional<CANData> canData = can.getData(0x37, 0x1FFFFFFF);
	std::optional<CANData> canData = can.getData(0x38, 0x1FFFFFFF);

	if (canData->data[3] == 0) // If GPS doesnt have a fix (which it always should at this point)
		return false;

	canbusFrame->data[0] = canData->data[0];
	canbusFrame->data[1] = canData->data[1];
	canbusFrame->data[2] = canData->data[2];
	canbusFrame->data[3] = canData->data[3];
	canbusFrame->data[4] = canData->data[4];
	canbusFrame->data[5] = canData->data[5];
	canbusFrame->data[6] = canData->data[6];
	return true;
}


/*
/*************************************************** GPS Utility Functions *********************************************
static void DisplayTime(const uint8_t, const float);

// A very basic atof function (no exponentials, no sign).
static float atof_(char s[]) {
	float val, power;
	int8_t i = 0;

	while (!isdigit(s[i]))
		i++;

	for (val = 0.0f; isdigit(s[i]); i++)
		val = 10.0f * val + (s[i] - '0');

	if (s[i] == '.')
		i++;

	for (power = 1.0f; isdigit(s[i]); i++)
	{
		val = 10.0f * val + (s[i] - '0');
		power *= 10.0f;
	}

	return val / power;
}



// strtok implementation which recognizes consecutive delimiters.
static char* strtok_(char* str, const char* delim) {
	static char* staticStr = 0;          // Stores last address.
	int i = 0, strLen = 0, delimLen = 0; // Indexes.

	// If delimiter is NULL or no more chars remaining.
	if (delim == 0 || (str == 0 && staticStr == 0))
		return 0;

	if (str == 0)
		str = staticStr;

	// Get length of string and delimiter.
	while (str[strLen])
		strLen++;
	while (delim[delimLen])
		delimLen++;

	// Find a delimiter.
	char* p = strpbrk(str, delim);
	if (p)
		i = p - str;
	else
	{
		// If no delimiters, return str.
		staticStr = 0;
		return str;
	}

	// Terminate the string.
	str[i] = '\0';

	// Save remaining string.
	if ((str + i + 1) != 0)
		staticStr = (str + i + 1);
	else
		staticStr = 0;

	return str;
}



// Convert hex string to decimal.
static char hex(const char ch) {
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	if (ch >= 'a' && ch <= 'f')
		return ch - 'a' + 10;

	return 0;
}



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


// Copy lat/long strings and format as ddd.dddd
static void GeoCopy(const char* s, char* d, const unsigned char value) {
	assert(s != nullptr && d != nullptr);

	int i = 0;

	// Copy all numerals, insert/skip decimal point.
	do {
		if (value == LONGITUDE && i == 3)
		{
			*d++ = '.';
			i++;
		}

		if (value == LATITUDE && i == 2)
		{
			*d++ = '.';
			i++;
		}

		if ((*s >= '0') && (*s <= '9'))
		{
			*d++ = *s;
			i++;
		}
	} while (*s++ != '\0');

	// Null terminate.
	*d = '\0';
}



// Prepends s onto d. Assumes d has enough space allocated for the combined string.
static void Prepend(char* d, const char* s) {
	assert(s != nullptr && d != nullptr); 
	
	size_t len = strlen(s);

	memmove(d + len, d, strlen(d) + 1);
	memcpy(d, s, len);
}



// Parse GPS string into tokens
static size_t ParseRMC(char* sentence, char* tokens[]) {
	assert(sentence != nullptr);

	size_t n = 0;

	tokens[n] = strtok_(sentence, ",");
	while (tokens[n] && n < RMC_CHECKSUM)
		tokens[++n] = strtok_(NULL, ",*");

	return n;
}



template<typename T>
static T htoi(const char* hexStr) {
	T value = T{ 0 };

	if (hexStr != nullptr)
		for (size_t i = 0; i < sizeof(T) * 2; ++i)
			value |= hex(tolower(hexStr[i])) << (8 * sizeof(T) - 4 * (i + 1));

	return value;
};



// Verify the checksum of the RMC string
static bool Checksum(char* sentence)
{
	assert(sentence != nullptr);

	uint8_t crc = 0;
	uint8_t n = htoi<uint8_t>(&sentence[strlen(sentence) - 2]);

	// Skip initial '$' and '*' + last 2 bytes (crc).
	for (size_t i = 1; i < strlen(sentence) - 3; i++)
		crc ^= sentence[i];

	return (crc == n);
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