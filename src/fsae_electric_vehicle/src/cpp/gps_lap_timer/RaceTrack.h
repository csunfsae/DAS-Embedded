// Functions are not supposed to be defined in header files but they are in this case because I cant seem to get the program to link
// correctly if I create another cpp file in addition to the gps_lap_imer.cpp I already have.

#pragma once

#include "GPS.h"

struct point { float x, y; };
struct line { point p0, p1; };

class RaceTrack {
private:
	std::string name;
	point		startPoint;			// Coordinate of start/finish location
	line		startLine;			// Defined by two points
	uint16_t	startHeading;		// Current heading when startLine was created
	line		carCoordinates;		// Coordinates of current & previous GPS location
    int         numOfLaps;
	std::pair<unsigned int, float> bestLapTime;		// Best lap time (lap #, time)

public:
    RaceTrack() {
        name = "Unknown";
        startPoint.x = 0, startPoint.y = 0; // Null Island
        startLine.p0.x = 0, startLine.p0.y = 0, startLine.p1.x = 0, startLine.p1.y = 0;
        startHeading = 361; // Intentionally > 360 degrees
        carCoordinates.p0.x = 0, carCoordinates.p0.y = 0, carCoordinates.p1.x = 0, carCoordinates.p1.y = 0;
        numOfLaps = 0;
        bestLapTime.first = 0, bestLapTime.second = 0.0;
    }

    std::string getName() const { return name; }
    void setName(){
        this->name = name;
    }

	point getStartPoint() const { return startPoint; }

	uint16_t getStartHeading() const { return startHeading; }

	line getCarCoordinates() const { return carCoordinates; }

	void EstablishStartLine(const std::pair<std::optional<CANData>, std::optional<CANData>> gpsUnitData) {
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
		DefineStartLine(startPoint, static_cast<float>(startHeading));

		// Set the current carCoordinates position
		carCoordinates.p0.x = startPoint.x;
		carCoordinates.p0.y = startPoint.y;

		return;
	}

	// Checks if the start line was crossed, returns bool.
	void updateCarCoordinates(point carPosition) {
		carCoordinates.p1 = carCoordinates.p0;
		carCoordinates.p0 = carPosition;

		wasStartLineCrossed();
	}

    void startTimer() {

    }

    void stopTimer() {

    }

private:
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
	void DefineStartLine(const point headingLineCoor, const float sHeading) {
		point headingLineCoor2;	// Second pair of coodinates that defines an infinite imaginary line (headingLine) lying in the direction the car is facing
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

	// Check if the 2 lines intersect
	bool LinesIntersect(const line currentCarCoordinates) {
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

    /*
	// 2d lines point of intersection.
	void IntersectPoint const(const point p1, const point p2, point* intersectionPoint) {
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
    */

    // Check if heading and angle are within 30 degrees of each other
    bool Within30(const uint16_t a, const uint16_t h) { return ((360 - abs(a - h) % 360 < 30) || (abs(a - h) % 360 < 30)); }

    float Distance(const point t1, const point t2) {
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

    void wasStartLineCrossed() {
        if (LinesIntersect(carCoordinates)) {
            // Record lap time with timer

            numOfLaps++;

            // Update the new best lap and best lap time if needed
            /*if (endLapTime < bestLapTime) {
                //bestLapTime.first = numOfLaps; 
                //bestLapTime.second = currentLapTime
            }*/
		}
    }
};