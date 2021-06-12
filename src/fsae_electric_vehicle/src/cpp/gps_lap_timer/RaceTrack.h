// Functions are not supposed to be defined in header files but they are in this case because I cant seem to get the program to link
// correctly if I create another cpp file in addition to the gps_lap_imer.cpp I already have.

#pragma once

#include "GPS.h"

struct point { float x, y; };
struct line { point p0, p1; };

class RaceTrack {
private:
	std::string name;
	point		    startPoint;			// Coordinates of start/finish location
	line		    startLine;			// Defined by two points
	uint16_t	    startHeading;		// Vehicle eading when startLine was created
	line		    carCoordinates;		// Coordinates of current & previous vehicle location
    unsigned int    numOfLaps;
    bool crossedStartLine;
    std::chrono::_V2::steady_clock::time_point lapStartTime;
    std::chrono::duration<float> currentLapTimer;
    std::chrono::duration<float> lapDuration;
	std::pair<unsigned int, std::chrono::duration<float>> bestLapTime; // Best lap time (lap #, time)

public:
    RaceTrack() {
        name = "Unknown";
        startPoint.x = 0, startPoint.y = 0; // Null Island
        startLine.p0.x = 0, startLine.p0.y = 0, startLine.p1.x = 0, startLine.p1.y = 0;
        startHeading = 361; // Intentionally > 360 degrees
        carCoordinates.p0.x = 0, carCoordinates.p0.y = 0, carCoordinates.p1.x = 0, carCoordinates.p1.y = 0;
        numOfLaps = 0;
        crossedStartLine = false;
        lapStartTime = {};
        lapDuration = {};
        bestLapTime.first = 0, bestLapTime.second = {};
    }

    std::string GetName() const      { return name; }

    void SetName()                   { this->name = name; }

	point GetStartPoint() const      { return startPoint; }

    line GetStartLine() const        { return startLine; }

	uint16_t GetStartHeading() const { return startHeading; }

	line GetCarCoordinates() const   { return carCoordinates; }

    int GetNumOfLaps() const         { return numOfLaps; }

    bool WasStartLineCrossed() const { return crossedStartLine; }

    auto GetCurrentLapTimer() const  { return currentLapTimer; }

    auto GetLapDuration() const      { return lapDuration; }

    std::pair<unsigned int, std::chrono::duration<float>> GetBestLapTime() const     { return bestLapTime; }



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

        ROS_INFO("StartLine is Established!");

		return;
	}

	// Checks if the start line was crossed, returns bool.
	void UpdateCarCoordinates(point newCoordinates) {
        auto currentTime = std::chrono::steady_clock::now();
        currentLapTimer = currentTime - lapStartTime;

		carCoordinates.p1 = carCoordinates.p0;  // Move the cars previous coordinates into p1
		carCoordinates.p0.x = newCoordinates.x; // New car coordinates always go into p0
        carCoordinates.p0.y = newCoordinates.y;

		WasStartLineCrossed(currentTime);
	}

    void StartTimer() {
        lapStartTime = std::chrono::steady_clock::now();
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
    //
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

        ROS_DEBUG("StartLine is defined.");
	}

    /*
    Determining if two lines intersect is accomplished by taking two points from 'line one' (point one and point two) and a single point from 'line two' (point three).
    Draw an imaginary line(not realated to imaginary numbers) from the first point to the second point and then another imaginary line from point two 
    to point three, and then draw anothe imaginary line from point three to the first point. The direction of those imaginary lines will dictate the 'orientaion'.
    Now do the same thing again but this time change point three to be the point on line two that wasnt used for the previous imaginary triangle. If the orientation
    of those two triangles are different then the lines may intersect. More steps are needed to determine with certainty if the lines will intersect, but that is
    the general idea of what is going on in this LinesIntersect() function. A simple google search will show you indetail how to determine if two lines intersect.
    This problem can also be thought of in terms of vectors and their cross product. The determinant will indicate which orientation the imaginary triangle is 
    */
    //
	// Check if the line between the cars current position and the cars previous position intersects with the startLine. If they do, then that counts as crossing the startLine
	bool LinesIntersect() {
		float z;
        enum orientation { collinear = 0, clockwise, counterClockwise };
        orientation orientation1, orientation2, orientation3, orientation4;

		// Quick rejection test
		if (!(MAX(startLine.p0.x, startLine.p1.x) >= MIN(carCoordinates.p0.x, carCoordinates.p1.x) &&
			MAX(carCoordinates.p0.x, carCoordinates.p1.x) >= MIN(startLine.p0.x, startLine.p1.x) &&
			MAX(startLine.p0.y, startLine.p1.y) >= MIN(carCoordinates.p0.y, carCoordinates.p1.y) &&
			MAX(carCoordinates.p0.y, carCoordinates.p1.y) >= MIN(startLine.p0.y, startLine.p1.y)))
			return false;

		// Determine orientation of the imaginary triangle. Do this for each of the 4 imaginary triangles
		if ((z = ((carCoordinates.p0.x - startLine.p0.x) * (startLine.p1.y - startLine.p0.y)) - ((carCoordinates.p0.y - startLine.p0.y) * (startLine.p1.x - startLine.p0.x))) < 0.0f)
			orientation1 = counterClockwise; // Counter-Clockwise
		else if (z > 0.0f)
			orientation1 = clockwise;  // Clockwise
		else
			orientation1 = collinear;  // Collinear

        // An interesting thing about this equation.... Its the determinant of the matrix formed by the 4 points that make up the two lines
		if ((z = ((carCoordinates.p1.x - startLine.p0.x) * (startLine.p1.y - startLine.p0.y)) - ((carCoordinates.p1.y - startLine.p0.y) * (startLine.p1.x - startLine.p0.x))) < 0.0f)
			orientation2 = counterClockwise;
		else if (z > 0.0f)
			orientation2 = clockwise;
		else
			orientation2 = collinear;

		if ((z = ((startLine.p0.x - carCoordinates.p0.x) * (carCoordinates.p1.y - carCoordinates.p0.y)) - ((startLine.p0.y - carCoordinates.p0.y) * (carCoordinates.p1.x - carCoordinates.p0.x))) < 0.0f)
			orientation3 = counterClockwise;
		else if (z > 0.0f)
			orientation3 = clockwise;
		else
			orientation3 = collinear;

		if ((z = ((startLine.p1.x - carCoordinates.p0.x) * (carCoordinates.p1.y - carCoordinates.p0.y)) - ((startLine.p1.y - carCoordinates.p0.y) * (carCoordinates.p1.x - carCoordinates.p0.x))) < 0.0f)
			orientation4 = counterClockwise;
		else if (z > 0.0f)
			orientation4 = clockwise;
		else
			orientation4 = collinear;

		if ((orientation1 * orientation2 <= 0) && (orientation3 * orientation4 <= 0)) {
			ROS_DEBUG("StartLine and carCoordinates line intersect!");
            return true;
        }

		return false; // Line segments do not intersect
	}

    void WasStartLineCrossed(auto currentTime) {
        if (LinesIntersect()) {
            // Record Lap Duration
            lapDuration = currentTime - lapStartTime;
            lapStartTime = currentTime;

            crossedStartLine = true;

            ROS_INFO("Car has crossed the startLine!");

            // Update the new best lap and best lap time if needed
            if (lapDuration < bestLapTime.second) {
                bestLapTime.first = numOfLaps;
                bestLapTime.second = lapDuration;
                ROS_INFO("New best lap time recorded!");
            }

            numOfLaps++;
		} else {
            lapDuration = {}; // Reset lapDuration
            crossedStartLine = false;
        }
    }

/*
// These 2 functions are intended to be used for estimating the exact time the car crosses the finish line without needing to increase the GPS update frequency
// This is done by determining where the point of intersection is between the startLine and the line between the previous car position (carCoordinates.p1) and
// the current car position (carCoordinates.p0). Once we know the intersection point then we can estimate the exact time the car crossed the line based on its
// speed while crossing the startLine

	// Determine the point of intersection between two lines
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
    */

/*
// This function is intended to be used to lower the amount of false positives of the car crossing the startLine. A new lap would only be triggered if
// the cars heading was between 75 and 105 degrees assuming that the startLine was a straight line pointing in the direction of 0 and 180 degrees on
// the trigonometric unit circle.

    // Check if heading and angle are within 30 degrees of each other
    bool Within30(const uint16_t a, const uint16_t h) { return ((360 - abs(a - h) % 360 < 30) || (abs(a - h) % 360 < 30)); }
*/
};