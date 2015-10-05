//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"
#include <math.h>

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI
	
	if(!checkRobust()) {
		return;
	}

	std::vector<Point> drawPoints;

	for(float t=0; t<controlPoints[controlPoints.size()-1].time; t=t+1) {
		Point newPoint;
		calculatePoint(newPoint, t);
		drawPoints.push_back(newPoint);
	}
	drawPoints.push_back(controlPoints[controlPoints.size()-1].position);

	
	for(int i=0; i<drawPoints.size()-1; i++) {
		DrawLib::drawLine(drawPoints[i], drawPoints[i+1], curveColor, curveThickness);
	}

	// Robustness: make sure there is at least two control point: start and end points

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	std::vector<CurvePoint> temp;

	int i = 0;
	for(i; i<controlPoints.size()-1; i++) {
		int minIndex = i;
		for(int j=i+1; j<controlPoints.size(); j++) {
			if(controlPoints[j].time < controlPoints[minIndex].time) {
				minIndex = j;
			}
		}
			
		temp.push_back(controlPoints[minIndex]);
		controlPoints[minIndex] = controlPoints[i];
		controlPoints[i] = temp[0];
		temp.clear();
	}

	/*for(int j=0; j<controlPoints.size(); j++) {
		std::cout << controlPoints[j].time << " : " << controlPoints[j].position << std::endl;
	}*/

	return;
}


// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
		//std::cout << outputPoint << std::endl;
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if(controlPoints.size()<=1) {
		return false;
	}

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	for(int i=0; i<controlPoints.size(); i++) {
		if(time < controlPoints[i].time) {
			nextPoint = i;
			return true;
		}
	}

	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	
	normalTime = time - controlPoints[nextPoint-1].time;
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	
	float a, b, c, d; //Blending functions
	a = (2*pow(normalTime,3) / pow(intervalTime,3)) - (3*pow(normalTime,2) / pow(intervalTime,2)) + 1;
	b = (-2*pow(normalTime,3) / pow(intervalTime,3)) + (3*pow(normalTime,2) / pow(intervalTime,2));
	c = (pow(normalTime,3) / pow(intervalTime,2)) + (-2*pow(normalTime,2) / intervalTime) + normalTime;
	d = (pow(normalTime,3) / pow(intervalTime,2)) - (pow(normalTime,2) / intervalTime);

	float sx, sy, sz; //start point
	float nx, ny, nz; //next point
	sx = controlPoints[nextPoint-1].position.x;
	sy = controlPoints[nextPoint-1].position.y;
	sz = controlPoints[nextPoint-1].position.z;
	nx = controlPoints[nextPoint].position.x;
        ny = controlPoints[nextPoint].position.y;
        nz = controlPoints[nextPoint].position.z;

	float stx, sty, stz; //start tangent
	float ntx, nty, ntz; //next tangent
	stx = controlPoints[nextPoint-1].tangent.x;
	sty = controlPoints[nextPoint-1].tangent.y;
        stz = controlPoints[nextPoint-1].tangent.z;
        ntx = controlPoints[nextPoint].tangent.x;
        nty = controlPoints[nextPoint].tangent.y;
        ntz = controlPoints[nextPoint].tangent.z;


	newPosition.x = sx*a + nx*b + stx*c + ntx*d;
	newPosition.y = sy*a + ny*b + sty*c + nty*d;
	newPosition.z = sz*a + nz*b + stz*c + ntz*d;

	//std::cout << "Time: " << time << "    " << newPosition.y << " " << controlPoints[nextPoint-1].position.y << " " << controlPoints[nextPoint].position.y << std::endl;

	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Hermite curve

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	
	normalTime = time - controlPoints[nextPoint-1].time;
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	
	float a, b, c, d; //Blending functions
	a = (2*pow(normalTime,3) / pow(intervalTime,3)) - (3*pow(normalTime,2) / pow(intervalTime,2)) + 1;
	b = (-2*pow(normalTime,3) / pow(intervalTime,3)) + (3*pow(normalTime,2) / pow(intervalTime,2));
	c = (pow(normalTime,3) / pow(intervalTime,2)) + (-2*pow(normalTime,2) / intervalTime) + normalTime;
	d = (pow(normalTime,3) / pow(intervalTime,2)) - (pow(normalTime,2) / intervalTime);

	float sx, sy, sz; //start point
	float nx, ny, nz; //next point
	sx = controlPoints[nextPoint-1].position.x;
	sy = controlPoints[nextPoint-1].position.y;
	sz = controlPoints[nextPoint-1].position.z;
	nx = controlPoints[nextPoint].position.x;
        ny = controlPoints[nextPoint].position.y;
        nz = controlPoints[nextPoint].position.z;

	Vector sTangent, nTangent; //calculated tangents
	float stx, sty, stz; //start tangent components
	float ntx, nty, ntz; //next tangent components
	sTangent = getCatmullTangent(nextPoint-1);
	nTangent = getCatmullTangent(nextPoint);
	stx = sTangent.x;
	sty = sTangent.y;
	stz = sTangent.z;
	ntx = nTangent.x;
	nty = nTangent.y;
	ntz = nTangent.z;
	
	newPosition.x = sx*a + nx*b + stx*c + ntx*d;
	newPosition.y = sy*a + ny*b + sty*c + nty*d;
	newPosition.z = sz*a + nz*b + stz*c + ntz*d;
	//std::cout << time << std::endl;
	//std::cout << nextPoint << std::endl;
	//std::cout << newPosition << std::endl;

	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Catmull-Rom curve
	
	// Return result
	return newPosition;
}

Vector Curve::getCatmullTangent(const unsigned int currPoint) {
	
	Vector newTangent;

	if(currPoint == 0) {	
		newTangent.x = getCatmullTangent1D(0, controlPoints[currPoint].position.x, controlPoints[currPoint+1].position.x, controlPoints[currPoint+2].position.x, controlPoints[currPoint].time, controlPoints[currPoint+1].time, controlPoints[currPoint+2].time);
		newTangent.y = getCatmullTangent1D(0, controlPoints[currPoint].position.y, controlPoints[currPoint+1].position.y, controlPoints[currPoint+2].position.y, controlPoints[currPoint].time, controlPoints[currPoint+1].time, controlPoints[currPoint+2].time);
		newTangent.z = getCatmullTangent1D(0, controlPoints[currPoint].position.z, controlPoints[currPoint+1].position.z, controlPoints[currPoint+2].position.z, controlPoints[currPoint].time, controlPoints[currPoint+1].time, controlPoints[currPoint+2].time);
		std::cout << "HERE" << std::endl;
		return newTangent;
	}

	else if(currPoint == controlPoints.size()-1) {
		newTangent.x = getCatmullTangent1D(2, controlPoints[currPoint-2].position.x, controlPoints[currPoint-1].position.x, controlPoints[currPoint].position.x, controlPoints[currPoint-2].time, controlPoints[currPoint-1].time, controlPoints[currPoint].time);
		newTangent.y = getCatmullTangent1D(2, controlPoints[currPoint-2].position.y, controlPoints[currPoint-1].position.y, controlPoints[currPoint].position.y, controlPoints[currPoint-2].time, controlPoints[currPoint-1].time, controlPoints[currPoint].time);
		newTangent.z = getCatmullTangent1D(2, controlPoints[currPoint-2].position.z, controlPoints[currPoint-1].position.z, controlPoints[currPoint].position.z, controlPoints[currPoint-2].time, controlPoints[currPoint-1].time, controlPoints[currPoint].time);
		return newTangent;
	}

	newTangent.x = getCatmullTangent1D(1, controlPoints[currPoint-1].position.x, controlPoints[currPoint].position.x, controlPoints[currPoint+1].position.x, controlPoints[currPoint-1].time, controlPoints[currPoint].time, controlPoints[currPoint+1].time);
	newTangent.y = getCatmullTangent1D(1, controlPoints[currPoint-1].position.y, controlPoints[currPoint].position.y, controlPoints[currPoint+1].position.y, controlPoints[currPoint-1].time, controlPoints[currPoint].time, controlPoints[currPoint+1].time);
	newTangent.z = getCatmullTangent1D(1, controlPoints[currPoint-1].position.z, controlPoints[currPoint].position.z, controlPoints[currPoint+1].position.z, controlPoints[currPoint-1].time, controlPoints[currPoint].time, controlPoints[currPoint+1].time);

	return newTangent;	
}

//type = 0 for 1st point, 1 for normal, 2 for last point
float Curve::getCatmullTangent1D(int type, float prevPos, float currPos, float nextPos, float prevTime, float currTime, float nextTime) {

	if(type==0) {
		return ((nextTime-prevTime)/(nextTime-currTime)) * ((currPos-prevPos)/(currTime-prevTime)) - ((currTime-prevTime)/(nextTime-currTime)) * ((nextPos-prevPos)/(nextTime-prevTime));
	}

	else if(type==2) {
		return ((nextTime-prevTime)/(currTime-prevTime)) * ((nextPos-currPos)/(nextTime-currTime)) - ((nextTime-currTime)/(currTime-prevTime)) * ((nextPos-prevPos)/(nextTime-prevTime));
	}

	else {
		return ((currTime-prevTime)/(nextTime-prevTime)) * ((nextPos-currPos)/(nextTime-currTime)) + ((nextTime-currTime)/(nextTime-prevTime)) * ((currPos-prevPos)/(currTime-prevTime));
	}
}
