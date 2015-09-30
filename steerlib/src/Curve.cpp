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
	
	std::vector<Point> drawPoints;

	drawPoints.push_back(controlPoints[0].position);
	for(float t=0; t<=controlPoints[controlPoints.size()-1].time; t=t+window) {
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
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function checkRobust is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================


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

	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Hermite curve

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================


	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Catmull-Rom curve
	
	// Return result
	return newPosition;
}
