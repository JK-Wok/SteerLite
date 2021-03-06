//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * PolygonObstacle.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: gberseth
 */

#include "obstacles/PolygonObstacle.h"
#include "util/DrawLib.h"

PolygonObstacle::PolygonObstacle(std::vector<Util::Point> points, float traversalCost)
{
	// TODO Auto-generated constructor stub
	_traversalCost = traversalCost;
	_points = points;

    for ( size_t i = 0 ; i < _points.size() ; ++i )
    {
        _vectors.push_back( Util::Vector( _points[i].x, _points[i].y, _points[i].z ) );
    }

	_centerPosition = Util::Point(0,0,0);
	_radius = 0.0;
	float ymin = 0.0;
	float ymax = 1.0;
	float xmin, xmax, zmin, zmax;
	xmin = _points[0].x;
	xmax = _points[0].x;
	zmin = _points[0].z;
	zmax = _points[0].z;
	for(int i=1; i<_points.size(); i++) {
		if(_points[i].x > xmax)
			xmax = _points[i].x;
		if(_points[i].x < xmin)
			xmin = _points[i].x;
		if(_points[i].z > zmax)
			zmax = _points[i].z;
		if(_points[i].z < zmin)
			zmin = _points[i].z;
	}

	_bounds.ymin = ymin;
	_bounds.ymax = ymax;
	_bounds.xmin = xmin+(xmax-xmin)/8;
	_bounds.xmax = xmax-(xmax-xmin)/8;
	_bounds.zmin = zmin+(zmax-zmin)/8;
	_bounds.zmax = zmax-(zmax-zmin)/8;

	// TODO make parameter
	isConvex_ = true;

}

PolygonObstacle::~PolygonObstacle()
{
	// TODO Auto-generated destructor stub
}

void PolygonObstacle::draw()
{

	Util::Point height_dist(0.0,1.0,0.0);
	for (size_t _vert=0; _vert < this->_points.size()-1; _vert++)
	{
		Util::Point p0, p1;
		p0 = this->_points.at(_vert);
		p1 = this->_points.at(_vert+1);
		Util::DrawLib::drawLine(p0, p1, Util::gBlack, 2.0);
		Util::DrawLib::drawLine(p0+height_dist, p1+height_dist, Util::gBlack, 2.0);
		Util::DrawLib::drawLine(p0, p0+height_dist, Util::gBlack, 2.0);
		Util::DrawLib::drawQuad(p0, p1,	p1 + height_dist, p0 +height_dist,Util::gDarkMagenta);

	}


	if ( isConvex_ )
	{
		Util::Point p0, p1;
		p0 = this->_points.at(this->_points.size()-1);
		p1 = this->_points.at(0);
		Util::DrawLib::drawLine(p0, p1, Util::gBlack, 2.0);
		Util::DrawLib::drawLine(p0+height_dist, p1+height_dist, Util::gBlack, 2.0);
		Util::DrawLib::drawLine(p0, p0+height_dist, Util::gBlack, 2.0);
		Util::DrawLib::drawQuad(p0, p1,	p1 + height_dist, p0 +height_dist, Util::gDarkMagenta);
	}

}

std::pair<std::vector<Util::Point>,std::vector<size_t> > PolygonObstacle::getStaticGeometry()
{
	std::cout << "*****PolygonObstacle:: get static geometry not implemented yet" << std::endl;
	std::vector<Util::Point> ps;
	std::vector<size_t> vs;
	return std::make_pair(ps, vs);
}
