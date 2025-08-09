#pragma once
#include "Feature.h"
#include <string>
#include <vector>
#include <iostream>
#include "ofMain.h"


class Point {
	public:
		int x, y;
		bool null;
};

class LinePoint {

	public: 
		LinePoint(std::vector<std::string> p);

		Point pos;
		int pType;

};

class SplinePoint {
	public:
		SplinePoint(Point p, Point p_h, Point n_h);
		Point pos;
		Point p_handle;
		Point n_handle;


};

class LineFeature : public Feature {

	public:
		LineFeature();

		inline void add_point(LinePoint p) { points.push_back(p); }
		inline ofPolyline get_line() { return line;}
		inline bool get_closed() { return closed;}

		void construct_splines();
		void construct_polyline();
		void draw();

		
		Point min_coords;
		Point max_coords;

	private:
		std::vector<LinePoint> points; //raw data
		std::vector<SplinePoint> spline_points; //parsed spline data (3 points in each)

		bool closed;

		ofPolyline line; //draw line


};
