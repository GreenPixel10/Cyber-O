#pragma once
#include "Feature.h"
#include <string>
#include <vector>
#include <iostream>


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

		void construct_splines();

	private:
		std::vector<LinePoint> points;
		std::vector<SplinePoint> spline_points;

};
