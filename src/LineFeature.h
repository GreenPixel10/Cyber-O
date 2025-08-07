#pragma once
#include "Feature.h"
#include <string>
#include <vector>

class LinePoint {

	public: 
		LinePoint(std::vector<std::string> p);

		int x, y;
		int pType;

};

class LineFeature : public Feature {

	public:
		LineFeature();

		inline void add_point(LinePoint p) { points.push_back(p); }

	private:
		std::vector<LinePoint> points;

};
