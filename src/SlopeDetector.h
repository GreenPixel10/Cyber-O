#pragma once

#include <string>
#include <vector>
#include <map>
#include "ofMain.h"
#include "Feature.h"
#include "PointFeature.h"
#include "LineFeature.h"
#include "SymbolManager.h"
#include <iostream>


#define POINT_CLOSE_TO_CONTOUR 5 //metres
#define LINE_CLOSE_TO_CONTOUR 7.5 //metres

class SlopeDetector {
	public:
		SlopeDetector();
		inline void set_features(std::map<int, std::vector<Feature *>> * features_) {features = features_; }

		void detect_slope();
		void repair_contours();
		void slope_from_directional_points(); //eg. slope tags
		void slope_from_directional_linears(); //eg. long cliffs
		void slope_from_closed_loops(); //eg. hilltops

		void apply_contour_leaners();

		int get_percent_verified();

	private:
		std::map<int, std::vector<Feature *>>* features; 
};
