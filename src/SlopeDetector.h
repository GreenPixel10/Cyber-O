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


class SlopeDetector {
	public:
		SlopeDetector();
		inline void set_features(std::map<int, std::vector<Feature *>> * features_) {features = features_; }

		void detect_slope();
		void slope_from_directional_points();
		void apply_contour_leaners();
	private:
		std::map<int, std::vector<Feature *>>* features; 
};
