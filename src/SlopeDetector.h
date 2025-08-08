#pragma once

#include <string>
#include <vector>
#include <map>
#include "ofMain.h"
#include "Feature.h"
#include <iostream>


class SlopeDetector {
	public:
		SlopeDetector();
		inline void set_features(std::map<int, std::vector<Feature *>> * features_) {features = features_; }

	private:
		std::map<int, std::vector<Feature *>>* features; 
};
