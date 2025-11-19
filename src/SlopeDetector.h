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


		void cast_contours();
		void detect_slope();
		void set_debug_colours();
		void print_contour_amount(bool only_valid = false);
		void autoclose_almost_loops();
		void detect_contour_gaps();
		int auto_classify_gaps(bool unambigous_only = true);

		void detect_slope_2();
		void manual_gaps();
		void fill_gaps();
		void cleanup_deleted_contours();
		void slope_from_directional_points(); //eg. slope tags
		void slope_from_directional_linears(); //eg. long cliffs
		void slope_from_closed_loops(); //eg. hilltops
		void slope_from_similarity();

		void get_end_from_click(glm::vec2 pos, bool is_release);

		void apply_contour_leaners();

		int get_percent_verified();
		int get_num_unverified();

		int get_similarity(LineFeature* f1, LineFeature* f2);


		std::vector<LineFeature *> contours;

	private:
		std::map<int, std::vector<Feature *>>* features;

		std::pair<LineFeature*, bool> click_start;
		std::pair<LineFeature *, bool> click_end;

};
