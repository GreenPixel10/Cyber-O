#include "SlopeDetector.h"

SlopeDetector::SlopeDetector() { }

void SlopeDetector::detect_slope() {
	slope_from_directional_points();
	apply_contour_leaners();
}

void SlopeDetector::slope_from_directional_points(){

	//for each point feature     v--point feature

	std::vector<int> points_to_compare = {S_MIN_CLIFF, S_SLOPE_TAG };
	for (int &point_symbol : points_to_compare){
		for (auto mc : (*features)[point_symbol]) {
			PointFeature* point_feature = dynamic_cast<PointFeature*>(mc); //whatever
			glm::vec3 mc_pos = glm::vec3(point_feature->get_pos(), 0);

			//for each line feature     v--line feature
			for (auto c : (*features)[S_CONTOUR]) {
				LineFeature * contour = dynamic_cast<LineFeature *>(c);
				ofPolyline line = contour->get_line();
				unsigned int ind;
				glm::vec3 closest = line.getClosestPoint(mc_pos, &ind);
				#define close 5 //metres
				int dist = glm::distance(closest, mc_pos);
				if (dist < close * 100) {
					int index = ind - 1;

					glm::vec2 segment_vec = line.getNormalAtIndex(ind);
					//std::cout << segment_vec.x << " " << segment_vec.y << "\n";

					float rot = -point_feature->get_rotation() + HALF_PI;
					glm::vec2 point_vec = { cos(rot), sin(rot) };


					//std::cout << point_vec.x << " " << point_vec.y << "\n";

					float alignment = glm::dot(glm::vec3(segment_vec, 0), glm::vec3(point_vec, 0));

					//correct slope!
					if (alignment > 0.5f) {
						contour->lean_slope_correct();
					}
					///wrong slope!
					if (alignment < -0.5f) {
						contour->lean_slope_wrong();
					}
 				
				}
			
			}
		}
	}
}

void SlopeDetector::apply_contour_leaners() {

	for (auto c : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(c);
		contour->lean_slope_apply();
	}
}
