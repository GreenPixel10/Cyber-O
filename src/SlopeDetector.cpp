#include "SlopeDetector.h"

SlopeDetector::SlopeDetector() { }

void SlopeDetector::detect_slope() {
	slope_from_directional_points();
	apply_contour_leaners();
	slope_from_directional_linears();
	apply_contour_leaners();
	slope_from_closed_loops();
	std::cout << get_percent_verified() << "% of contours verified\n";
}


void SlopeDetector::slope_from_directional_points(){

	std::vector<int> points_to_compare = {S_MIN_CLIFF, S_SLOPE_TAG, S_IMPASSABLE_MIN_CLIFF};

	for (int &point_symbol : points_to_compare){
		for (auto mc : (*features)[point_symbol]) {
			PointFeature* point_feature = dynamic_cast<PointFeature*>(mc); //whatever
			glm::vec3 point_pos = glm::vec3(point_feature->get_pos(), 0);


			for (auto c : (*features)[S_CONTOUR]) {
				LineFeature * contour = dynamic_cast<LineFeature *>(c);
				if (contour->get_slope_verified()) { continue;}
				ofPolyline line = contour->get_line();

				//find where on the contour the point is closest to
				unsigned int ind;
				glm::vec3 closest = line.getClosestPoint(point_pos, &ind);
				
				int dist = glm::distance(closest, point_pos);

				//if the point is on or near the contour:
				if (dist < POINT_CLOSE_TO_CONTOUR * 100) {
					int index = ind;

					//get vector of uncertain slope from contour
					glm::vec2 segment_vec = line.getNormalAtIndex(ind);

					//get vector of certain slope from point feature
					float rot = -point_feature->get_rotation() + HALF_PI;
					glm::vec2 point_vec = { cos(rot), sin(rot) };

					//see if they match
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

void SlopeDetector::slope_from_directional_linears() {

	std::vector<int> linears_to_compare = { S_CLIFF, S_IMPASSABLE_CLIFF, S_EARTH_BANK};

	for (int & linear_symbol : linears_to_compare) {
		for (auto mc : (*features)[linear_symbol]) {
			LineFeature * line_feature = dynamic_cast<LineFeature *>(mc);
			ofPolyline lf_line = line_feature->get_line();

			for (auto c : (*features)[S_CONTOUR]) {
				LineFeature * contour = dynamic_cast<LineFeature *>(c);

				if (contour->get_slope_verified()) { continue;}

				ofPolyline contour_line = contour->get_line();

				bool touching = false;

				int last_length_along = -1;
				int traversal_net = 0;

				for (auto vertex : lf_line) { 
					glm::vec3 closest = contour_line.getClosestPoint(vertex);

					int dist = glm::distance(closest, vertex);

					int length_along = contour->get_length_at_point(closest); //where this vertex of the LF falls along the contour

					if (dist < LINE_CLOSE_TO_CONTOUR * 100) {touching = true;} //DOES NOT WORK FOR CLIFFS THAT CROSS CONTOURS
																			   //because none of the cliff vertices fall under the
																			   //distance threshold even though the segment crosses.
																			   //this is probably a rare edge case

					if (last_length_along != -1) { //if this isn't the first point
						traversal_net += length_along - last_length_along;
					}

					last_length_along = length_along;
					

				}

				#define FOLLOW_THRESHOLD 3 //how many metres LF must actually follow the contour to count

				if (touching) { 
					if (traversal_net > FOLLOW_THRESHOLD * 100) { //aligned
						contour->lean_slope_correct();
					}

					else if (traversal_net < -FOLLOW_THRESHOLD * 100) { //not aligned
						contour->lean_slope_wrong();
					}
				}


			}
		}
	}
}

void SlopeDetector::slope_from_closed_loops() {
	for (auto c : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(c);
		if (contour->get_slope_verified()) { continue;}
		if (contour->get_closed()) {
			contour->set_slope_verified(true);
			contour->set_colour(ofColor::purple);
			}
		
	}
}

void SlopeDetector::apply_contour_leaners() {

	for (auto c : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(c);
		contour->lean_slope_apply();
	}
}

int SlopeDetector::get_percent_verified() {
	int verified = 0;
	for (auto c : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(c);
		if (contour->get_slope_verified()) { verified++;}
	}
	return ((double)verified / (double)(((*features)[S_CONTOUR]).size())) * 100;
}
