#include "SlopeDetector.h"

SlopeDetector::SlopeDetector() { }

void SlopeDetector::detect_slope() {

	repair_contours();


	for (auto &f : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(f);
		contour->align_linked();
		reset_contour_link_flags();
	}



	return;
	slope_from_directional_points();
	apply_contour_leaners();
	reset_contour_link_flags();
	slope_from_directional_linears();
	apply_contour_leaners();
	reset_contour_link_flags();
	slope_from_closed_loops();
	std::cout << get_percent_verified() << "% of contours verified\n";


}


void SlopeDetector::reset_contour_link_flags() {
	for (auto & f : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(f);
		contour->set_linked_flag(false);
	}
}

void SlopeDetector::repair_contours() {
	for (auto cA : (*features)[S_CONTOUR]) {
		LineFeature * contourA = dynamic_cast<LineFeature *>(cA);
		ofPolyline Aline = contourA->get_line();
		int A_end_index = Aline.size() - 1;

		glm::vec2 cA_s = Aline.getPointAtIndexInterpolated(0);
		glm::vec2 cA_s2 = Aline.getPointAtIndexInterpolated(1);
		glm::vec2 cA_svec = glm::normalize(cA_s - cA_s2);

		glm::vec2 cA_e = Aline.getPointAtIndexInterpolated(A_end_index);
		glm::vec2 cA_e2 = Aline.getPointAtIndexInterpolated(A_end_index-1);
		glm::vec2 cA_evec = glm::normalize(cA_e - cA_e2);



		for (auto cB : (*features)[S_CONTOUR]) {
			if (cB == cA) { continue;}
			LineFeature * contourB = dynamic_cast<LineFeature *>(cB);
			ofPolyline Bline = contourB->get_line();
			int B_end_index = Bline.size() - 1;
			glm::vec2 cB_s = Bline.getPointAtIndexInterpolated(0);
			glm::vec2 cB_s2 = Bline.getPointAtIndexInterpolated(1);
			glm::vec2 cB_svec = glm::normalize(cB_s - cB_s2);

			glm::vec2 cB_e = Bline.getPointAtIndexInterpolated(B_end_index);
			glm::vec2 cB_e2 = Bline.getPointAtIndexInterpolated(B_end_index-1);
			glm::vec2 cB_evec = glm::normalize(cB_e - cB_e2);

			//for each of the possible 4 connections
			//check if they're close


			//distances between points
			int ss_dist = glm::distance(cA_s, cB_s);
			int se_dist = glm::distance(cA_s, cB_e);
			int es_dist = glm::distance(cA_e, cB_s);
			int ee_dist = glm::distance(cA_e, cB_e);

			#define _START 1
			#define _END 0


			int A_side = (std::min(ss_dist, se_dist) < std::min(es_dist, ee_dist)) ? _START : _END;
			int B_side;

			if (A_side == _START) {B_side = (ss_dist < se_dist)?_START:_END; }
			if (A_side == _END) {B_side = (es_dist < ee_dist)?_START:_END; }

			//std::cout << A_side << " " << B_side << "\n";

			bool aligned = !(A_side == B_side); //if both a and b are the same end, they aren't aligned

			//get endpoints
			glm::vec2 P = A_side == _START? cA_s : cA_e;
			glm::vec2 Q = B_side == _START? cB_s : cB_e;

			
			int dist = glm::distance(P, Q);

			//skip if too far apart
			#define CONTOUR_GAP_CLOSE 50 //metres
			if (dist > CONTOUR_GAP_CLOSE * 100) {
				continue;
			}

			//get rough endpoint extension vectors
			glm::vec2 Pv = A_side == _START ? cA_svec : cA_evec;
			glm::vec2 Qv = B_side == _START ? cB_svec : cB_evec;


			glm::vec2 Pex = P + (Pv * (dist / 2));
			glm::vec2 Qex = Q + (Qv * (dist / 2));

			#define CONTOUR_ANGLE_THRESHOLD -0.4f //
			float dot = glm::dot(Pv, Qv);
			if (dot > CONTOUR_ANGLE_THRESHOLD) { continue;}


			glm::vec2 ideal_midpoint = (Q + P) / 2;


			int variance = std::max(glm::distance(Pex, ideal_midpoint), glm::distance(Qex, ideal_midpoint));
			//std::cout << "variance " << (float)variance / 100 << "\n";

			

			#define CONTOUR_GAP_VARIANCE_THRESHOLD 10 //metres

			if (variance < CONTOUR_GAP_VARIANCE_THRESHOLD * 100) {
				
				if (A_side == _START) {
					contourA->add_link_prev({ contourB, !aligned });
					std::cout << "linking!\n";
				}
				if (A_side == _END) {
					contourA->add_link_next({ contourB, !aligned });
					std::cout << "linking!\n";
				}
					
				contourA->set_colour(ofColor::green);

			}
			




		}
	}
		
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
