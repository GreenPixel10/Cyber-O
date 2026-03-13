#include "SlopeDetector.h"
#include <algorithm>
#include <random>


SlopeDetector::SlopeDetector() {
	click_start = {nullptr, false };
	click_end = { nullptr, false };
}





void SlopeDetector::prepass_gaps() {


	/* setup */
	cast_contours(); //convert features to linefeatures
	print_contour_amount(); //initial contour amount
	set_debug_colours(); //set colours and names
	autoclose_almost_loops(); //first autoloop pass

	/* contour merging */
	detect_contour_gaps(); //find potential gaps
	auto_classify_gaps(); //sort potential gaps by liklihood and ambiguity





}

void SlopeDetector::manage_gaps() {
	fill_gaps(); //connect and merge linked contours
	cleanup_deleted_contours(); //clean up redirect contours left by merge
	cast_contours(); //regenerate contour list
	autoclose_almost_loops(); //second autoloop pass
	print_contour_amount(); //contour count after merges
}

void SlopeDetector::auto_detect_slope() {


	/* slope detetction */
	std::cout << "Auto-detecting slopes:\n";
	slope_from_directional_points(); //slope from eg. slope tags
	slope_from_directional_linears(); //slope from eg. cliffs
	apply_contour_leaners(); //confirm slope absed on liklihoods
	slope_from_closed_loops(); //assume remaining closed loops are hills
	slope_from_similarity(); //assume parallel implies same slope

	std::cout << get_percent_verified() << "% of contours verified\n";
	std::cout << get_num_unverified() << " contours could not be verified\n";
	std::cout << "\n";

}

void SlopeDetector::manual_detect_slope() {
}

void SlopeDetector::cast_contours() {
	contours.clear();
	for (auto & f : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(f);
		contours.push_back(contour);
	}
}


void SlopeDetector::set_debug_colours() {
	std::vector<ofColor> cols = { ofColor::yellow, ofColor::orange, ofColor::red, ofColor::purple, ofColor::blue, ofColor::green, ofColor::magenta, ofColor::turquoise, ofColor::indigo};
	std::vector<std::string> colnames = { "yellow", "orange", "red", "purple", "blue", "green", "magenta", "turquoise", "indigo" };
	int numcols = cols.size();

	for (int i = 0; i < (*features)[S_CONTOUR].size(); i++) {

		auto f = (*features)[S_CONTOUR][i];
		f->set_colour(cols[i % numcols]);
		//f->set_debug(colnames[i % numcols]);
	}
}

void SlopeDetector::print_contour_amount(bool only_valid) {
	if (!only_valid) {
		std::cout << "" << contours.size() << " contours exist\n";
	}
	else {
		int t = 0;
		for (auto & c : contours) {
			if (!c->merge_tunnel) {
				t++;
			}
		}
		std::cout << "" << t << " valid contours exist\n";
	}
}

void SlopeDetector::autoclose_almost_loops() {
	for (auto& c : contours) {
		c->autoclose_almost_loop();
	}
}

void SlopeDetector::detect_contour_gaps() {


	#define DEBUG_GAP_REJECTION false

	for (auto cA : (*features)[S_CONTOUR]) {
		LineFeature * contourA = dynamic_cast<LineFeature *>(cA);

		if (contourA->get_closed()) {
			if (DEBUG_GAP_REJECTION) std::cout << "cloased A\n";
			continue;
		}

		ofPolyline Aline = contourA->get_line();
		int A_end_index = Aline.size() - 1;

		glm::vec2 cA_s = Aline.getPointAtIndexInterpolated(0);
		glm::vec2 cA_s2 = Aline.getPointAtIndexInterpolated(1);
		glm::vec2 cA_svec = glm::normalize(cA_s - cA_s2);

		glm::vec2 cA_e = Aline.getPointAtIndexInterpolated(A_end_index);
		glm::vec2 cA_e2 = Aline.getPointAtIndexInterpolated(A_end_index - 1);
		glm::vec2 cA_evec = glm::normalize(cA_e - cA_e2);

		for (auto cB : (*features)[S_CONTOUR]) {
			bool loop = false;
			if (cB == cA) {
				loop = true;
			}
			LineFeature * contourB = dynamic_cast<LineFeature *>(cB);

			if (contourB->get_closed()) {
				if (DEBUG_GAP_REJECTION) std::cout << "closed B\n";
				continue;
			}

			ofPolyline Bline = contourB->get_line();
			int B_end_index = Bline.size() - 1;
			glm::vec2 cB_s = Bline.getPointAtIndexInterpolated(0);
			glm::vec2 cB_s2 = Bline.getPointAtIndexInterpolated(1);
			glm::vec2 cB_svec = glm::normalize(cB_s - cB_s2);

			glm::vec2 cB_e = Bline.getPointAtIndexInterpolated(B_end_index);
			glm::vec2 cB_e2 = Bline.getPointAtIndexInterpolated(B_end_index - 1);
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

			for (int i = 0; i < 2; i++) {

				int A_side = 1-i;
				int B_side;

				if (A_side == _START) {
					B_side = (ss_dist < se_dist) ? _START : _END;
				}
				if (A_side == _END) {
					B_side = (es_dist < ee_dist) ? _START : _END;
				}
				if (loop) {
					if (cA->some_bullshit_gap_flag) {
						B_side = _END;
						A_side = _START;
					}
					else {
						B_side = _START;
						A_side = _END;
					}

					cA->some_bullshit_gap_flag = true;
				}

				bool aligned = (A_side != B_side); //aligned if A and B are connected at different ends

				//get endpoints
				glm::vec2 P = A_side == _START ? cA_s : cA_e;
				glm::vec2 Q = B_side == _START ? cB_s : cB_e;

				int dist = glm::distance(P, Q);

				//skip if too far apart
				#define CONTOUR_GAP_CLOSE 35 //metres
				if (dist > CONTOUR_GAP_CLOSE * 100) {
					if (DEBUG_GAP_REJECTION) std::cout << "too wide\n";
					continue;
				}

				//get rough endpoint extension vectors
				glm::vec2 Pv = A_side == _START ? cA_svec : cA_evec;
				glm::vec2 Qv = B_side == _START ? cB_svec : cB_evec;

				glm::vec2 Pex = P + (Pv * (dist / 2));
				glm::vec2 Qex = Q + (Qv * (dist / 2));

				#define CONTOUR_ANGLE_THRESHOLD -0.4f //
				#define CONTOUR_TOUCHING_THRESHOLD 1 //metre
				float dot = glm::dot(Pv, Qv);
				if (dot > CONTOUR_ANGLE_THRESHOLD && dist > CONTOUR_TOUCHING_THRESHOLD * 100) {
					if (DEBUG_GAP_REJECTION) std::cout << "bad angle\n"; //unfortunately nessesary to avoid parallel contours linking up
					if(!loop){continue;}
				}

				glm::vec2 ideal_midpoint = (Q + P) / 2;

				int variance = std::max(glm::distance(Pex, ideal_midpoint), glm::distance(Qex, ideal_midpoint));

				#define CONTOUR_GAP_VARIANCE_THRESHOLD 20 //metres

				float distance_factor = (float)dist / ((float)CONTOUR_GAP_CLOSE * 100);
				float variance_multiplier = 1.0f - distance_factor;

				float var_thresh = (CONTOUR_GAP_VARIANCE_THRESHOLD * 100) * variance_multiplier;

				if (variance < var_thresh) {

				
					GapLink * gl = new GapLink(contourB, aligned, variance * distance_factor);

					if (A_side == _START) {		
							contourA->link_prev.push_back(gl);	
					}
					if (A_side == _END) {
							contourA->link_next.push_back(gl);	
					}

				} else {
					if (DEBUG_GAP_REJECTION) std::cout << variance << " > " << variance_multiplier * (CONTOUR_GAP_VARIANCE_THRESHOLD * 100) << " bad variance\n";
				}
			}
		}
	}

	//SORT connections by variance (bad to good)
	for (auto & c : contours) {
		std::sort(c->link_next.begin(), c->link_next.end(), [](auto & left, auto & right) {
			return abs(left->variance) > abs(right->variance);
		});
		std::sort(c->link_prev.begin(), c->link_prev.end(), [](auto & left, auto & right) {
			return abs(left->variance) > abs(right->variance);
		});
		int last_index;

		#define SO_GOOD_BONUS 0.7f // if the best connection is <.75 of the second best, just say its right idec

		last_index = c->link_next.size() - 1;
		if (last_index >= 1 && (c->link_next[last_index]->variance <= (c->link_next[last_index - 1]->variance * SO_GOOD_BONUS))) {
			//c->link_next.clear(); #def a memory leak w/o this but idk why this crashes it
			c->link_next = { c->link_next[last_index]};
		}
		last_index = c->link_prev.size() - 1;
		if (last_index >= 1 && (c->link_prev[last_index]->variance <= (c->link_prev[last_index - 1]->variance * SO_GOOD_BONUS))) {
			//c->link_prev.clear();
			c->link_prev = { c->link_prev[last_index] };
		}

	}

	

	

}



int SlopeDetector::auto_classify_gaps(bool unambigous_only) {

	std::cout << "Auto detecting gaps... ";

	int unambigous_count = 0;
	int semiambigous_count = 0;

	for (auto& c : contours) {


		std::vector<std::vector<GapLink *> *> linksets = {&(c->link_next), &(c->link_prev) };

		for (int i = 0; i < 2; i++) { //FIRST: A->  THEN: <-A


			std::vector<GapLink *> * forwardlinks = linksets[i];
			
			if (unambigous_only && forwardlinks->size() > 1) { continue;}

			if (!forwardlinks->empty()) {
				
				GapLink * best_single_gaplink = nullptr;
				std::vector<GapLink *> * backlinks;
		

				for (auto & gl : *forwardlinks) { 

		

					//set backlinks to B-> or <-B based on alignment / which end of A
					if (i == 0) {
						backlinks = gl->is_aligned ? &(gl->to->link_prev) : &(gl->to->link_next);
					}
					if (i == 1) {
						backlinks = gl->is_aligned ? &(gl->to->link_next) : &(gl->to->link_prev);
					}

					
					if (backlinks->size() == 1) {
						best_single_gaplink = gl;
					}
	
				
				}

				if (unambigous_only && backlinks->size() > 1) {continue;}

				//priotitize singly linked ones, if there are none just skip
				if (!best_single_gaplink) { continue; }
				
				//std::cout << "Linking " << c->get_debug() << " with " << best_single_gaplink->to->get_debug() << "\n";
				
				if (i == 0) {
					//c->link_next_final = best_single_gaplink->to;
					//c->link_next_point = c->link_next_final->get_line().getPointAtPercent(best_single_gaplink->is_aligned?0:100);
					create_manual_link(c, true, best_single_gaplink->to, best_single_gaplink->is_aligned ? false : true);

				}
				if (i == 1) {
					//c->link_prev_final = best_single_gaplink->to;
					//c->link_prev_point = c->link_prev_final->get_line().getPointAtPercent(best_single_gaplink->is_aligned ? 100 : 0);
					create_manual_link(c, false, best_single_gaplink->to, best_single_gaplink->is_aligned ? true : false);
				}


				if (forwardlinks->size() == 1 && backlinks->size() == 1) {unambigous_count++;}
				else{semiambigous_count++;}

				forwardlinks->clear();
				backlinks->clear();
				


			}
		}




	}

	std::cout << unambigous_count << " gaps detected";
	if(!unambigous_only) {std::cout << " plus " << semiambigous_count << "potential gaps";}
	std::cout << "\n";

	return unambigous_count + semiambigous_count;
}

void SlopeDetector::manual_gaps() {
	
	if (click_start.first && click_end.first) {
		std::cout << "click between " << click_start.first->get_debug() << " and " << click_end.first->get_debug() << "\n";


		LineFeature * f1 = click_start.first;
		LineFeature * f2 = click_end.first;

		bool is_end_of_f1 = click_start.second;
		bool is_start_of_f1 = !click_start.second;
		bool is_end_of_f2 = click_end.second;
		bool is_start_of_f2 = !click_end.second;

		create_manual_link(f1, is_end_of_f1, f2, is_end_of_f2, ofColor::lime);

		click_start = { nullptr, false };
		click_end = { nullptr, false };
	}
	
}

void SlopeDetector::create_manual_link(LineFeature * f1, bool is_end_of_f1, LineFeature * f2, bool is_end_of_f2, ofColor colour) {

	ManualLink * new_link = new ManualLink(f1, is_end_of_f1 * 100, f2, is_end_of_f2 * 100, colour);

	if (!is_end_of_f1) {
		f1->clear_manual_link_start();
		f1->manual_link_start = new_link;
	} else { //if end of LF1
		f1->clear_manual_link_end();
		f1->manual_link_end = new_link;
	}

	if (!is_end_of_f2) {
		f2->clear_manual_link_start();
		f2->manual_link_start = new_link;
	} else { //if end of LF2
		f2->clear_manual_link_end();
		f2->manual_link_end = new_link;
	}

}

void SlopeDetector::fill_gaps() {
	std::cout << "Filling gaps... ";
	int count = -1;
	while (count != 0) {
		count = 0;
		for (auto & c : contours) {
			if (c->manual_link_end) {
				count += c->append_line(c->manual_link_end->get_other_end_from(c), true);
			}
			if (c->manual_link_start) {
				count += c->append_line(c->manual_link_start->get_other_end_from(c), false);
			}
		}
	}
	std::cout << " Gap filling complete\n";
}

void SlopeDetector::cleanup_deleted_contours() {
	for (int i = ((*features)[S_CONTOUR]).size() - 1; i >= 0; i--) {
		LineFeature * contour = dynamic_cast<LineFeature *>(((*features)[S_CONTOUR])[i]);
		if (contour->merge_tunnel) {
			((*features)[S_CONTOUR]).erase(((*features)[S_CONTOUR]).begin() + i);
		}
	}
}







void SlopeDetector::slope_from_directional_points(){
	std::cout << "Analyzing point features... ";
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
	std::cout << " Done\n";
}

void SlopeDetector::slope_from_directional_linears() {
	std::cout << "Analyzing line features... ";
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
	std::cout << " Done\n";
}

void SlopeDetector::slope_from_closed_loops() {
	std::cout << "Analyzing loops... ";
	for (auto c : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(c);

		if (contour->get_slope_verified()) { continue;}
		if (contour->get_closed()) {

			if (!contour->is_facing_outwards()) {
				contour->reverse_single_slope();
			}
			
			contour->set_slope_verified(true, true); //recursion needed?
			//contour->set_colour(ofColor::purple);
			}
		
	}
	std::cout << " Done\n";
}

void SlopeDetector::slope_from_similarity() {
//return;
//std::cout << (*features)[S_CONTOUR].size() << " <<\n";
	std::cout << "Detecting remaining slopes via similarity:\n";

#define SIMILARITY_LENGTH_THRESHOLD 175 //metres

	int verified_count = -1;

	int pass_number = 1;

	while (verified_count != 0) {

		std::cout << "    Pass " << pass_number << ": ";

		verified_count = 0;

		for (auto cA : (*features)[S_CONTOUR]) {
			LineFeature * contourA = dynamic_cast<LineFeature *>(cA);
			if (contourA->get_slope_verified()) {
				//std::cout << cA->get_debug() << " is already verified\n";
				continue;
			} //contour has already been matched

			ofPolyline line = contourA->get_line();

			for (auto cB : (*features)[S_CONTOUR]) {

				if (cB == cA) {
					//std::cout << "Skipping " << cA->get_debug() << " vs " << cB->get_debug() << ": Same Contour " << "\n ";
					continue;
				} //same contour

				LineFeature * contourB = dynamic_cast<LineFeature *>(cB);

				if (contourB->get_slope_verified() == false) {
					//std::cout << "Skipping " << cA->get_debug() << " vs " << cB->get_debug() << ": " << cB->get_debug() << " is not verified." << "\n ";
					continue;
				} //other contour is not confirmed yet

				int similarity = get_similarity(contourB, contourA);
				bool needs_flip = similarity < 0;
				similarity = abs(similarity);


				int contourA_metre_length = (int)((line.getLengthAtIndex(line.size() - 1)) / 100); //surely better way
				int threshold = std::min(SIMILARITY_LENGTH_THRESHOLD, contourA_metre_length) * 0.75;
				
				if (similarity >= threshold) {
					contourA->set_slope_verified(true, true);
					//contourA->set_colour(ofColor::cyan);
					if (needs_flip) { contourA->reverse_single_slope();}
					verified_count++;
				}
			}
		}

		std::cout << "Verified " << verified_count << " contours via similarity\n";
		pass_number++;
	}
}

void SlopeDetector::get_end_from_click(glm::vec2 pos, bool is_release) {

	bool found = false;
	bool end_is_next = false;
	LineFeature* selection = nullptr;
	
	for (auto& c : contours) {
		
		glm::vec2 start = c->get_line().getPointAtPercent(0);
		glm::vec2 end = c->get_line().getPointAtPercent(100);

		#define CLICK_CLOSE 5 //metres
		int s_distance = glm::distance(pos, start);
		int e_distance = glm::distance(pos, end);
		bool s_close = s_distance < ((CLICK_CLOSE * 100));
		bool e_close = e_distance < ((CLICK_CLOSE * 100));
		if (s_close) {
			found = true;
			end_is_next = false;
			selection = c;
			break;
		}
		if (e_close) {
			found = true;
			end_is_next = true;
			selection = c;
			break;
		}
	}

	if (!found) {
		std::cout << "resetting\n";
		if (is_release) { //if nothing found on the end click, clear everything (invalid gap)
			click_start = {nullptr, false };
			click_end = { nullptr, false };
		}
		return;
	}


	if (!is_release) { //click (gap start)
		click_start = {selection, end_is_next};
		std::cout << "valid click\n";
		std::cout << selection << "\n";
	}
	
	if (is_release) { //release (gap end)
		click_end = { selection, end_is_next };
		std::cout << "valid release\n";
		std::cout << selection << "\n";
	}


}

void SlopeDetector::apply_contour_leaners() {

	for (auto c : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(c);
		if (contour->get_slope_verified()) { continue;}
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

int SlopeDetector::get_num_unverified() {
	int unverified = 0;
	for (auto c : (*features)[S_CONTOUR]) {
		LineFeature * contour = dynamic_cast<LineFeature *>(c);
		if (!contour->get_slope_verified()) {
			unverified++;
		}
	}
	return unverified;
}


int SlopeDetector::get_similarity(LineFeature * f1, LineFeature * f2) {
	std::vector<int> nearness;

	int overall_nearness_length = 0;

	int direction = 0;
	int last_pos_along = -1;

	ofPolyline l1 = f1->get_line();
	ofPolyline l2 = f2->get_line();

	#define SIMILAR_THRESHOLD 40 //metres

	for (glm::vec3 & p1 : l1) {
		glm::vec3 p2 = l2.getClosestPoint(p1);
		int dist = glm::distance(p1, p2);
		nearness.push_back(dist / 100);

		int pos_along = f2->get_length_at_point(p2);
		if (last_pos_along != -1) { direction += pos_along - last_pos_along;}
		last_pos_along = pos_along;


	}

	for (int i = 1; i < nearness.size(); i++) {
		
		bool similar_segment = (nearness[i] <= SIMILAR_THRESHOLD && nearness[i-1] <= SIMILAR_THRESHOLD);

		if (similar_segment) {
			int A = l1.getLengthAtIndex(i-1);
			int B = l1.getLengthAtIndex(i);
			overall_nearness_length += (abs(B-A))/100; //should always be positive anyways
		}
	}

	//std::cout << f1->get_debug() << " to " << f2->get_debug() << ": " << overall_nearness_length << "m of similarity\n";

	
	int needs_flip = direction < 0 ? -1:1;

	return overall_nearness_length * needs_flip;

}
