
#include "LineFeature.h"
#include "SymbolManager.h"

LinePoint::LinePoint(std::vector<std::string> p) {
	pos = { std::stoi(p[0]), std::stoi(p[1]), false };
	pType = p.size() == 3 ? std::stoi(p[2]) : -1;
}

SplinePoint::SplinePoint(Point p, Point p_h, Point n_h):
	pos(glm::vec2(p.x, p.y)), p_handle(glm::vec2(p_h.x, p_h.y)), n_handle(glm::vec2(n_h.x, n_h.y)) { }

LineFeature::LineFeature() {
	closed = false;
	col = ofColor::black;
	slope_verified = false;
	slope_leaner = 0;
	link_next = {};
	link_prev = {};
	all_links_gathered = false;
	//link_next_final = nullptr;
	//link_prev_final = nullptr;
	manual_link_end = nullptr;
	manual_link_start = nullptr;
	merge_tunnel = nullptr;
}

void LineFeature::init() {
	construct_splines();
}

bool LineFeature::get_closed_via_linked() {
	//LineFeature* next = link_next; 
	//while (true) {
		
	//}
	return false;
}

bool LineFeature::is_facing_outwards() {

	int s = line.size();
	double total = 0;

	for (int i = 0; i < s; i++) {

		int last_index = (i == 0) ? (s-1) : (i-1);
		int next_index = (i == s-1) ? (0) : (i + 1);
		
		glm::vec2 A = line[last_index];
		glm::vec2 B = line[i];
		glm::vec2 C = line[next_index];

		glm::vec2 BA = A - B;
		glm::vec2 BC = C - B;

		double dot = (BA.x * BC.x) + (BA.y * BC.y);
		double det = (BA.x * BC.y) - (BA.y * BC.x);     
		double angle = atan2(det, dot) ;
		angle = (angle * RAD_TO_DEG);
		total += angle;
	}

	return total > 0;
}



void LineFeature::set_slope_verified(bool is_slope_verified, bool recurse) {
	slope_verified = is_slope_verified;
	if (recurse) {
		//GO TO LINKS
	}

}

void LineFeature::lean_slope_apply() {


#define DRAW_SLOPE_FLIPS false


	if (slope_leaner == 0) { //not enough info
		return;
	}

	//otherwise, slope determined!
	set_slope_verified(true, true);
	//col = ofColor::green;

	//if the contour was not correct...
	if (slope_leaner < 0) {
		reverse_single_slope(); //flip it and all linked contours
	}

	slope_leaner = 0; //reset leaner (probably unneeded)
}

int LineFeature::get_length_at_point(glm::vec2 point) {
	auto vertices = line.getVertices();
	if(closed){vertices.push_back(vertices[0]);}

	int walk_total = 0;
	int step = 100;
	for (int i = 0; i < vertices.size() - 1; i++) {
		glm::vec2 A = vertices[i];
		glm::vec2 B = vertices[i+1];
		glm::vec2 vec = B-A;
		float len = glm::distance(A,B);
		for (int j = 0; j < len; j += step) {
			double p = j/len;
			glm::vec2 checkpoint = A + (p*vec);
			float dist = glm::distance(checkpoint, point);
			if (dist < step) {
				return walk_total + j;
			}
		}
		walk_total += len;
	}

	return -1;
}

void LineFeature::autoclose_almost_loop() {
	if (line.size() <= 2) { return;}
	//auto close looped features that don't quite meet
	float len = line.getLengthAtIndex(line.size()-1);
	#define AUTO_CLOSE_THRESHOLD 20 //metres
	#define AUTO_CLOSE_MINSIZE 35 // metres
	float d = glm::distance(line.getPointAtPercent(0), line.getPointAtPercent(100));
	if ((len > AUTO_CLOSE_MINSIZE * 100) && (d < AUTO_CLOSE_THRESHOLD * 100)) {
		closed = true;
	}
	//std::cout << "close check: " << d << " vs " << AUTO_CLOSE_THRESHOLD * 100 << "\n";

}


void LineFeature::clear_manual_link_start() {
	if (manual_link_start) {
		LineFeature * A = manual_link_start->A;
		LineFeature * B = manual_link_start->B;
		LineFeature* f2 = (A == this)?B:A; //find the other feature
		int f2_perc = (A == this) ? manual_link_start->Bperc : manual_link_start->Aperc; //find which end of the other feature is linked to this one

		if (f2_perc) { f2->manual_link_end = nullptr;} else{f2->manual_link_start = nullptr;} //remove that link reference for f2

		delete manual_link_start; //delete the link from the heap
		manual_link_start = nullptr; //clear the reference for this feature
	}
}

void LineFeature::clear_manual_link_end() {
	if (manual_link_end) {
		LineFeature * A = manual_link_end->A;
		LineFeature * B = manual_link_end->B;
		LineFeature * f2 = (A == this) ? B : A; //find the other feature
		int f2_perc = (A == this) ? manual_link_end->Bperc : manual_link_end->Aperc; //find which end of the other feature is linked to this one

		if (f2_perc) { //100, ie. end
			f2->manual_link_end = nullptr;
		} else {
			f2->manual_link_start = nullptr;
		} //remove that link reference for f2

		delete manual_link_end; //delete the link from the heap
		manual_link_end = nullptr; //clear the reference for this feature
	}
}


void LineFeature::construct_splines() {

	std::vector<Point> full_points; //includes dummies for non-existent handles (size is multiple of 3)

	for (int n = 0; n < (points.size() * 3); n++) {
		full_points.push_back(Point { NULL, NULL, true });
	}

	int num_points = 0;

	for (int i = 0; i < points.size(); i++) {

		LinePoint p = points[i];


		int next_index = (num_points * 3) + 1;

		switch (p.pType) {

			case (-1): //straight segment next
			case (16): //end of line, spline previous
			case (32): //unsure
				full_points[next_index] = p.pos; //just add point
				num_points++;
				break;

				
			case (33): //point with non-linked spline handles
			case (1): //two spline handles next
				{
					LinePoint p2 = points[i + 1];
					LinePoint p3 = points[i + 2];
					full_points[next_index] = p.pos; //add point
					full_points[next_index + 1] = p2.pos; //add next handle
					full_points[next_index + 2] = p3.pos; //add next handle
					num_points++;
					i += 2; //skip forward
					break;
				}

			case (2): //end, cycle 
			case (18): //end, cycle  (usure diff of 2 and 18)
				{
					Point p_last = full_points[next_index - 1]; //get p_handle of end point
					full_points[0] = p_last; //stick it to the beginning
					//DON'T increment point counter (closed loop flag gets set isntead)
					closed = true;
					break;
				}

			

			default:
				std::cout << "Unknown spline coordinate tag: " << p.pType << "\n";
				col = ofColor::red;


		}



		
	}

	max_coords = Point { INT_MIN, INT_MIN, false };
	min_coords = Point { INT_MAX, INT_MAX, false };

	//construct final spline point list
	for (int m = 0; m < num_points; m++) {
		int index = (m * 3) + 1;
		spline_points.push_back(SplinePoint(full_points[index], full_points[index - 1], full_points[index + 1]));

		//garbage bro
		if (full_points[index].x > max_coords.x) {max_coords.x = full_points[index].x;}
		if (full_points[index].y > max_coords.y) {max_coords.y = full_points[index].y;}
		if (full_points[index].x < min_coords.x) {min_coords.x = full_points[index].x;}
		if (full_points[index].y < min_coords.y) {min_coords.y = full_points[index].y;}
	}

	int len = 0;
	for (int i = 1; i < spline_points.size(); i++) {
		len += glm::distance(spline_points[i].pos, spline_points[i-1].pos);
	}





}

void LineFeature::construct_polyline() {
	line.clear();
	for (auto & sp : spline_points) {
		int x = sp.pos.x;
		int y = sp.pos.y;
		line.addVertex(x,y);
	}
	
}



void LineFeature::reverse_single_slope() {
	
	//std::cout << "Reversing " << debug << "...";
	std::reverse(line.begin(), line.end());
	//construct_polyline();
	std::swap(manual_link_end, manual_link_start);
	//std::cout << "1";
	if(manual_link_end){manual_link_end->reverse_feature(this);}
	//std::cout << "2";
	if(manual_link_start){manual_link_start->reverse_feature(this);}
	//std::cout << "3";
	if (DRAW_SLOPE_FLIPS) {col = ofColor::blue;}
	//std::cout << "done!\n";

}

int LineFeature::append_line(LineFeature * lf, bool after) {
	//std::cout << "Appending line:\n";

	//std::cout << ((lf) ? "true" : "false") << "uhoh\n";
	//std::cout << ((lf->merge_tunnel) ? "true" : "false") << "uhoh\n";
	while (lf->merge_tunnel) {
		//std::cout << "whee\n";
		lf = lf->merge_tunnel;

	}
	//std::cout << "0.5";
	
	if (line.size() == 0) { return 0;}

	//std::cout << "0.75";

	if (lf == this) {
		closed = true;
		//delete manual_link_end; breaks things, not sure why
		manual_link_end = nullptr;
		manual_link_start = nullptr;
		//std::cout << "Appended.\n";
		return 1;
	}

	//std::cout << "0.8";

	//std::cout << "appending " << lf->get_debug() << " to " << debug << "\n";

	glm::vec3 connection_point_A = line.getPointAtPercent(after?100:0);

	glm::vec3 lf_start = lf->get_line().getPointAtPercent(0);
	glm::vec3 lf_end = lf->get_line().getPointAtPercent(100);

	//std::cout << "1";

	bool closest_part_is_start = glm::distance2(connection_point_A, lf_start) < glm::distance2(connection_point_A, lf_end);

	if ((after && !closest_part_is_start) || (!after && closest_part_is_start)) {
		lf->reverse_single_slope();
	}

	//std::cout << "2";

	if (after) {
		line.addVertices(lf->get_line().getVertices());
		manual_link_end = lf->manual_link_end;
	}
	else {
		lf->get_line().addVertices(line.getVertices());
		line = lf->get_line();
		manual_link_start = lf->manual_link_start;
	}

	//std::cout << "3";

	lf->get_line().clear();
	//lf->clear_manual_link_end();
	//lf->clear_manual_link_start();
	lf->merge_tunnel = this;
	//anything referencing the now-empty feature willr eference the merged one instead

	//construct_polyline();
	//std::cout << " Appended.\n";
	return 1;

}




void LineFeature::draw(float zoom) {
	
	if (merge_tunnel) { return;}

	line.setClosed(closed);

	ofSetLineWidth(1);

	ofColor render_col = col;

	#define OVERRIDE_COLOUR true

	if (OVERRIDE_COLOUR) {
		render_col = ofColor::black;
	}


	#define HIGHLIGHT_UNVERIFIED false

	if (HIGHLIGHT_UNVERIFIED) {
		ofSetColor(get_slope_verified() ? render_col : ofColor::black);
		ofSetLineWidth(get_slope_verified() ? 5 : 1);
	}
	else {
		ofSetColor(render_col);
	}
	
	



	#define ISOLATE_UNVERIFIED false

	if (ISOLATE_UNVERIFIED && get_slope_verified()) {
		return;
	}
	
	line.draw();
	
	#define DRAW_TAGS true
	#define DRAW_TAGS_ALWAYS true
	if (DRAW_TAGS && (DRAW_TAGS_ALWAYS || get_slope_verified())) {
		for (int i = 1; i < line.size(); i++) {
			glm::vec2 p = line[i];
			glm::vec2 last_p = line[i - 1];
			glm::vec2 dir = p - last_p;
			glm::vec2 mid = (p+last_p)/2;
			dir = { -dir.y, dir.x };
			dir = glm::normalize(dir);
			dir *= 400;
			ofDrawLine(mid, mid+dir);
		}
	}

	int point_size = zoom * 8;

	#define DRAW_GAPS true
	if (DRAW_GAPS && closed == false) {
		

		glm::vec2 startpos;
		glm::vec2 endpos;

		ofSetLineWidth(5);

		//draw gaps based on auto bridger
		/*
		if (link_prev_final) {
			startpos = line[0];
			endpos = link_prev_point;
			ofDrawLine(startpos, endpos);
		}
		if (link_next_final) {
			startpos = line[line.size() - 1];
			endpos = link_next_point;
			ofDrawLine(startpos, endpos);
		}
		*/

		//draw gaps based on manual bridger
		
		if (manual_link_start) {
			ofSetColor(manual_link_start->colour);
			//std::cout << manual_link_start->Aperc << " " << manual_link_start->Bperc << "\n";
			//std::cout << (manual_link_start->A && manual_link_start->B ? "all good" : "uhoh!") << "\n";
			startpos = manual_link_start->A->get_line().getPointAtPercent(manual_link_start->Aperc);
			endpos = manual_link_start->B->get_line().getPointAtPercent(manual_link_start->Bperc);
			ofDrawLine(startpos, endpos);
		}
		if (manual_link_end) {
			ofSetColor(manual_link_end->colour);
			//std::cout << manual_link_end->Aperc << " " << manual_link_end->Bperc << "\n";
			//std::cout << (manual_link_end->A && manual_link_end->B ? "all good" : "uhoh!") << "\n";
			startpos = manual_link_end->A->get_line().getPointAtPercent(manual_link_end->Aperc);
			endpos = manual_link_end->B->get_line().getPointAtPercent(manual_link_end->Bperc);
			ofDrawLine(startpos, endpos);
		}

		//draw loose ends as red dots
		ofSetColor(ofColor::red);
		if (!manual_link_start) { ofDrawCircle(line[0].x, line[0].y, point_size);}
		if (!manual_link_end) { ofDrawCircle(line[line.size() - 1].x, line[line.size() - 1].y, point_size);}
	}

	#define DRAW_ENDPOINTS false
	if (DRAW_ENDPOINTS && line.size() >= 2 ) {
		ofSetColor(ofColor::green);
		ofDrawCircle(line[0].x, line[0].y, point_size);
		ofSetColor(ofColor::red);
		ofDrawCircle(line[line.size() - 1].x, line[line.size() - 1].y, point_size);
	}

	
	#define DRAW_LINKS true
	if (DRAW_LINKS) {
		ofSetColor(ofColor::cyan);

	}


}

GapLink::GapLink(LineFeature * to_, bool is_aligned_, int variance_): to(to_), is_aligned(is_aligned_), variance(variance_) {}

ManualLink::ManualLink(LineFeature * A_, int Aperc_, LineFeature * B_, int Bperc_, ofColor colour_) {
	A = A_;
	Aperc = Aperc_;
	B = B_;
	Bperc = Bperc_;
	colour = colour_;

	if (!A || !B) {
		std::cout << "Invalid Manual Link Created!!!" << "\n";
	} else {
		std::cout << "created manual link between " << A->get_debug() << " and " << B->get_debug() << "\n";
	}
}

void ManualLink::reverse_feature(LineFeature* f) {
	if (f == A) { Aperc = Aperc ? 0 : 100;}
	if (f == B) { Bperc = Bperc ? 0 : 100;}
}

LineFeature * ManualLink::get_other_end_from(LineFeature * f) {
	return (f == A) ? B : A;
}

