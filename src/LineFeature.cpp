
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
	linked_flag = false;
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

void LineFeature::align_linked() {


	set_linked_flag(true);

	//std::cout << "aligning " << debug << "\n";


	for (auto &n : link_next) {
		//std::cout << (n.second == true) << (n.first->linked_flag == false) << "\n";
		if (n.first && n.first != this && n.second == true && n.first && !n.first->linked_flag) {
			n.first->reverse_slope(this, this, n.second);
			n.second = false;
			//std::cout << debug << "is now aligned with " << n.first->get_debug() << "\n";
		}
	}
	///does not get backt o it!
	for (auto & n : link_prev) {
		//std::cout << (n.second == true) << (n.first->linked_flag == false) << "\n";
		if (n.first && n.first != this && n.second == true && !n.first->linked_flag) {
			n.first->reverse_slope(this, this, n.second);
			n.second = false;
			//std::cout << debug << "is now aligned with " << n.first->get_debug() << "\n";

		}
	}

	//std::cout << "-----------------\n";

}

void LineFeature::lean_slope_apply() {


#define DRAW_SLOPE_FLIPS true


	if (slope_leaner == 0) { //not enough info
		return;
	}

	//otherwise, slope determined!
	set_slope_verified(true);
	col = ofColor::green;

	//if the contour was not correct...
	if (slope_leaner < 0) {
		reverse_slope(this, nullptr, false); //flip it and all linked contours
		if (DRAW_SLOPE_FLIPS) {col = ofColor::blue;}
	}

	slope_leaner = 0; //reset leaner (probably unneeded)
}

int LineFeature::get_length_at_point(glm::vec2 point) {
	auto vertices = line.getVertices();
	if(closed){vertices.push_back(vertices[0]);}

	int walk_total = 0;

	for (int i = 0; i < vertices.size() - 1; i++) {
		glm::vec2 A = vertices[i];
		glm::vec2 B = vertices[i+1];
		glm::vec2 vec = B-A;
		float len = glm::distance(A,B);
		for (int j = 0; j < len; j += 100) {
			double p = j/len;
			glm::vec2 checkpoint = A + (p*vec);
			float dist = glm::distance(checkpoint, point);
			if (dist < 100) {
				return walk_total + j;
			}
		}
		walk_total += len;
	}

	return -1;
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

	//auto close looped contours that don't quite meet
	if (S_CODE == S_CONTOUR) {
		#define CLOSE_THRESHOLD 20 //metres
		float d = glm::distance(spline_points.front().pos, spline_points.back().pos);
		if (d < CLOSE_THRESHOLD * 100) {
			closed = true;
		}
	}



}

void LineFeature::construct_polyline() {
	line.clear();
	for (auto & sp : spline_points) {
		int x = sp.pos.x;
		int y = sp.pos.y;
		line.addVertex(x,y);
	}
	line.setClosed(closed);
}

void LineFeature::reverse_slope(LineFeature* origin, LineFeature* last, int lazy_depth) {

	
	//debug stuff
	std::string gap = "";
	for(int i = 0; i <= lazy_depth; i++) {
		gap += "   ";
	}
	//std::cout << gap << debug << "\n";


		bool towards_next = false;
		//which end to move towards
		for (auto &n : link_prev) {
			if (n.first == last) {
				towards_next = true;
			}
		}

		for (auto & n : link_prev) { //DUP
			
			if (n.first == origin) {
				n.second = !n.second;
				//std::cout << "inverting backlink ---";
				n.first->flip_link_to(this);
		
			}
		}

		for (auto & n : link_next) { 

			if (n.first == origin) {
				n.second = !n.second;
				//std::cout << "inverting backlink ---";
				n.first->flip_link_to(this);
			}
		}

	

	set_linked_flag(true);

	std::reverse(spline_points.begin(), spline_points.end());
	construct_polyline();
	//std::cout << gap << "reversing " << debug << "\n";

	if (lazy_depth > 50) {
		//std::cout << "RECURSION\n";
		return;
	}


	
		for (auto n : link_prev) {
			if (n.first && !n.first->linked_flag) {
				n.first->reverse_slope(origin, this, lazy_depth + 1);
			}
		}
	


	
		for (auto & n : link_next) {
			if (n.first && !n.first->linked_flag) {
				n.first->reverse_slope(origin, this, lazy_depth + 1);
			}
		}







	

}


void LineFeature::draw() {
	ofSetLineWidth(get_slope_verified()?2:1);
	ofSetColor(col);
	line.draw();
	//std::cout << col << "\n";

	#define DRAW_ENDPOINTS true
	if (DRAW_ENDPOINTS) {
		ofSetColor(ofColor::green);
		ofDrawCircle(line[0].x, line[0].y, 300);
		ofSetColor(ofColor::red);
		ofDrawCircle(line[line.size() - 1].x, line[line.size() - 1].y, 300);
	}

	#define DRAW_LINKS false
	if (DRAW_LINKS) {
		ofSetColor(ofColor::cyan);

		for (auto& l : link_next) {
			ofDrawLine(line.getPointAtPercent(0), l.first->get_line().getPointAtPercent(100));
		}
		for (auto & l : link_prev) {
			ofDrawLine(line.getPointAtPercent(100), l.first->get_line().getPointAtPercent(0));
		}
		
	}

}

void LineFeature::flip_link_to(LineFeature * lf) {
	

	for (auto & n : link_prev) { //DUP

		if (n.first == lf) {
			n.second = !n.second;
			//std::cout << "--- inverting backlink\n";
		}
	}

	for (auto & n : link_next) {

		if (n.first == lf) {
			n.second = !n.second;
			//std::cout << "--- inverting backlink\n";
		}
	}
}

bool LineFeature::is_aligned() {
	for (auto & n : link_prev) { //DUP

		if (n.second) {
			return false;
		}
	}

	for (auto & n : link_next) {

		if (n.second) {
			return false;
		}
	}

	return true;

}
