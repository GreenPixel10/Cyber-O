#include "HeightMapBuilder.h"

HeightMapBuilder::HeightMapBuilder(){
	cdt = CDT::Triangulation<double>(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::TryResolve, 3);
}


void HeightMapBuilder::load_contours(std::vector<LineFeature *> contours_){
	//contours = contours_;
	for (auto& c : contours_) {
		simple_contours.push_back(new simpleContour(c));
	}

}

void HeightMapBuilder::build() {

	process_raw_contours();
	triangulate();
	calculate_slopes();
	generate_confidence_graph();
	bottleneck();

	simpleContour* origin = simple_contours[0];
	origin->elevation = 0;
	origin->propagate_elevation();

	std::cout << "\n";

	int lowest = INT_MAX;
	int highest = INT_MIN;

	for (auto & c : simple_contours) {

		lowest = std::min(lowest, c->elevation);
		

	}


	for (auto & c : simple_contours) {

		c->elevation += (- lowest);
		highest = std::max(highest, c->elevation);
	}

	float col_scale = 255.0f/(float)highest;

	for (auto & c : simple_contours) {
		ofColor height = ofColor(c->elevation * col_scale, 175, c->elevation * col_scale);
		c->contour->set_colour(height);
	}
	
}

void HeightMapBuilder::process_raw_contours() {

	int id = 0;
	for (auto & c : simple_contours) {
		ofPolyline line = c->contour->get_line();
		for (int i = 0; i < line.size(); i++) {


			glm::vec2 last;
			glm::vec2 next;
			glm::vec2 p = line[i];

			if (i == 0) {
				if (c->contour->get_closed()) {
					last = line[line.size() - 1];
				}
				else {
					last = p;
				}
			}
			else {
				last = line[i - 1];
			}



			if (i == line.size() - 1) {
				if (c->contour->get_closed()) {
					next = line[0];
				} else {
					next = p;
				}
			}
			else {
				next = line[i+1];
			}


			demp * new_demp = new demp(p, c);
			new_demp->calculate_slope_ranges(last, next);

			demps.push_back(new_demp);

			if (i > 0) {
				constrained_edges.push_back(new demedge(demps.size() - 1, demps.size() - 2));
			}
		}
		id++;
	}
}

void HeightMapBuilder::triangulate() {

	
	
	cdt.insertVertices(demps.begin(), demps.end(),
		[](const demp* p) { return p->pos.x; },
		[](const demp* p){ return p->pos.y; }
	);

	
	cdt.insertEdges(
		constrained_edges.begin(),
		constrained_edges.end(),
		[](const demedge * e) { return e->vertices.first; },
		[](const demedge * e) { return e->vertices.second; }
	);
	
	
	cdt.eraseSuperTriangle();

	//auto tris = cdt.triangles;
	//auto verts = cdt.vertices;
	//auto bounds = cdt.fixedEdges;

	edges = CDT::extractEdgesFromTriangles(cdt.triangles);



	for (auto & e : edges) {
		CDT::VertInd i1 = e.v1();
		CDT::VertInd i2 = e.v2();

		if (i1 >= demps.size() || i2 >= demps.size()) {
			continue; //extra vertices from crossed edges
		}


		demp * d1 = demps[i1];
		demp * d2 = demps[i2];

		if (d1->contour == d2->contour) {
			//continue; //skip self-connections?
		}

		demedge* new_edge = new demedge(i1, i2, d1, d2);
		
		tri_edges.push_back(new_edge);

		d1->connections.push_back(new_edge);
		d2->connections.push_back(new_edge);
		
	}
}

void HeightMapBuilder::calculate_slopes() {
	for (auto & e : tri_edges) {
		glm::vec2 p1 = e->v1->pos;
		glm::vec2 p2 = e->v2->pos;

		if (e->v1->contour == e->v2->contour) {
			continue;
		}

		glm::vec2 v12 = p2 - p1;
		glm::vec2 v21 = p1 - p2;

		int slope1 = e->v1->slope_direction_by_vector(v12);
		int slope2 = e->v2->slope_direction_by_vector(v21);


		if (slope1 == slope2) {
			continue;
		}

		e->slope = slope1;
	}
}

void HeightMapBuilder::generate_confidence_graph() {
	for (auto & c : simple_contours) {
		for (auto & d : demps) {
			for (auto & e : d->connections) {
				if (e->v1->contour != c && e->v2->contour != c) {
					continue;
				}

				if (e->v1->contour == c && e->v2->contour == c) {
					continue;
				}

				

				simpleContour * connected_to = nullptr;
				int slope = 0;
				if (e->v1->contour == c) {
					connected_to = e->v2->contour;
					slope = e->slope;
				}
				if (e->v2->contour == c) {
					connected_to = e->v1->contour;
					slope = -(e->slope);
				}

				link * new_link = c->get_link_by_contour(connected_to);
				if (new_link == nullptr) {
					c->links.push_back(new link(connected_to, 1, slope));
					//std::cout << c->contour->get_debug() << " " << connected_to->contour->get_debug() << "\n";
				} else {
					new_link->slope += slope;
					new_link->confidence++;
				}
			}
		}
	}

	/*
	for (auto & c : simple_contours) {
		std::sort(c->links.begin(), c->links.end(), [](auto & left, auto & right) {
			return abs(left->confidence) > abs(right->confidence);
		});
	}
	*/

	/*
	std::cout << simple_contours[0]->contour->get_debug() << "\n";
	for (auto & test : simple_contours[0]->links) {
		std::cout << test->link_to->contour->get_debug() << " " << test->confidence << " " << test->slope << "\n";
	}
	*/
}

void HeightMapBuilder::bottleneck() {



	simpleContour * source = simple_contours[0];
	source->confidence_distance = INT_MAX;
	while (true) {
		simpleContour * current = nullptr;
		for (auto & find_nearest : simple_contours) {
			bool is_unvisited = !find_nearest->visited;
			bool is_closer = !current || (find_nearest->confidence_distance > current->confidence_distance);

			if (is_unvisited && is_closer) {
				current = find_nearest;
			}
		}
		if (!current) {
			break;
		} //done

		//std::cout << "Setting current to " << current->contour->get_debug() << "\n";

		for (auto & l : current->links) {
			if (l->link_to->visited) {
				continue;
			}
			//std::cout << "	testing " << l->link_to->contour->get_debug() << "\n";
			int con = current->confidence_distance;
			int next_con = std::min(con, l->confidence);
			//std::cout << "		Current: " << con << " Next:" << next_con << "\n";
			if (next_con > l->link_to->confidence_distance) {
				l->link_to->confidence_distance = next_con;
				l->link_to->prev = current;
				//std::cout << "			linked " << l->link_to->contour->get_debug() << " back to " << current->contour->get_debug() << "\n";
			}
		}

		current->visited = true;
	}


	//reverse links;
	for (auto & sc : simple_contours) {
		if (sc->prev) {
			sc->prev->next.push_back(sc);
		}
	}

	for (int p = 0; p < 5; p++) {

		for (auto & test : simple_contours[p]->next) {
			//std::cout << simple_contours[p]->contour->get_debug() << " " << test->contour->get_debug() << "\n";
		}
	}
	
}




void HeightMapBuilder::draw_triangulation() {

		ofSetColor(ofColor::purple);

		for (auto & d : demps) {
			
			ofDrawCircle(d->pos.x, d->pos.y, 150);
		}

		
		
		ofSetLineWidth(2);

		for (auto& te : tri_edges) {
			

			demp * d1 = te->v1;
			demp * d2 = te->v2;

			std::vector<ofColor> cols = {ofColor::red, ofColor::grey, ofColor::pink};
			
			ofSetColor(cols[te->slope+1]);


			//ofSetColor((te->slope)?ofColor::red : ofColor::green);
			ofDrawLine(d1->pos, d2->pos);
			
		}

}

demp::demp(glm::vec2 pos_, simpleContour * contour_): pos(pos_), contour(contour_), visited(false){
}

void demp::calculate_slope_ranges(glm::vec2 last_, glm::vec2 next_) {
	last = last_;
	next = next_;

	nextV = next - pos;
	lastV = last - pos;

	if (glm::distance2(last, pos) < 10) {lastV = -nextV;}
	if (glm::distance2(next, pos) < 10) {nextV = -lastV;}


}

int demp::slope_direction_by_vector(glm::vec2 vec) {



	glm::vec2 A = lastV;
	glm::vec2 N = vec;
	glm::vec2 B = nextV;

	float Aa = atan2(A.y, A.x) ;
	float Na = atan2(N.y, N.x);
	float Ba = atan2(B.y, B.x);


	if (Aa > Ba) {
		Aa -= TWO_PI;
	}
	
	bool inside = (Na > Aa && Na < Ba);

	Na -= TWO_PI;

	bool inside2 = (Na > Aa && Na < Ba);

	return (inside || inside2) ? 1 : -1;

	//-1: downhill
	//+1: uphill

}



demedge::demedge(std::size_t i1_, std::size_t i2_, demp* v1_, demp* v2_) {
	vertices.first = i1_;
	vertices.second = i2_;
	v1 = v1_;
	v2 = v2_;
	slope = 0;
}

void demp::propagate() {
	return;
	visited = true;
	for (auto& c : connections) {
		if (c->slope == 0) {
			continue;
		}

		if (c->v2 != this) {
			//c->v2->propagate();
			//c->v2->
		}
		if (c->v1 != this) {
			//c->v1->propagate();
		}

	}
}

simpleContour::simpleContour(LineFeature * lf):
	contour(lf), visited(false), confidence_distance(0), prev(nullptr), elevation(-6969){}

link* simpleContour::get_link_by_contour(simpleContour * target) {
	for (auto& l : links) {
		if (l->link_to == target) {
			return l;
		}
	}
	return nullptr;
}

link * simpleContour::get_best_unvisited_link() {
	for (auto& l : links) {
		if (!l->link_to->visited) {
			return l;
		}
	}
}

void simpleContour::propagate_elevation() {
	for (auto& next_contour : next) {


		link* link_with_slope_info = get_link_by_contour(next_contour);
		int raw_slope = link_with_slope_info->slope;
		int confidence = link_with_slope_info->confidence;
		simpleContour * link_to = link_with_slope_info->link_to;

		if (abs(raw_slope) <= confidence / 2) {
			link_to->elevation = elevation;
		}
		else if (raw_slope < 0) {
			link_to->elevation = elevation - 1;
		}
		else if (raw_slope > 0) {
			link_to->elevation = elevation + 1;
		}

		link_to->propagate_elevation();
	}
}

link::link(simpleContour * link_to_, int confidence_, int slope_): link_to(link_to_), confidence(confidence_), slope(slope_) {
}
