#include "HeightMapBuilder.h"

HeightMapBuilder::HeightMapBuilder(){
	cdt = CDT::Triangulation<double>(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::TryResolve, 1);
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

	traverse_graph();

	normalize_elevations();


	int count = 0;
	for (auto & a : cdt.fixedEdges) {
		for (auto & b : tri_edges) {

			if (b->v1->contour != b->v2->contour) { continue;} //cant be a contour line if both ends are diff contours
			if (a.v1() >= demps.size() || a.v2() >= demps.size()) { continue;} //invalid overlaps

			if ((demps[a.v1()] == b->v1 && demps[a.v2()] == b->v2) || (demps[a.v1()] == b->v2 && demps[a.v2()] == b->v1)) {
				//count++;
				b->shape = 0;
			}

			ofPolyline line = b->v1->contour->contour->get_line();
			glm::vec2 line_start = line.getPointAtPercent(0);
			glm::vec2 line_end = line.getPointAtPercent(100);

			glm::vec2 start = b->v1->pos;
			glm::vec2 end = b->v2->pos;

			//if ((line_start == start && line_end == end) || (line_start == end && line_end == start)) {
				//count++;
			//}

			//if (b->v1->pos == b->v1->contour->contour->get_line().getPointAtPercent(0).
		}
	}
	std::cout << constrained_edges.size() << " - " << tri_edges.size() << "\n";
	std::cout << "COUNT--- " << count << "\n";


	generate_mesh();


	
}

void HeightMapBuilder::process_raw_contours() {

	int id = 0;
	for (auto & c : simple_contours) {
	
		ofPolyline line = c->contour->get_line();
		std::cout << line.size() << "\n ";

		demp* last_demp = nullptr;
		demp* start_demp = nullptr;

		for (int i = 0; i < line.size(); i++) {
			//std::cout << i << "-----<\n";

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

			demp* exists = nullptr;

			for (auto& d : demps) {
				if (d->get_x() == p.x && d->get_y() == p.y) {
					exists = d; //don't think this really ever happens?
					break;
				}
			}

			demp * new_demp;

			if (exists) { new_demp = exists;}
			else {
				new_demp = new demp(p, c);
				new_demp->calculate_slope_ranges(last, next);
				new_demp->ID = demps.size();
				demps.push_back(new_demp);
			}


			if (i == 0) {
				start_demp = new_demp;
			}

			if (i > 0) {
				constrained_edges.push_back(new demedge(new_demp->ID, last_demp->ID));
				
				std::cout << "adding from " << new_demp->ID << " to " << last_demp->ID << "\n";
			}

			//if the loop is closed also add the last edge
			if (c->contour->get_closed() && i == line.size() - 1) {
				
				constrained_edges.push_back(new demedge(new_demp->ID, start_demp->ID));
				std::cout << "adding from " << new_demp->ID << " to " << start_demp->ID << "\n";
				std::cout << glm::distance(new_demp->pos, start_demp->pos) << "------dist\n";
			}

			last_demp = new_demp;
		}
		id++;
	}
}

void HeightMapBuilder::triangulate() {




	std::cout << "inserting vertices\n";
	cdt.insertVertices(demps.begin(), demps.end(),
		[](const demp* p) { return p->pos.x; },
		[](const demp* p){ return p->pos.y; }
	);

	std::cout << "inserting edges\n";
	cdt.insertEdges(
		constrained_edges.begin(),
		constrained_edges.end(),
		[](const demedge * e) { return e->vertices.first; },
		[](const demedge * e) { return e->vertices.second; }
	);
	
	std::cout << "erasing supertriangle\n";
	cdt.eraseSuperTriangle();

	//auto tris = cdt.triangles;
	//auto verts = cdt.vertices;
	//auto bounds = cdt.fixedEdges;

	std::cout << "extracting edges\n";
	edges = CDT::extractEdgesFromTriangles(cdt.triangles);


	std::cout << "gen connections\n";
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
	std::cout << "calculate slopes\n";
	for (auto & e : tri_edges) {
		glm::vec2 p1 = e->v1->pos;
		glm::vec2 p2 = e->v2->pos;

		if (e->v1->contour == e->v2->contour) {
			//continue;
		}

		glm::vec2 v12 = p2 - p1;
		glm::vec2 v21 = p1 - p2;

		int slope1 = e->v1->slope_direction_by_vector(v12);
		int slope2 = e->v2->slope_direction_by_vector(v21);


		if (slope1 == slope2) {

			e->shape = -slope1; //bump or a valley
			/*
			if (e->v1->contour == e->v2->contour) { //edge might have been part of an original contour (should not have bump or valley)
				LineFeature * c = e->v1->contour->contour;
				int i1 = c->get_index_at_point(p1); //these are not working
				int i2 = c->get_index_at_point(p2);

				bool beside = abs(i1 - i2) <= 1;
				std::cout << i1 << " " << i2 << "\n";
				bool ends = (i1 == 0 && i2 == c->get_line().size()) || (i2 == 0 && i1 == c->get_line().size());

				if (beside || ends) {
					e->shape = 2; //contour
				}
			}
			*/
			continue;
		}

		e->slope = slope1;
		
	}
}

void HeightMapBuilder::generate_confidence_graph() {
	std::cout << "gen confidence graph\n";
	for (auto & c : simple_contours) { //for every contour
		for (auto & d : demps) { //for every point
			for (auto & e : d->connections) { //for every edge connection to that point

				
				if (e->v1->contour != c && e->v2->contour != c) { //if both ends of the edge aren't on c, skip
					continue;	//(not relavant connection)
				}

				if (e->v1->contour == c && e->v2->contour == c) { //if both ends of the edge ARE on c, skip
					continue; //(both on the same contour, must be 0 slope)
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

				//std::cout << "SLOPE: " << slope << "\n";

				link * new_link = c->get_link_by_contour(connected_to);
				if (new_link == nullptr) {
					c->links.push_back(new link(connected_to, 1, slope));
					//std::cout << c->contour->get_debug() << " " << connected_to->contour->get_debug() << " " << slope << "\n";
				} else {
					new_link->slope += slope;
					new_link->confidence++;
					//std::cout << c->contour->get_debug() << " to " << connected_to->contour->get_debug() << " modified to " << new_link->slope << "\n";
				}
			}
		}
	}

	std::cout << "sort confidence conenctions\n";
	for (auto & c : simple_contours) {
		std::sort(c->links.begin(), c->links.end(), [](auto & left, auto & right) {
			return abs(left->confidence) > abs(right->confidence);
		});
	}
	

	/*
	std::cout << simple_contours[0]->contour->get_debug() << "\n";
	for (auto & test : simple_contours[0]->links) {
		std::cout << test->link_to->contour->get_debug() << " " << test->confidence << " " << test->slope << "\n";
	}
	*/
}

simpleContour * HeightMapBuilder::bottleneck() {

	std::cout << "bottleneck pathfinding\n";
	simpleContour * source = nullptr;

	for (auto & find_strongest : simple_contours) {
		if (!source) {
			source = find_strongest;
			continue;
		}

		link * potential_link = find_strongest->get_best_unvisited_link();
		link * strongest_link = source->get_best_unvisited_link();

		if (!strongest_link) {
			source = find_strongest;
			continue;
		}

		if (!potential_link) {
			continue;
		}

		if (potential_link->confidence > strongest_link->confidence) {
			source = find_strongest;
		}
	}
	if (!source) {
		std::cout << "uhoh2 - no source found for bottleneck pathfinding?\n";
		return nullptr;
	}
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

	std::cout << "reversing links to generate a tree\n";

	//reverse links;
	for (auto & sc : simple_contours) {
		if (sc->prev) {
			sc->prev->next.push_back(sc);
		}
	}


	return source;
	
	
}


void HeightMapBuilder::traverse_graph() {
	simpleContour * origin = bottleneck();

	std::cout << "propagate\n";

	//simpleContour* origin = simple_contours[50];
	origin->elevation = 0;
	origin->propagate_elevation();

	std::cout << "\n";
}

void HeightMapBuilder::normalize_elevations() {

	//find highet and lowest in graph

	int lowest = INT_MAX;
	int highest = INT_MIN;

	int count = 0;
	for (auto & c : simple_contours) {
		if (c->elevation == -6969) {
			count++;
			continue;
		}
		lowest = std::min(lowest, c->elevation);
		highest = std::max(highest, c->elevation);
	}


	std::cout << "error: " << count << "\n";
	std::cout << lowest << " -> " << highest << "\n";

	//adjust so lowest is at 0m

	highest = INT_MIN;
	for (auto & c : simple_contours) {

		c->elevation += (-lowest);
		highest = std::max(highest, c->elevation);
	}


	//apply colours

	float col_scale = 255.0f / (float)highest;
	std::cout << highest << "m\n";
	for (auto & c : simple_contours) {
		if (c->elevation == -6969) {
			/////////////////////////c->contour->set_colour(ofColor::red);
			continue;
		}
		ofColor height = ofColor(c->elevation * col_scale, 175, c->elevation * col_scale);
		/////////////////c->contour->set_colour(height);
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

			std::vector<ofColor> cols = {ofColor::red, ofColor::grey, ofColor::blue, ofColor::brown};
			
			ofSetColor(cols[te->shape+1]);
			//ofSetColor(te->v1->contour->contour->get_colour());

			//ofSetColor((te->slope)?ofColor::red : ofColor::green);
			ofDrawLine(d1->pos, d2->pos);
			
		}

}

void HeightMapBuilder::generate_mesh() {

	int original_num_points = demps.size();

	std::cout << "generating mesh:\n";

	std::ofstream mesh("D:/Projects/OrienteeringSim/models/mesh.obj");

	int shrink = 1000;


	//get offsets so mesh isnt crazy far away
	double min_x = 99999999999999;
	double min_y = 99999999999999;

	for (auto & v : demps) {
		if (v->get_x() < min_x) { min_x = v->get_x();}
		if (v->get_y() < min_y) { min_y = v->get_y();}
	}

	//write all vertices into .obj
	int ind = 1;
	for (auto& v : demps) {
		v->obj_vertex_index = ind;
		ind++;

		double x = (v->get_x() - min_x) / shrink;
		double y = (v->get_y() - min_y) / shrink;
		int elevation = v->contour->elevation;

		mesh << "v " << x << " " << elevation << " " << y << "\n";
	}
	int bad = 0;

	std::vector<demp*> mesh_vertices = demps;
	std::vector<string> faces_buffer;

	//write all faces into .obj
	for (auto & t : cdt.triangles) {
		CDT::VertInd i1 = t.vertices[0];
		CDT::VertInd i2 = t.vertices[1];
		CDT::VertInd i3 = t.vertices[2];

		//std::cout << i1 << " " << i2 << " " << i3;
		if (i1 >= original_num_points || i2 >= original_num_points || i3 >= original_num_points) {
			//std::cout << "...aborted\n";
			bad++;
			continue;
		}
		demp * v1 = mesh_vertices[i1];
		demp * v2 = mesh_vertices[i2];
		demp * v3 = mesh_vertices[i3];

		glm::vec3 A = glm::vec3(v1->pos, v1->contour->elevation);
		glm::vec3 B = glm::vec3(v2->pos, v2->contour->elevation);
		glm::vec3 C = glm::vec3(v3->pos, v3->contour->elevation);

		glm::vec3 D = (A + B) / 2;
		glm::vec3 E = (B + C) / 2;
		glm::vec3 F = (C + A) / 2;



		if (A.z == B.z && B.z == C.z) {

			int doff = 0;
			int eoff = 0;
			int foff = 0;

			for (auto & e : tri_edges) {
				if ((e->v1 == v1 && e->v2 == v2) || (e->v1 == v2 && e->v2 == v1)) {
					doff = e->shape;
				}
				if ((e->v1 == v2 && e->v2 == v3) || (e->v1 == v3 && e->v2 == v2)) {
					eoff = e->shape;
				}
				if ((e->v1 == v3 && e->v2 == v1) || (e->v1 == v1 && e->v2 == v3)) {
					foff = e->shape;
				}
			}
			float mult = 0.5;
			//std::cout << doff << " " << eoff << " " << foff << "\n";
			if(true){D.z -= doff * mult;}
			if(true){E.z -= eoff * mult;}
			if(true){F.z -= foff * mult;}

		}

		mesh << "v " << (D.x - min_x) / shrink << " " << D.z << " " << (D.y - min_y) / shrink << "\n";
		mesh << "v " << (E.x - min_x) / shrink << " " << E.z << " " << (E.y - min_y) / shrink << "\n";
		mesh << "v " << (F.x - min_x) / shrink << " " << F.z << " " << (F.y - min_y) / shrink << "\n";

		//std::cout << "...done\n";

		string ABC = "f " + to_string(v1->obj_vertex_index) + " " + to_string(v2->obj_vertex_index) + " " + to_string(v3->obj_vertex_index) + "\n";

		string ADF = "f " + to_string(v1->obj_vertex_index) + " " + to_string(ind) + " " + to_string(ind + 2) + "\n";
		string BED = "f " + to_string(v2->obj_vertex_index) + " " + to_string(ind + 1) + " " + to_string(ind) + "\n";
		string CFE = "f " + to_string(v3->obj_vertex_index) + " " + to_string(ind + 2) + " " + to_string(ind + 1) + "\n";
		string DEF = "f " + to_string(ind) + " " + to_string(ind + 1) + " " + to_string(ind + 2) + "\n";

		#define SUBDIV true
		if (SUBDIV) {
			faces_buffer.push_back(ADF);
			faces_buffer.push_back(BED);
			faces_buffer.push_back(CFE);
			faces_buffer.push_back(DEF);
		} else {
			faces_buffer.push_back(ABC);
		}

		ind += 3;


		//mesh << "f " << v1->obj_vertex_index << " " << v2->obj_vertex_index << " " << v3->obj_vertex_index << "\n";


	}


	for (auto& f : faces_buffer) {
		mesh << f;
	}

	std::cout << bad << " out of " << cdt.triangles.size() << " triangles were ignored (invalid vertex index)\n";
	//caused by overlap
	
	mesh.close();
}

void HeightMapBuilder::draw_DEM() {

	

	for (auto & d : demps) {
	
		ofColor c = d->contour->contour->get_colour();
		ofSetColor(c);
		ofDrawCircle(d->pos.x, d->pos.y, 5000);
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
	return nullptr;
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
