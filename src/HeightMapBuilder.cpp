#include "HeightMapBuilder.h"

HeightMapBuilder::HeightMapBuilder(){
	cdt = CDT::Triangulation<double>(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::TryResolve, 3);
}


void HeightMapBuilder::load_contours(std::vector<LineFeature *> contours_){
	contours = contours_;
}

void HeightMapBuilder::build() {

	process_raw_contours();
	triangulate();


	for (auto& e : tri_edges) {
		glm::vec2 p1 = e->v1->pos;
		glm::vec2 p2 = e->v2->pos;

		if (e->v1->contour == e->v2->contour) {
			continue;
		}

		glm::vec2 v12 = p2 - p1;
		glm::vec2 v21 = p1 - p2;

		int slope1 = e->v1->slope_direction_by_vector(v12);
		int slope2 = e->v2->slope_direction_by_vector(v21);

		std::cout << slope1 << " " << slope2 << "\n";
		std::cout << e->v2->lastV << " " << e->v2->nextV << " " << v21 << "\n\n";

		if (slope1 == slope2) {
			continue;
		}

		e->slope = slope1;

	}

	//int dir = demps[1]->slope_direction_by_vector(demps[3]->pos - demps[1]->pos);
	//std::cout << "Direction: " << dir << "\n";
	
	
}

void HeightMapBuilder::process_raw_contours() {

	int id = 0;
	for (auto & c : contours) {
		ofPolyline line = c->get_line();
		for (int i = 0; i < line.size(); i++) {


			glm::vec2 last;
			glm::vec2 next;
			glm::vec2 p = line[i];

			if (i == 0) {
				if (c->get_closed()) {
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
				if (c->get_closed()) {
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
			continue; //skip self-connections?
		}

		tri_edges.push_back(new demedge(i1, i2, d1, d2));
		
	}
}




void HeightMapBuilder::draw() {

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

demp::demp(glm::vec2 pos_, LineFeature * contour_): pos(pos_), contour(contour_){
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

	//std::cout << A << N << B << "\n";

	double AxN = cross2(A, N);
	double AxB = cross2(A, B);
	double NxB = cross2(N, B);
	double BxN = cross2(B, N);
	double BxA = cross2(B, A);

	if (AxB < 0) {
		if (AxN < 0 && NxB < 0){return 1;}
	}
	else{
		if (AxN > 0 && NxB > 0){return 1;}
	}

	return -1;

	//return (AxN * AxB >= 0 && BxN * BxA >= 0) ? 1 : -1;



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
