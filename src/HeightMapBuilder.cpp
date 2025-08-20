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
	
	
}

void HeightMapBuilder::process_raw_contours() {

	int id = 0;
	for (auto & c : contours) {
		ofPolyline line = c->get_line();
		for (int i = 0; i < line.size(); i++) {

			glm::vec2 p = line[i];

			demp * new_demp = new demp(p, c);
			//demp * last = demps.back();

			demps.push_back(new_demp);

			if (i > 0) {
				constrained_edges.push_back(new demedge(demps.size() - 1, demps.size() - 2));
				//std::cout << "edge from " << demps.size() - 1 << " to " << demps.size() - 2 << "\n";
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

			ofSetColor((d1->contour == d2->contour)?ofColor::red : ofColor::green);
			ofDrawLine(d1->pos, d2->pos);
			
		}

}

demp::demp(glm::vec2 pos_, LineFeature * contour_): pos(pos_), contour(contour_){
}

demedge::demedge(std::size_t i1_, std::size_t i2_, demp* v1_, demp* v2_) {
	vertices.first = i1_;
	vertices.second = i2_;
	v1 = v1_;
	v2 = v2_;
}
