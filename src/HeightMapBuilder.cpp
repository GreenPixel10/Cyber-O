#include "HeightMapBuilder.h"

HeightMapBuilder::HeightMapBuilder(){}


void HeightMapBuilder::load_contours(std::vector<LineFeature *> contours_){
	contours = contours_;
}

void HeightMapBuilder::build() {

	int id = 0;
	for (auto & c : contours) {
		ofPolyline line = c->get_line();
		for (int i = 0; i < line.size(); i++ ) {

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

	triangulate();
	
	
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
}




void HeightMapBuilder::draw() {

		ofSetColor(ofColor::purple);

		for (auto & d : demps) {
			
			ofDrawCircle(d->pos.x, d->pos.y, 150);
		}

		
		
		ofSetLineWidth(2);

		for (auto& e : edges) {
			CDT::VertInd i1 = e.v1();
			CDT::VertInd i2 = e.v2();

			demp* d1 = demps[i1];
			demp* d2 = demps[i2];


			glm::vec2 p1 = d1->pos;
			glm::vec2 p2 = d2->pos;

			ofSetColor((d1->contour == d2->contour)?ofColor::red : ofColor::green);
			ofDrawLine(p1, p2);
			
		}

}

demp::demp(glm::vec2 pos_, LineFeature * contour_): pos(pos_), contour(contour_){
}

demedge::demedge(std::size_t v1_, std::size_t v2_) {
	vertices.first = v1_;
	vertices.second = v2_;
}
