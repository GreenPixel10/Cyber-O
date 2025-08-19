#include "HeightMapBuilder.h"

HeightMapBuilder::HeightMapBuilder(){}


void HeightMapBuilder::load_contours(std::vector<LineFeature *> contours_){
	contours = contours_;
}

void HeightMapBuilder::build() {
	
}


void HeightMapBuilder::draw() {
	for (auto & c : contours) {
		for (auto & p : c->get_line()) {
			ofSetColor(ofColor::purple);
			ofDrawCircle(p.x, p.y, 500);
		}
	}
}
