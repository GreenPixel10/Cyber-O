
#include "PointFeature.h"


PointFeature::PointFeature(glm::vec2 pos_)
	: pos(pos_), rot(0){
	col = ofColor::green;
}

void PointFeature::construct_point() {
	symbol = glm::vec3();
	symbol.x = pos.x;
	symbol.y = pos.y;
	symbol.z = 500;
	
}


void PointFeature::draw() {
	ofSetColor(col);
	ofDrawCircle(symbol.x, symbol.y, symbol.z);
}
