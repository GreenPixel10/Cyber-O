
#include "PointFeature.h"


PointFeature::PointFeature(glm::vec2 pos_, float rot_)
	: pos(pos_), rot(rot_){
	col = ofColor::green;
}

void PointFeature::init() {
	//
}

void PointFeature::construct_point() {
	symbol = glm::vec3();
	symbol.x = pos.x;
	symbol.y = pos.y;
	symbol.z = 500;
	
}


void PointFeature::draw(float zoom) {
	ofSetColor(col);
	ofDrawCircle(symbol.x, symbol.y, symbol.z);
}
