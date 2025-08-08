
#include "PointFeature.h"


PointFeature::PointFeature(glm::vec2 pos_)
	: pos(pos_) {
	col = ofColor::green;
}

void PointFeature::construct_point(double scale, glm::vec2 offset) {
	symbol = glm::vec3();
	symbol.x = (pos.x + offset.x) * scale;
	symbol.y = (pos.y + offset.y) * scale;
	symbol.z = 1000 * scale;
	
}


void PointFeature::draw() {
	ofSetColor(col);
	ofDrawCircle(symbol.x, symbol.y, symbol.z);
}
