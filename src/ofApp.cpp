#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {

	ofBackground(0, 0, 0);


	load_map();

	glm::vec2 max_coords = glm::vec2 (INT_MIN, INT_MIN);
	glm::vec2 min_coords = glm::vec2 (INT_MAX, INT_MAX);

	for (auto& c : contours) {
		if (c.max_coords.x > max_coords.x) {max_coords.x = c.max_coords.x;} //why did i not use built ins??
		if (c.max_coords.y > max_coords.y) {max_coords.y = c.max_coords.y;}
		if (c.min_coords.x < min_coords.x) {min_coords.x = c.min_coords.x;}
		if (c.min_coords.y < min_coords.y) {min_coords.y = c.min_coords.y;}
	}

	int margin = 10000;
	//max_coords += glm::vec2(margin, margin);
	//min_coords -= glm::vec2(margin, margin);

	offset = min_coords;

	scale = max_coords;
	scale -= min_coords;
	scale /= glm::vec2(1024, 768);
	scale *= 1.5;

	for (auto & c : contours) {
		c.construct_polyline(scale, offset);
	}
}

void ofApp::load_map() {

	std::string filename = "D:/Projects/OrienteeringSim/CampFortune.omap";
	//std::string filename = "D:/Projects/OrienteeringSim/test2.omap";

	if (!omap.load(filename)) {
		ofLogError() << "Couldn't load file";
	}

	auto basemap = omap.getChild("map");
	auto parts = omap.findFirst("//parts");
	auto partlist = parts.getChildren("part");

	for (auto & pt : partlist) {
		auto objs = pt.getChild("objects");
		auto objlist = objs.getChildren("object");

		for (auto & obj : objlist) { //should filter for line features here (and then points etc)
			auto coords = obj.getChild("coords");
			std::vector<std::vector<std::string>> test = parse_delimited(coords.getValue(), ';', ' ');
			contours.push_back(LineFeature());
			for (auto& p : test) {
				contours.back().add_point(LinePoint(p));
			}
			contours.back().construct_splines();
		}
	}

	

}

std::vector<std::vector<std::string>> ofApp::parse_delimited(std::string s, char d1, char d2) {
	std::vector<std::vector<std::string>> main;
	std::vector<std::string> items;
	std::string item = "";
	for (auto& c : s) {
		if (c == d1 || c == d2) {
			items.push_back(item);
			item = "";

			if (c == d1) {
				main.push_back(items);
				items.clear();
			}

			continue;
		}



		item += c;

	}

	return main;

}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	for (auto & c : contours) {
		c.draw();
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
