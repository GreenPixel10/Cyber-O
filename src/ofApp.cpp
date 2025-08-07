#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {
	load_map();
	cout << contours.size() << "\n";
}

void ofApp::load_map() {

	if (!omap.load("D:/Projects/OrienteeringSim/test2.omap")) {
		ofLogError() << "Couldn't load file";
	}

	auto basemap = omap.getChild("map");
	auto parts = omap.findFirst("//parts");
	auto partlist = parts.getChildren("part");

	for (auto & pt : partlist) {
		auto objs = pt.getChild("objects");
		auto objlist = objs.getChildren("object");

		for (auto & obj : objlist) {
			auto coords = obj.getChild("coords");
			std::vector<std::vector<std::string>> test = parse_delimited(coords.getValue(), ';', ' ');
			contours.push_back(LineFeature());
			for (auto& p : test) {
				contours.back().add_point(LinePoint(p));
			}
		}
	}

	//string test_s = "-28722 -3915;-22732 -40940;2042 -5140;-28722 -3915 18;";
	

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
