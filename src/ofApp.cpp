#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {

	ofBackground(0, 0, 0);


	load_map("D:/Projects/OrienteeringSim/CampFortune.omap");
	//load_map("D:/Projects/OrienteeringSim/test2.omap");

	load_symbols();
	load_features();
	get_view_transforms(win_w, win_h);




}

void ofApp::load_map(std::string mapname) {



	if (!omap.load(mapname)) {
		ofLogError() << "Couldn't load file";
	}
}

void ofApp::load_features() {

	int n_contours = 0;
	int n_cliffs = 0;
	int n_mincliffs = 0;
	int n_tags = 0;

	auto parts = omap.findFirst("//parts");
	auto partlist = parts.getChildren("part");

	for (auto & pt : partlist) {
		auto objs = pt.getChild("objects");
		auto objlist = objs.getChildren("object");

		for (auto & obj : objlist) {

			int symboltype = obj.getAttribute("symbol").getIntValue();
			sm.
			

			

			
			
		}
	}


}



void ofApp::load_line_feature(ofXml obj) {
	auto coords = obj.getChild("coords");
	std::vector<std::vector<std::string>> test = parse_delimited(coords.getValue(), ';', ' ');
	line_features.push_back(LineFeature());
	for (auto & p : test) {
		line_features.back().add_point(LinePoint(p));
	}
	line_features.back().construct_splines();
}

void ofApp::load_point_feature(ofXml obj) {
	auto coords = obj.getChild("coords");
	std::vector<std::vector<std::string>> test = parse_delimited(coords.getValue(), ';', ' ');
	glm::vec2 pos = glm::vec2(std::stoi(test[0][0]), std::stoi(test[0][1]));
	point_features.push_back(PointFeature(pos));
}

void ofApp::load_symbols() {
	auto all_symbs = omap.findFirst("//symbols");
	auto symbol_list = all_symbs.getChildren("symbol");
	for (auto& s : symbol_list) {
		int id = s.getAttribute("id").getIntValue();
		std::string name = s.getAttribute("name").getValue();
		sm.add_symbol(new Symbol(id, name));
	}

}


void ofApp::get_view_transforms(int window_x, int window_y) {
	glm::vec2 max_coords = glm::vec2(INT_MIN, INT_MIN);
	glm::vec2 min_coords = glm::vec2(INT_MAX, INT_MAX);

	for (auto & c : contours) {
		if (c.max_coords.x > max_coords.x) {
			max_coords.x = c.max_coords.x;
		} //why did i not use built ins??
		if (c.max_coords.y > max_coords.y) {
			max_coords.y = c.max_coords.y;
		}
		if (c.min_coords.x < min_coords.x) {
			min_coords.x = c.min_coords.x;
		}
		if (c.min_coords.y < min_coords.y) {
			min_coords.y = c.min_coords.y;
		}
	}

	int margin = 1000;
	max_coords += glm::vec2(margin, margin);
	min_coords -= glm::vec2(margin, margin);

	offset = -min_coords;

	scale = max_coords - min_coords;

	int size = std::max(scale.x, scale.y); 
	size /= std::min(window_x, window_y);

	double square = 1.0 / size;
	


	for (auto & c : contours) {
		c.construct_polyline(square, offset);

	}
	for (auto & t : tags) {
		t.construct_point(square, offset);
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

	for (auto & t : tags) {
		t.draw();
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
	get_view_transforms(w,h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
