#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {

	ofBackground(255, 255, 255);
	init_camera();


	load_map("D:/Projects/OrienteeringSim/CampFortune.omap");
	//load_map("D:/Projects/OrienteeringSim/ark.omap");
	//load_map("D:/Projects/OrienteeringSim/test2.omap");
	//load_map("D:/Projects/OrienteeringSim/forest.omap");


	load_colours();
	load_symbols();
	load_features();
	get_view_transforms();

	sd.set_features(&features);
	sd.detect_slope();




}

void ofApp::init_camera() {
	#define ORTHO_HEIGHT 100
	zoom = 11;
	camera.enableOrtho();
	camera.setNearClip(-100000);
	camera.setFarClip(100000);
	camera.setVFlip(true);
}

void ofApp::load_map(std::string mapname) {



	if (!omap.load(mapname)) {
		ofLogError() << "Couldn't load file";
	}
}

void ofApp::load_features() {


	auto parts = omap.findFirst("//parts");
	auto partlist = parts.getChildren("part");

	for (auto & pt : partlist) {
		auto objs = pt.getChild("objects");
		auto objlist = objs.getChildren("object");

		for (auto & obj : objlist) { //for each drawn feature

			int symboltype = obj.getAttribute("symbol").getIntValue(); //eg. cliff
			Symbol* s = sm.get_symbol_by_omapID(symboltype); //eg. cliff symbol template
			if (!s) { continue;} //eg. mysterious -3 symbol

			Feature* f; //eg. specific cliff

			int category = s->get_symbol_category(); //eg. line

			if (category == SC_POINT) {
				f = load_point_feature(obj);
			}
			else if (category == SC_LINE) {
				f = load_line_feature(obj);
			}

			else {
				continue; //eg. text items (for now)
			}

			int omapID = obj.getAttribute("symbol").getIntValue();
			Symbol* symbol = sm.get_symbol_by_omapID(omapID);
			int S_CODE = symbol->get_S_CODE(); //eg. S_CLIFF
			f->set_S_CODE(S_CODE);
			f->set_colour(sm.symbol_colours[S_CODE]);
			features[S_CODE].push_back(f); //eg. add to S_CLIFF list

			f->init();
	
			

			
			
		}
	}


}



Feature* ofApp::load_line_feature(ofXml obj) {

	line_features.push_back(new  LineFeature());



	auto coords = obj.getChild("coords");
	std::vector<std::vector<std::string>> test = parse_delimited(coords.getValue(), ';', ' ');
	for (auto & p : test) {
		line_features.back()->add_point(LinePoint(p));
	}


	return line_features.back();
}

Feature* ofApp::load_point_feature(ofXml obj) {


	auto coords = obj.getChild("coords");
	float rot = obj.getAttribute("rotation").getFloatValue();

	std::vector<std::vector<std::string>> test = parse_delimited(coords.getValue(), ';', ' ');
	glm::vec2 pos = glm::vec2(std::stoi(test[0][0]), std::stoi(test[0][1]));

	point_features.push_back(new PointFeature(pos, rot));




	return point_features.back();

	
}

void ofApp::load_symbols() {
	auto all_symbs = omap.findFirst("//symbols");
	auto symbol_list = all_symbs.getChildren("symbol");
	for (auto& s : symbol_list) {
		int id = s.getAttribute("id").getIntValue();
		std::string name = s.getAttribute("name").getValue();
		int s_cat = s.getAttribute("type").getIntValue();
		sm.add_symbol(new Symbol(id, name, s_cat, ofColor::hotPink));
	}

}

void ofApp::load_colours() {
	auto all_cols = omap.findFirst("//colors");
	auto col_list = all_cols.getChildren("color");
	for (auto & col : col_list) {
		float c = col.getAttribute("c").getFloatValue();
		float m = col.getAttribute("m").getFloatValue();
		float y = col.getAttribute("y").getFloatValue();
		float k = col.getAttribute("k").getFloatValue();
		sm.add_colour(c,m,y,k);
	}
}


void ofApp::get_view_transforms() {
	glm::vec2 max_coords = glm::vec2(INT_MIN, INT_MIN);
	glm::vec2 min_coords = glm::vec2(INT_MAX, INT_MAX);

	for (auto & c : line_features) {
		if (c->max_coords.x > max_coords.x) {
			max_coords.x = c->max_coords.x;
		} //why did i not use built ins??
		if (c->max_coords.y > max_coords.y) {
			max_coords.y = c->max_coords.y;
		}
		if (c->min_coords.x < min_coords.x) {
			min_coords.x = c->min_coords.x;
		}
		if (c->min_coords.y < min_coords.y) {
			min_coords.y = c->min_coords.y;
		}
	}

	int margin = 1000;
	max_coords += glm::vec2(margin, margin);
	min_coords -= glm::vec2(margin, margin);

	glm::vec2 dims = max_coords - min_coords;

	int max_side = std::max(dims.x, dims.y);
	zoom = max_side / 1000;

	glm::vec2 centre = glm::vec2((min_coords.x + max_coords.x)/2, (min_coords.y + max_coords.y)/2);


	camera.setScale(zoom);
	camera.setPosition(glm::vec3(centre, ORTHO_HEIGHT));



	for (auto & c : line_features) {
		c->construct_polyline();

	}
	for (auto & t : point_features) {
		t->construct_point();
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
	

	camera.setScale(zoom);

	///glm::vec3 cam_pos = camera.getPosition();
	//std::cout << cam_pos.x << ", " << cam_pos.y << ", " << zoom << "\n";
	//std::cout << line_features[0].min_coords.x << ", " << line_features[0].min_coords.y << "\n\n";
}

//--------------------------------------------------------------
void ofApp::draw(){


	camera.begin();


	for (auto & c : line_features) {
		//c->draw();
	}

	for (auto & t : point_features) {
		//t->draw();
	}

#define DRAW_MODE_ALL 0
#define DRAW_MODE_ALL_KNOWN 1
#define DRAW_MODE_SPECIFIED 2

	int draw_mode = DRAW_MODE_ALL_KNOWN;

	std::vector<int> features_to_draw = {S_CONTOUR, S_MIN_CLIFF};

	if (draw_mode == DRAW_MODE_SPECIFIED) { //render specified
		for (auto & ftd : features_to_draw) {
			for (Feature * f : features[ftd]) {
				f->draw();
			}
		}
	}

	else {
		for (auto & ftd : sm.symbol_names) {
			if (DRAW_MODE_ALL_KNOWN && ftd.first == S_UNKNOWN) { continue;}
			for (Feature * f : features[ftd.first]) {
				f->draw();
			}
		}
	}



	camera.end();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	int speed = zoom * zoom;
	if (key == 's') camera.setPosition(camera.getPosition() + glm::vec3(0, speed, 0));
	if (key == 'a') camera.setPosition(camera.getPosition() + glm::vec3(-speed, 0, 0));
	if (key == 'w') camera.setPosition(camera.getPosition() + glm::vec3(0, -speed, 0));
	if (key == 'd') camera.setPosition(camera.getPosition() + glm::vec3(speed, 0, 0));

	if (key == 'e') zoom /= 1.2;
	if (key == 'q') zoom *= 1.2;
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
	get_view_transforms();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
