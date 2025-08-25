#pragma once

#include "ofMain.h"
#include "LineFeature.h"
#include "PointFeature.h"
#include "SymbolManager.h"
#include "SlopeDetector.h"
#include "HeightMapBuilder.h"


#define win_w 1024
#define win_h 1024

enum STATE {
	LOADING,
	SLOPE1,
	GAPS,
	SLOPE2,
	DEM,
	IDLE
}; 

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseScrolled(int x, int y, float scrollX, float scrollY);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);


	private:

		ofCamera camera;

		ofXml omap;
		void init_camera();
		void load_map(std::string);
		void load_colours();
		void load_symbols();
		void load_features();
		Feature* load_line_feature(ofXml obj);
		Feature* load_point_feature(ofXml obj);
		void get_view_transforms();
		
		std::vector<std::vector<std::string>> parse_delimited(std::string s, char d1, char d2);

		glm::vec2 scale;
		glm::vec2 offset;

		SymbolManager sm;
		SlopeDetector sd;
		HeightMapBuilder hb;
		
		std::vector<LineFeature*> line_features;
		std::vector<PointFeature*> point_features;

		std::map<int, std::vector<Feature*>> features; //indexed by S_CODE

		glm::vec3 pan_cam_pos;
		bool panning = false;
		glm::vec3 pan_mouse_pos;

		glm::vec3 left_click_mouse_pos;

		float zoom;

		STATE state;

};
