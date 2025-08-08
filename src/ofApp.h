#pragma once

#include "ofMain.h"
#include "LineFeature.h"
#include "PointFeature.h"
#include "SymbolManager.h"


#define win_w 1024
#define win_h 1024

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
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);


	private:
		ofXml omap;
		void load_map(std::string);
		void load_symbols();
		void load_features();
		void load_line_feature(ofXml obj);
		void load_point_feature(ofXml obj);
		void get_view_transforms(int, int);
		
		std::vector<std::vector<std::string>> parse_delimited(std::string s, char d1, char d2);
		glm::vec2 scale;
		glm::vec2 offset;

		SymbolManager sm;
		
		std::vector<LineFeature> line_features;
		std::vector<PointFeature> point_features;
};
