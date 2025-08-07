#pragma once

#include "ofMain.h"
#include "LineFeature.h"

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
		void load_map();
		std::vector<LineFeature> contours;
		std::vector<std::vector<std::string>> parse_delimited(std::string s, char d1, char d2);
		
		
};
