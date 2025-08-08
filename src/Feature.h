#pragma once
#include <ofMain.h>

class Feature {

	public:
		Feature();
		inline void set_colour(ofColor c){col = c;}
	protected:
		ofColor col;
};
