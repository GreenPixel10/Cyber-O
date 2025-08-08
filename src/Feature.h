#pragma once
#include <ofMain.h>

class Feature {

	public:
		Feature();
		inline void set_colour(ofColor c){col = c;}
		inline void set_S_CODE(int S_CODE_) { S_CODE = S_CODE_;}
		inline int get_S_CODE() { return S_CODE; }
		virtual void draw() = 0;
	protected:
		ofColor col;
		int S_CODE;
};
