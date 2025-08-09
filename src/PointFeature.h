#pragma once
#include "Feature.h"
#include <string>
#include <vector>
#include <iostream>
#include "ofMain.h"





class PointFeature : public Feature {

	public:
		PointFeature(glm::vec2 pos_, float rot_);

		void construct_point();
		void draw();
		inline glm::vec2 get_pos(){return pos;};
		inline float get_rotation() { return rot;}


	private:
		glm::vec2 pos;
		float rot;

		glm::vec3 symbol;

		//ofColor col;


};
