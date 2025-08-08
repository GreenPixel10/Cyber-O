#pragma once
#include "Feature.h"
#include <string>
#include <vector>
#include <iostream>
#include "ofMain.h"





class PointFeature : public Feature {

	public:
		PointFeature(glm::vec2 pos_);

		void construct_point();
		void draw();


	private:
		glm::vec2 pos;
		double rot;

		glm::vec3 symbol;

		//ofColor col;


};
