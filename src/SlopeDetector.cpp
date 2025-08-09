#include "SlopeDetector.h"

SlopeDetector::SlopeDetector() { }

void SlopeDetector::detect_slope() {
	//for each point feature     v--point feature
	for (auto mc : (*features)[S_MIN_CLIFF]) {
		PointFeature* cliff = dynamic_cast<PointFeature*>(mc); //whatever
		glm::vec3 mc_pos = glm::vec3(cliff->get_pos(), 0);

		//for each line feature     v--line feature
		for (auto c : (*features)[S_CONTOUR]) {
			LineFeature * contour = dynamic_cast<LineFeature *>(c);
			ofPolyline line = contour->get_line();
			unsigned int ind;
			glm::vec3 closest = line.getClosestPoint(mc_pos, &ind);
			int close = 5; //metres
			int dist = glm::distance(closest, mc_pos);
			if (dist < close * 100) {
				int index = ind - 1;

				glm::vec2 segment_vec = line.getNormalAtIndex(ind);
				//std::cout << segment_vec.x << " " << segment_vec.y << "\n";

				float rot = -cliff->get_rotation() + HALF_PI;
				glm::vec2 point_vec = { cos(rot), sin(rot) };


				//std::cout << point_vec.x << " " << point_vec.y << "\n";

				float alignment = glm::dot(glm::vec3(segment_vec, 0), glm::vec3(point_vec, 0));

				if (alignment > 0.5f) {c->set_colour(ofColor::green);
									mc->set_colour(ofColor::green);
				}
				if (alignment < -0.5f) {
					c->set_colour(ofColor::red);
					mc->set_colour(ofColor::red);
				}
 				
			}
			
		}
	}
	//getClosestPoint();
}
