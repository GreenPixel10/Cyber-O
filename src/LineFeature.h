#pragma once
#include "Feature.h"
#include <string>
#include <vector>
#include <iostream>
#include "ofMain.h"


class Point {
	public:
		int x, y;
		bool null;
};

class LinePoint {

	public: 
		LinePoint(std::vector<std::string> p);

		Point pos;
		int pType;

};

class SplinePoint {
	public:
		SplinePoint(Point p, Point p_h, Point n_h);
		glm::vec2 pos;
		glm::vec2 p_handle;
		glm::vec2 n_handle;


};

class LineFeature : public Feature {

	public:
		LineFeature();

		void init() override;

		inline void add_point(LinePoint p) { points.push_back(p); }
		inline ofPolyline get_line() { return line;}
		inline bool get_closed() { return closed;}
		bool get_closed_via_linked();

		void align_linked();
		inline void set_linked_flag(bool set) { linked_flag = set;}

		inline bool get_slope_verified() { return slope_verified; }
		inline void set_slope_verified(bool is_slope_verified) { slope_verified = is_slope_verified; }
		inline void lean_slope_correct() {slope_leaner++;}
		inline void lean_slope_wrong() { slope_leaner--; }
		void lean_slope_apply();

		int get_length_at_point(glm::vec2 point);
		inline int get_length_at_point(glm::vec3 point) {return get_length_at_point(glm::vec2(point.x, point.y));};

		inline void add_link_prev(std::pair<LineFeature *, bool> lf) { link_prev.push_back(lf); }
		inline void add_link_next(std::pair<LineFeature *, bool> lf) { link_next.push_back(lf); }
		inline vector<std::pair<LineFeature *, bool>> get_link_prev() { return link_prev; }
		inline vector<std::pair<LineFeature *, bool>> get_link_next() { return link_next; }

		void construct_splines();
		void construct_polyline();
		void reverse_slope(LineFeature * origin, LineFeature * last, int lazy_depth = 0);
		void draw();

		
		
		
		Point min_coords;
		Point max_coords;

	private:
		std::vector<LinePoint> points; //raw data
		std::vector<SplinePoint> spline_points; //parsed spline data (3 points in each)

		bool closed;

		ofPolyline line; //draw line

		bool slope_verified;
		int slope_leaner; //negative means probably WRONG, positive means probably RIGHT

		std::vector<std::pair<LineFeature *, bool>> link_next;
		std::vector<std::pair<LineFeature *, bool>> link_prev;
		bool linked_flag;



};
