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

class LineFeature;
class GapLink {
	public:
		GapLink(LineFeature* to_, bool is_aligned_, int variance_);
		LineFeature *to;
		bool is_aligned;
		int variance;
};


enum LINKTYPE {
	NONE,
	UNAMB,
	AMB_AUTO,
	AMB_UNKNOWN,
	USER
}; 

class LineFeature : public Feature {

	public:
		LineFeature();

		void init() override;

		inline void add_point(LinePoint p) { points.push_back(p); }
		inline ofPolyline& get_line() { return line;}
		inline bool get_closed() { return closed;}
		bool get_closed_via_linked();
		bool is_facing_outwards();


		inline bool get_slope_verified() { return slope_verified; }
		void set_slope_verified(bool is_slope_verified, bool recurse = false);
		inline void lean_slope_correct() {slope_leaner++;}
		inline void lean_slope_wrong() { slope_leaner--; }
		void lean_slope_apply();

		int get_length_at_point(glm::vec2 point);
		inline int get_length_at_point(glm::vec3 point) {return get_length_at_point(glm::vec2(point.x, point.y));};

		void autoclose_almost_loop();

		inline bool are_all_links_gathered() { return all_links_gathered;}
		inline void set_all_links_gathered(bool b) { all_links_gathered = b; }

		void construct_splines();
		void construct_polyline();
		void draw(float zoom);

		void reverse_single_slope();
		int append_line(LineFeature* lf, bool after);

		

		
		
		
		Point min_coords;
		Point max_coords;



		std::vector < GapLink *> link_next;
		std::vector< GapLink *> link_prev;

		LineFeature* link_next_final;
		glm::vec3 link_next_point;
		LineFeature* link_prev_final;
		glm::vec3 link_prev_point;
		LineFeature* merge_tunnel;


		bool deletion_flag;


		int elevation = INT_MIN;

	private:
		std::vector<LinePoint> points; //raw data
		std::vector<SplinePoint> spline_points; //parsed spline data (3 points in each)

		bool closed;

		ofPolyline line; //draw line

		bool slope_verified;
		int slope_leaner; //negative means probably WRONG, positive means probably RIGHT

		
		bool all_links_gathered;



};
