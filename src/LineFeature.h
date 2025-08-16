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
		void set_slope_verified(bool is_slope_verified, bool recurse = false);
		inline void lean_slope_correct() {slope_leaner++;}
		inline void lean_slope_wrong() { slope_leaner--; }
		void lean_slope_apply();

		int get_length_at_point(glm::vec2 point);
		inline int get_length_at_point(glm::vec3 point) {return get_length_at_point(glm::vec2(point.x, point.y));};

		inline void add_link_prev(std::pair<LineFeature *, bool> lf) { link_prev.push_back(lf); }
		inline void add_link_next(std::pair<LineFeature *, bool> lf) { link_next.push_back(lf); }
		inline vector<std::pair<LineFeature *, bool>> get_link_prev() { return link_prev; }
		inline vector<std::pair<LineFeature *, bool>> get_link_next() { return link_next; }

		void store_all_links();
		std::vector<LineFeature *> traverse_link_chain();
		inline void add_linked_reference(LineFeature * lf) { if(lf!=this){linked_references.push_back(lf);} }
		inline bool are_all_links_gathered() { return all_links_gathered;}
		inline void set_all_links_gathered(bool b) { all_links_gathered = b; }

		void construct_splines();
		void construct_polyline();
		void draw();

		void slope_alignment_reversal(LineFeature * origin = nullptr, LineFeature * last = nullptr, int lazy_depth = 0);
		void reverse_single_slope();
		void reverse_all_linked_slopes();

		

		
		
		
		Point min_coords;
		Point max_coords;

		std::vector<std::pair<LineFeature *, bool>> link_next;
		std::vector<std::pair<LineFeature *, bool>> link_prev;
		void flip_link_to(LineFeature* lf);
		bool is_aligned();
		bool linked_flag;

	private:
		std::vector<LinePoint> points; //raw data
		std::vector<SplinePoint> spline_points; //parsed spline data (3 points in each)

		bool closed;

		ofPolyline line; //draw line

		bool slope_verified;
		int slope_leaner; //negative means probably WRONG, positive means probably RIGHT

		std::vector<LineFeature*> linked_references;
		bool all_links_gathered;



};
