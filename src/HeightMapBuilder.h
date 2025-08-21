#include <string>
#include <vector>
#include <map>
#include "ofMain.h"
#include "Feature.h"
#include "PointFeature.h"
#include "LineFeature.h"
#include "SymbolManager.h"
#include <iostream>

#include "CDT.h"

class demedge;
class simpleContour;

class demp {
	public:
		demp(glm::vec2 pos_, simpleContour* contour_);
		inline double get_x(){return pos.x;};
		inline double get_y(){return pos.y;};

		void calculate_slope_ranges(glm::vec2 last_, glm::vec2 next_);

		int slope_direction_by_vector(glm::vec2 vec);

		inline double cross2(glm::vec2 A, glm::vec2 B) { return (A.x * B.y) - (B.x * A.y); }

		
		void propagate();


		glm::vec2 pos;
		simpleContour* contour;

		glm::vec2 last;
		glm::vec2 next;

		glm::vec2 nextV;
		glm::vec2 lastV;

		std::vector<demedge*> connections;
		bool visited;

		

};

class demedge {
	public:
		demedge(std::size_t i1_, std::size_t i2_, demp* v1_ = nullptr, demp* v2_ = nullptr);
		std::pair<std::size_t, std::size_t> vertices;
		demp* v1;
		demp* v2;
		int slope;
		

};


class link {
	public:
	link(simpleContour * link_to_, int confidence_, int slope_);
	simpleContour* link_to;
	int confidence;
	int slope;
};

class simpleContour {
	public:
		simpleContour(LineFeature* lf);

		LineFeature* contour;

		std::vector<link*> links;

		link * get_link_by_contour(simpleContour * target);
		link * get_best_unvisited_link();
		void propagate_elevation();

		bool visited;

		simpleContour* prev;
		std::vector<simpleContour*> next;

		int confidence_distance;

		int elevation;
};

class HeightMapBuilder {

	public:
		HeightMapBuilder();
		void load_contours(std::vector<LineFeature *> contours_);
		void build();

		void process_raw_contours();
		void triangulate();
		void calculate_slopes();
		void generate_confidence_graph();
		void bottleneck();

		void draw_triangulation();
		void draw_DEM();

	private:

		CDT::Triangulation<double> cdt;

		//std::vector<LineFeature *> contours;
		std::vector<demp*> demps;
		std::vector<demedge*> constrained_edges;
		std::vector<demedge *> tri_edges;

		CDT::EdgeUSet edges;

		std::vector<simpleContour *> simple_contours;
};
