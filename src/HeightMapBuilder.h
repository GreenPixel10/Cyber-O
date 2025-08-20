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

class demp {
	public:
		demp(glm::vec2 pos_, LineFeature* contour_);
		inline double get_x(){return pos.x;};
		inline double get_y(){return pos.y;};

		glm::vec2 pos;
		LineFeature* contour;

		std::vector<demp*> connections;

};

class demedge {
	public:
		demedge(std::size_t i1_, std::size_t i2_, demp* v1_ = nullptr, demp* v2_ = nullptr);
		std::pair<std::size_t, std::size_t> vertices;
		demp* v1;
		demp* v2;
};



class HeightMapBuilder {

	public:
		HeightMapBuilder();
		void load_contours(std::vector<LineFeature *> contours_);
		void build();
		void process_raw_contours();
		void triangulate();
		void draw();

	private:

		CDT::Triangulation<double> cdt;

		std::vector<LineFeature *> contours;
		std::vector<demp*> demps;
		std::vector<demedge*> constrained_edges;
		std::vector<demedge *> tri_edges;

		CDT::EdgeUSet edges;
};
