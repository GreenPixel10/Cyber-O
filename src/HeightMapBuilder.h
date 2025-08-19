#include <string>
#include <vector>
#include <map>
#include "ofMain.h"
#include "Feature.h"
#include "PointFeature.h"
#include "LineFeature.h"
#include "SymbolManager.h"
#include <iostream>


class HeightMapBuilder {

	public:
		HeightMapBuilder();
		void load_contours(std::vector<LineFeature *> contours_);
		void build();
		void draw();

	private:
		std::vector<LineFeature *> contours;
};
