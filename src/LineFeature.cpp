
#include "LineFeature.h"

LinePoint::LinePoint(std::vector<std::string> p) {
	x = std::stoi(p[0]);
	y = std::stoi(p[1]);
	pType = p.size() == 3 ? std::stoi(p[2]) : -1;
}



LineFeature::LineFeature() {

}
