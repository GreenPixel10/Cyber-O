#include "SymbolManager.h"



Symbol::Symbol(int id_, std::string name_): id(id_), name(name_) {}


SymbolManager::SymbolManager() {

	//should read from file
	symbol_names[S_CONTOUR] = { "Contour", };
	symbol_names[S_INDEX_CONTOUR] = {"Index contour", "Index Contour" };
	symbol_names[S_CLIFF] = { "Cliff", "Cliff, with tags" };
	symbol_names[S_MIN_CLIFF] = { "Cliff (Minimum)", "Cliff, with tags, minimum size" };
	symbol_names[S_SLOPE_TAG] = { "Slope line, contour", "Slope Line for Contour" };


}

bool SymbolManager::is(int type, std::string name) {
	for (auto& s : symbols) {
		if (s->get_name() == name && s->get_id() == type) {
			return true;
		}
	}
	return false;
}

bool SymbolManager::is(int type, std::vector<std::string> names) {
	for (auto & n : names) {
		if (is(type, n)) {
			return true;
		}
	}
	return false;
}

