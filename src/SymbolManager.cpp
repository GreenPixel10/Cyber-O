#include "SymbolManager.h"



Symbol::Symbol(int id_, std::string name_, int symbol_category_)
	:id(id_), name(name_), symbol_category(symbol_category_), S_CODE(S_UNKNOWN){}


SymbolManager::SymbolManager() {

	//should read from file
	symbol_names[S_UNKNOWN] = { "Unknown", };
	symbol_names[S_CONTOUR] = { "Contour" };
	symbol_names[S_INDEX_CONTOUR] = { "Index contour", "Index Contour" };
	symbol_names[S_CLIFF] = { "Cliff", "Cliff, with tags" };
	symbol_names[S_MIN_CLIFF] = { "Cliff (Minimum)", "Cliff, with tags, minimum size" };
	symbol_names[S_SLOPE_TAG] = { "Slope line, contour", "Slope Line for Contour" };


	//reverse map
	for (auto const & [S_CODE, names] : symbol_names) {
		for (auto& name : names) {
			symbol_S_CODES[name] = S_CODE;
		}
	}

}

void SymbolManager::add_symbol(Symbol * s) {
	symbols.push_back(s);
	std::string symbol_name = s->get_name();
	//std::cout << symbol_name << "\n";


	if (symbol_S_CODES.count(symbol_name)) {
		s->set_S_CODE(symbol_S_CODES[symbol_name]);
	}
	else {
		s->set_S_CODE(S_UNKNOWN);
	}
	//std::cout << symbol_names.at(s->get_S_CODE())[0] << "\n\n";
}

Symbol * SymbolManager::get_symbol_by_omapID(int omapID) {
	if (omapID < symbols.size() && omapID >= 0) {
		return symbols[omapID];
	}
	else{
		std::cout << "Error identifying symbol " << omapID << "\n";
		return nullptr;
	}
	
}



