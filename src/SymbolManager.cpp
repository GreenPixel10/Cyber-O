#include "SymbolManager.h"



Symbol::Symbol(int id_, std::string name_, int symbol_category_, ofColor display_colour_)
	: id(id_)
	, name(name_)
	, symbol_category(symbol_category_)
	, S_CODE(S_UNKNOWN)
	, display_colour(display_colour_)
	{ }




SymbolManager::SymbolManager() {

	//should read from file
	symbol_names[S_UNKNOWN] = { "Unknown", };
	symbol_names[S_CONTOUR] = { "Contour", "Index contour", "Index Contour" };
	//symbol_names[S_INDEX_CONTOUR] = { "Index contour", "Index Contour" };
	symbol_names[S_CLIFF] = { "Cliff", "Cliff, with tags", "Passable rock face" };
	symbol_names[S_MIN_CLIFF] = { "Cliff (Minimum)", "Cliff, with tags, minimum size", "Passable rock face, minimum size" };
	symbol_names[S_SLOPE_TAG] = { "Slope line, contour", "Slope Line for Contour" };
	symbol_names[S_IMPASSABLE_CLIFF] = { "Impassable cliff"};
	symbol_names[S_IMPASSABLE_MIN_CLIFF] = { "Impassable cliff, minimum size"};



	//reverse map
	for (auto const & [S_CODE, names] : symbol_names) {
		for (auto& name : names) {
			symbol_S_CODES[name] = S_CODE;
		}
	}

	
	//should be pulled from the colours vector (just hard bc the colour ids are mutliple and nested)
	symbol_colours[S_UNKNOWN] = ofColor::hotPink;
	symbol_colours[S_CONTOUR] = ofColor::brown;
	symbol_colours[S_INDEX_CONTOUR] = ofColor::brown;
	symbol_colours[S_CLIFF] = ofColor::black;
	symbol_colours[S_MIN_CLIFF] = ofColor::black;
	symbol_colours[S_SLOPE_TAG] = ofColor::brown;
	symbol_colours[S_IMPASSABLE_CLIFF] = ofColor::black;
	symbol_colours[S_IMPASSABLE_MIN_CLIFF] = ofColor::black;


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


void SymbolManager::add_colour(float c, float m, float y, float k) {
	//std::cout << c << " " << m << " " << y << " " << k << "\n";
	float r = 255 * (1 - c) * (1 - k);
	float g = 255 * (1 - m) * (1 - k);
	float b = 255 * (1 - y) * (1 - k);
	colours.push_back(ofColor(r,g,b));
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



