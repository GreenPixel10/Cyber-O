#pragma once

#include <string>
#include <vector>
#include <map>

#include <iostream>

//these are the symbols that THIS program uses/recognizes
//values are unrelated to whatever values may be in the .omap
//eventually should be read from a file
#define S_UNKNOWN -1
#define S_CONTOUR 1000
#define S_INDEX_CONTOUR 1001
#define S_CLIFF 1002
#define S_MIN_CLIFF 1003
#define S_SLOPE_TAG 1004

//symbol categories
#define SC_UNKNOWN -1
#define SC_POINT 1
#define SC_LINE 2
#define SC_AREA 4
#define SC_TEXT 8
#define SC_COMBINED 16


class Symbol {
	public:
		Symbol(int id_, std::string name_, int symbol_category_);
		inline int get_id() { return id;};
		inline std::string get_name() {return name; };
		inline int get_symbol_category() { return symbol_category; }
		inline int get_S_CODE() { return S_CODE; };
		inline void set_S_CODE(int S_CODE_) {S_CODE = S_CODE_;  };

	private:
		int id;
		std::string name;
		int symbol_category;
		int S_CODE;
};


class SymbolManager {
	public:
		SymbolManager();
		void add_symbol(Symbol *s);
		inline int size() { return symbols.size(); }

		Symbol * get_symbol_by_omapID(int omapID); //ASSUMING that omap IDs are assigned 1-n.
																			//this may not be true...
															//in which case might need a omapID->symbol* std::map

		std::map<int, std::vector<std::string>> symbol_names;
		//maps S_CODES to potential names

		std::map<std::string, int> symbol_S_CODES;
		//maps names to S_CODES

		//std::map<int, int> symbols_SCODES_by_omapID;
		//maps omapIDs to S_CODES



	private:
		std::vector<Symbol*> symbols;
};
