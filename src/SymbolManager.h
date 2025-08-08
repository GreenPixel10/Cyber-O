#pragma once

#include <string>
#include <vector>
#include <map>

//these are the symbols that THIS program uses/recognizes
//values are unrelated to whatever values may be in the .omap
//eventually should be read from a file
#define S_CONTOUR 1000
#define S_INDEX_CONTOUR 1001
#define S_CLIFF 1002
#define S_MIN_CLIFF 1003
#define S_SLOPE_TAG 1004


class Symbol {
	public:
		Symbol(int id_, std::string name_);
		inline int get_id() { return id;};
		inline std::string get_name() {return name; };

	private:
		int id;
		std::string name;
};


class SymbolManager {
	public:
		SymbolManager();
		inline void add_symbol(Symbol *s) {symbols.push_back(s);}
		bool is(int type, std::string name);
		bool is(int type, std::vector<std::string> names);
		inline int size() { return symbols.size(); }


		std::map<int, std::vector<string>> symbol_names;


	private:
		std::vector<Symbol*> symbols;
};
