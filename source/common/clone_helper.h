#pragma once

#include "common/common.h"

#include <unordered_map>

namespace ruffles {

template<typename T>
struct listref_hash {
	size_t operator()(const listref<T> &i) const {
		return std::hash<T*>()(&*i);
	}
};
template<typename T>
class Translate {
	std::unordered_map<listref<T>,listref<T>,listref_hash<T>> table;
public:
	listref<T> operator()(listref<T> x) {
		return table[x];
	}
	template<class F>
	void transform(listref<T> begin, listref<T> end, list<T> &destination, F f) {
		for (listref<T> it = begin; it != end; ++it) {
			table.emplace(it, destination.insert(destination.end(), f(*it)));
		}
	}
};

}
