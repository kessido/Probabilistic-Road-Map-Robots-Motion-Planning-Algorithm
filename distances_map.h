#pragma once

#include <map>
using namespace std;

template<class Key>
class DistancesMap {
private:
	map<Key, double> _map;
public:
	void set(const Key& key, double value) {
		_map.insert(mp(key, value));
	}

	double get(const Key& key) const {
		auto res = _map.find(key);
		if(res==_map.end())
			return  numeric_limits<double>::max();
		return res->second;
	}
};