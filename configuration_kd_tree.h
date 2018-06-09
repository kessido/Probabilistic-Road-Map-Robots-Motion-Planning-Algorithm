#pragma once

#include "CGAL_defines.h"
#include <CGAL/Range_segment_tree_traits.h>
#include <CGAL/Range_tree_k.h>
#include "node_configuration.h"
#include <vector>
#include <map>
#include <memory>

using namespace std;

class ConfigurationKdTree {
private:
	const Trap_pl &_trap_pl;
	const FT _maximumDiameter;

	map<Point_2, shared_ptr<Configuration>> _point_to_configuration;
	FT _connectionRadius;
	Vector_2 _connectionRadiusVector;

	shared_ptr<Configuration> to_configuration(const Point_2 &p) {
		return make_shared<Configuration>(p);
	}

	bool configurationIsValid(shared_ptr<Configuration> &conf) {
		Arrangement_2::Face_handle face;
		auto res = _trap_pl.locate(conf->getPoint());
		if (assign(face, res)) {
			if (!face->contained()) {
				return false;
			}
		}
		return true;
	}

	void insert(shared_ptr<Configuration> &conf) {
		if (!is_in_set(_point_to_configuration, conf->getPoint())
			&& configurationIsValid(conf)) {
			for (auto &conf_neigh : _point_to_configuration) {
				Configuration::addNeighbor(conf, conf_neigh.second);
			}
			_point_to_configuration.emplace(conf->getPoint(), conf);
		}
	}


public:
	ConfigurationKdTree(const Trap_pl &trap_pl, const FT maximumDiameter)
		: _trap_pl(trap_pl), _maximumDiameter(maximumDiameter) {}

	void insert(const Point_2 &conf) {
		insert(to_configuration(conf));
	}

	void insert(const vector<Point_2> &confs) {
		vector<Point_2> toAdd;
		for (auto & conf : confs) {
			insert(conf);
		}
	}

	shared_ptr<Configuration> getConfiguration(const Point_2 &p) const {
		return _point_to_configuration.find(p)->second;
	}
};