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

	typedef CGAL::Range_tree_map_traits_2<Kernel, shared_ptr<Configuration>> Traits;
	typedef CGAL::Range_tree_2<Traits> Range_tree_2;

	Range_tree_2 _tree;
	map<Point_2, shared_ptr<Configuration>> _point_to_configuration;
	FT _connectionRadius;
	Vector_2 _connectionRadiusVector;

	void invalidate() {
		double s = _point_to_configuration.size();
		_connectionRadius = 10 * sqrt((1 + log(s)) / s) * _maximumDiameter;
		_connectionRadiusVector = { _connectionRadius , _connectionRadius };
		vector<Traits::Key> elements;
		for (auto & el : _point_to_configuration)
			elements.emplace_back(el.first, el.second);
		_tree.make_tree(all(elements));
	}

	shared_ptr<Configuration> to_configuration(const Point_2 &p) {
		return make_shared<Configuration>(p);
	}

	vector<shared_ptr<Configuration>> getNeighbors(const Point_2 &p) {
		vector<Traits::Key> res;
		_tree.window_query({ p - _connectionRadiusVector, p + _connectionRadiusVector }, back_inserter(res));
		vector<shared_ptr<Configuration>> realres;
		for (auto &i : res)
			realres.pb(i.second);
		return realres;
	}

	vector<shared_ptr<Configuration>> getNeighborsConfiguration(const Configuration &p) {
		return getNeighbors(p.getPoint());
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
			for (auto &conf_neigh : getNeighborsConfiguration(*conf)) {
				Configuration::addNeighbor(conf, conf_neigh);
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
		invalidate();
		vector<Point_2> toAdd;
		for (auto & conf : confs) {
			insert(conf);
		}
	}

	shared_ptr<Configuration> getConfiguration(const Point_2 &p) const {
		return _point_to_configuration.find(p)->second;
	}
};