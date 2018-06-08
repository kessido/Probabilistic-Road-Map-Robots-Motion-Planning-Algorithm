#pragma once

#include "CGAL_defines.h"
#include "node_configuration.h"
#include <vector>
#include <map>
#include <memory>

using namespace std;

class ConfigurationKdTree {
private:
	const Trap_pl &_trap_pl;
	const FT _maximumDiameter;

	KdTree _kdTree;
	map<Point_2, shared_ptr<Configuration>> _point_to_configuration;
	FT _connectionRadius;
	Vector_2 _connectionRadiusVector;

	void updateConnectionRadius() {
		double s = _kdTree.size();
		_connectionRadius = 10 * sqrt((1+log(s)) / s) * _maximumDiameter;
		_connectionRadiusVector = { _connectionRadius , _connectionRadius };
	}

	shared_ptr<Configuration> to_configuration(const Point_2 &p) {
		return shared_ptr<Configuration>(new Configuration(p));
	}

	void add_to_kdTree(const Point_2 &conf) {
		_kdTree.insert(conf);
		updateConnectionRadius();
	}

	void add_to_kdTree(const vector<Point_2> &confs) {
		_kdTree.insert(all(confs));
		updateConnectionRadius();
	}

	vector<Point_2> getNeighbors(const Point_2 &p) {
		Fuzzy_iso_box box(p - _connectionRadiusVector, p + _connectionRadiusVector);
		vector<Point_2> res;
		_kdTree.search(back_inserter(res), box);
		return res;
	}

	vector<shared_ptr<Configuration>> getNeighborsConfiguration(const Point_2 &p) {
		auto points = getNeighbors(p);
		vector<shared_ptr<Configuration>> res;
		res.reserve(points.size());
		for (auto &point : points) {
			res.pb(_point_to_configuration.find(point)->second);
		}
		return res;
	}

	vector<shared_ptr<Configuration>> getNeighborsConfiguration(const Configuration &p) {
		return getNeighborsConfiguration(p.getPoint());
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

	bool insert(shared_ptr<Configuration> &conf) {
		if (!is_in_set(_point_to_configuration, conf->getPoint())
			&& configurationIsValid(conf)) {
			for (auto &conf_neigh : getNeighborsConfiguration(*conf)) {
				Configuration::addNeighbor(conf, conf_neigh);
			}
			_point_to_configuration.insert(mp(conf->getPoint(), conf));
			return true;
		}
		return false;
	}


public:
	ConfigurationKdTree(const Trap_pl &trap_pl, const FT maximumDiameter)
		: _trap_pl(trap_pl), _maximumDiameter(maximumDiameter) {}

	bool insert(const Point_2 &conf, bool shouldAddToKdTree = true) {
		bool res = insert(to_configuration(conf));
		if (res && shouldAddToKdTree)
			add_to_kdTree(conf);
		return res;
	}

	void insert(const vector<Point_2> &confs) {
		vector<Point_2> toAdd;
		for (auto & conf : confs) {
			if (insert(conf, false)) {
				toAdd.pb(conf);
			}
		}
		add_to_kdTree(toAdd);
	}

	shared_ptr<Configuration> getConfiguration(const Point_2 &p) const {
		return _point_to_configuration.find(p)->second;
	}
};