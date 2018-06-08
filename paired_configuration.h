#pragma once

#include "CGAL_defines.h"
#include "node_configuration.h"
#include "shortcuts.h"
#include <memory>
#include <vector>
#include <list>

using namespace std;

class PairedConfigurationEdge;

class PairedConfiguration {
private:
	shared_ptr<Configuration> _conf[2];
public:
	PairedConfiguration(const shared_ptr<Configuration> conf1, const shared_ptr<Configuration> conf2) {
		_conf[0] = conf1;
		_conf[1] = conf2;
	}
	vector<PairedConfigurationEdge> getNeighbors();

	double distanceFrom(const PairedConfiguration &p) const { return _conf[0]->distanceFrom(*(p._conf[0])) + _conf[1]->distanceFrom(*(p._conf[1])); }
	explicit operator pair<Point_2, Point_2>() const { return { _conf[0]->getPoint() ,_conf[1]->getPoint() }; }
	bool operator<(const PairedConfiguration& other) const {
		auto res1 = *_conf[0] < *other._conf[0];
		if (res1 == (*other._conf[0] < *_conf[0])) {
			return *_conf[1] < *other._conf[1];
		}
		return res1;
	}
	bool operator ==(const PairedConfiguration &b) const {
		return (!(*this < b)) && (!(b < *this));
	}
	bool operator !=(const PairedConfiguration &b) const {
		return !(*this == b);
	}

	friend ostream& operator<<(ostream& os, const PairedConfiguration& t);
};

namespace {
	const vector<Vector_2> direction = { { -1,-1 },{ -1,1 },{ 1,1 },{ 1,-1 } };
}

class PairedConfigurationEdge {
private:
	PairedConfiguration _start;
	PairedConfiguration _target;
	shared_ptr<Configuration::ConfigurationEdge> _edge;
	shared_ptr<Configuration> _standStillConfiguration;

	bool _verified;
	bool _verified_results;
	bool _hasStart;

	bool verify_function(Arrangement_2& arr, const Trap_pl &trap_pl) {
		if (!_edge->verify(arr, trap_pl)) return false;
		Segment_2 segment1(_edge->getStart()->getPoint(), _edge->getTarget()->getPoint());
		auto p = _standStillConfiguration->getPoint();
		for (int i = 0; i < 4; i++) {
			Segment_2 segment2({ p + direction[i], p + direction[(i + 1) % 4] });
			if (CGAL::do_intersect(segment1, segment2))
				return false;
		}
		return true;
	}
public:
	PairedConfigurationEdge(const PairedConfiguration &start, const PairedConfiguration &target, const shared_ptr<Configuration::ConfigurationEdge> &edge,
		const shared_ptr<Configuration> &standStillConfiguration) :_start(start), _target(target), _edge(edge),
		_standStillConfiguration(standStillConfiguration), _hasStart(true), _verified(false) { }


	bool verify(Arrangement_2& arr, const Trap_pl &trap_pl) {
		if (_verified)
			return _verified_results;
		_verified = true;
		_verified_results = verify_function(arr, trap_pl);
		return _verified_results;
	}
	PairedConfiguration getStart() const { return _start; }
	PairedConfiguration getTarget() const { return _target; }
};


ostream& operator<<(ostream& os, const PairedConfiguration& t)
{
	os << "( " << *t._conf[0] << " ) , ( " << *t._conf[1] << " )";
	return os;
}

vector<PairedConfigurationEdge> PairedConfiguration::getNeighbors() {
	vector<PairedConfigurationEdge> res;
	for (int i = 0; i < 2; ++i) {
		auto iterator = _conf[i]->edges.begin();
		while (iterator != _conf[i]->edges.end()) {
			auto edge = *iterator;
			if (edge->verifyWithNoCheck())
			{
				if (i == 0) {
					res.emplace_back(*this, PairedConfiguration(edge->getTarget(), _conf[1]), edge, _conf[1]);
				}
				else {
					res.emplace_back(*this, PairedConfiguration(_conf[0], edge->getTarget()), edge, _conf[0]);
				}
				++iterator;
			}
			else {
				iterator = _conf[i]->edges.erase(iterator);
			}
		}
	}
	return res;
}

