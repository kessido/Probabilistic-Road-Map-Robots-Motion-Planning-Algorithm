#pragma once

#include "CGAL_defines.h"
#include "node_configuration.h"
#include "shortcuts.h"
#include <memory>
#include <vector>

using namespace std;

namespace {
	const vector<Vector_2> direction = { {-1,-1},{-1,1},{1,1},{1,-1} };
}

class PairedConfiguration {
private:
	shared_ptr<Configuration> _conf1, _conf2;
	static bool canBeConnected(const shared_ptr<Configuration> &_conf1, const shared_ptr<Configuration> &_conf1_2, const shared_ptr<Configuration> &_conf2) {
		Segment_2 segment1(_conf1->getPoint(), _conf1_2->getPoint());
		auto p = _conf2->getPoint();
		for (int i = 0; i < 4; i++) {
			Segment_2 segment2({ p + direction[i], p + direction[(i + 1) % 4] });
			if (CGAL::do_intersect(segment1, segment2))
				return false;
		}
		return true;
	}
public:
	PairedConfiguration(const shared_ptr<Configuration> conf1, const shared_ptr<Configuration> conf2) : _conf1(conf1), _conf2(conf2) {}
	double
		distanceFrom(const PairedConfiguration &p) const { return _conf1->distanceFrom(*(p._conf1)) + _conf1->distanceFrom(*(p._conf2)); }
	vector<PairedConfiguration> getNeighbors() const {
		vector<PairedConfiguration> res;
		auto iterator = _conf1->getNeighbors();
		for (auto &conf1_neigh : iterator) {
			if (canBeConnected(_conf1, conf1_neigh, _conf2))
			{
				PairedConfiguration p = { conf1_neigh, _conf2 };
				res.pb(p);
			}
		}
		iterator = _conf2->getNeighbors();
		for (auto &conf2_neigh : iterator) {
			if (canBeConnected(_conf2, conf2_neigh, _conf1))
			{
				PairedConfiguration p = { _conf1, conf2_neigh };
				res.pb(p);
			}
		}
		return res;
	}
	explicit operator pair<Point_2, Point_2>() const { return { _conf1->getPoint() ,_conf2->getPoint() }; }

	bool operator<(const PairedConfiguration& other) const {
		auto res1 = *_conf1 < *other._conf1;
		if (res1 == (*other._conf1 < *_conf1)) {
			return *_conf2 < *other._conf2;
		}
		return res1;
	}
	friend ostream& operator<<(ostream& os, const PairedConfiguration& t);
};
ostream& operator<<(ostream& os, const PairedConfiguration& t)
{
	os << "( " << *t._conf1 << " ) , ( " << *t._conf2 << " )";
	return os;
}
