#pragma once

#include "shortcuts.h"
#include "CGAL_defines.h"
#include <vector>

using namespace std;

class Configuration {
private:
	const Point_2 _point;
	const Arrangement_2& _arr;
	const Trap_pl &_trap_pl;
	vector<shared_ptr<Configuration>> _neighbors;
public:
	Configuration(const Point_2 p, const Arrangement_2& arr, const Trap_pl &trap_pl)
		:_point(p), _arr(arr), _trap_pl(trap_pl) {}
	Point_2
		getPoint() const { return _point; }
	double
		distanceFrom(const Point_2 &p) const { return sqrt((_point - p).squared_length()); }
	double
		distanceFrom(const Configuration &p) const { return p.distanceFrom(_point); }
	const vector<shared_ptr<Configuration>>&
		getNeighbors() const { return _neighbors; }
	bool
		isValid(){
		Arrangement_2::Face_handle face;
		auto res = _trap_pl.locate(_point);
		if (assign(face, res)) {
			if (!face->contained()) {
				return false;
			}
		}
		return true;
	}
	void
		addNeighbor(shared_ptr<Configuration> &conf) { _neighbors.pb(conf); }
	bool operator<(const Configuration& other) const { return _point < other.getPoint(); }

	static void addNeighbor(shared_ptr<Configuration> &conf1, shared_ptr<Configuration> &conf2) { conf1->addNeighbor(conf2); conf2->addNeighbor(conf1); }
	friend ostream& operator<<(ostream& os, const Configuration& t);
};
ostream& operator<<(ostream& os, const Configuration& t)
{
	os << t._point.x() << " , " << t._point.y();
	return os;
}


