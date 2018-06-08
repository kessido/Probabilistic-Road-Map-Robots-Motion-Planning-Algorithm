#pragma once

#include "shortcuts.h"
#include "CGAL_defines.h"
#include <list>

using namespace std;

class Configuration {
public:
	class ConfigurationEdge {
	private:
		shared_ptr<ConfigurationEdge> _twin;
		shared_ptr<Configuration> _target;
		bool _verified = false;
		bool _verified_results;
		bool verify_function(Arrangement_2& arr, const Trap_pl &trap_pl) {
			vector<CGAL::Object> zone_elems;
			CGAL::zone(arr, Segment_2(_twin->getTarget()->getPoint(), _target->getPoint()), back_inserter(zone_elems), trap_pl);
			Arrangement_2::Face_handle face;
			for (auto &obj : zone_elems)
				if (assign(face, obj))
					if (!face->contained())
						return false;
			return true;
		}
	public:
		ConfigurationEdge(const shared_ptr<Configuration> &target) :_target(target) { }

		void setTwin(const shared_ptr<ConfigurationEdge> &twin) { _twin = twin; }

		bool verify(Arrangement_2& arr, const Trap_pl &trap_pl) {
			if (_verified)
				return _verified_results;
			_verified = true;
			_verified_results = verify_function(arr, trap_pl);

			_twin->_verified = true;
			_twin->_verified_results = _verified_results;
			return _verified_results;
		}

		bool verifyWithNoCheck() { return !_verified || _verified_results; }

		shared_ptr<Configuration> getTarget() const { return _target; }
		shared_ptr<Configuration> getStart() const { return _twin->_target; }
	};
private:
	const Point_2 _point;
public:
	list<shared_ptr<ConfigurationEdge>> edges;
	Configuration(const Point_2 p)
		:_point(p) {}
	Point_2
		getPoint() const { return _point; }
	double
		distanceFrom(const Point_2 &p) const { return sqrt((_point - p).squared_length()); }
	double
		distanceFrom(const Configuration &p) const { return p.distanceFrom(_point); }
	bool
		operator<(const Configuration& other) const { return _point < other.getPoint(); }

	static void addNeighbor(shared_ptr<Configuration> conf1, shared_ptr<Configuration> conf2) {
		shared_ptr<ConfigurationEdge> e1 = make_shared<ConfigurationEdge>(conf2);
		shared_ptr<ConfigurationEdge> e2 = make_shared<ConfigurationEdge>(conf1);

		e1->setTwin(e2);
		e2->setTwin(e1);

		conf1->edges.pb(e1);
		conf2->edges.pb(e2);
	}
	friend ostream& operator<<(ostream& os, const Configuration& t);
};
ostream& operator<<(ostream& os, const Configuration& t)
{
	os << t._point.x() << " , " << t._point.y();
	return os;
}


