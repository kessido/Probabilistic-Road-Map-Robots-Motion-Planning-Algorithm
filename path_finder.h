#pragma once

#include "CGAL_defines.h"
#include "shortcuts.h"
#include "paired_configuration.h"
#include "node_configuration.h"
#include "configuration_kd_tree.h"
#include "distances_map.h"
#include <map>
#include <set>
#include <queue>

using namespace std;

class PathFinder {
private:
	class Pair_PairedConfiguration_AStarHuristic {
	public:
		PairedConfiguration conf;
		double huristicDistance;
		bool isStart;
		Pair_PairedConfiguration_AStarHuristic(const PairedConfiguration &conf, double huristicDistance, bool isStart = 0) :
			conf(conf), huristicDistance(huristicDistance), isStart(isStart) {}
		bool operator<(const Pair_PairedConfiguration_AStarHuristic& other) const { return other.huristicDistance < huristicDistance; }
	};

	typedef priority_queue<Pair_PairedConfiguration_AStarHuristic> my_priority_queue;
	ConfigurationKdTree &_manager;
	PairedConfiguration _start, _end;
	bool pathFound = false;

	bool DSU_checkForPathExistence(Arrangement_2& arr, const Trap_pl &trap_pl) {
		set<PairedConfiguration> startSet, endSet;
		startSet.insert(_start);
		startSet.insert(_end);

		my_priority_queue q;
		q.emplace(_start, _start.distanceFrom(_end), true);
		q.emplace(_end, _end.distanceFrom(_start), false);
		while (!q.empty()) {
			auto curr = q.top(); q.pop();
			auto &mySet = curr.isStart ? startSet : endSet;
			auto &otherSet = curr.isStart ? endSet : startSet;
			auto &huristicEndPoint = curr.isStart ? _end : _start;
			for (auto &edge : curr.conf.getNeighbors()) {
				auto edgeTarget = edge.getTarget();
				if (!is_in_set(mySet, edgeTarget) && edge.verify(arr, trap_pl)) {
					if (is_in_set(otherSet, edgeTarget))
						return true;
					mySet.insert(edgeTarget);
					q.emplace(edgeTarget, edgeTarget.distanceFrom(huristicEndPoint));
				}
			}
		}
		return false;
	}


public:
	PathFinder(ConfigurationKdTree &manager, const shared_ptr<Configuration> &start1, const shared_ptr<Configuration> &end1,
		const shared_ptr<Configuration> &start2, const shared_ptr<Configuration> &end2) :
		_manager(manager), _start(start1, start2), _end(end1, end2) {}

	vector<pair<Point_2, Point_2>> findPath(Arrangement_2& arr, const Trap_pl &trap_pl) {
		if (_start == _end) return { (pair<Point_2, Point_2>) _start,(pair<Point_2, Point_2>) _end };
		if (!pathFound) {
			if (DSU_checkForPathExistence(arr, trap_pl)) {
				pathFound = true;
			}
			else {
				return {};
			}
		}
		DistancesMap<PairedConfiguration> distances;
		map<PairedConfiguration, PairedConfiguration> comeFrom;
		set<PairedConfiguration> wasHere;
		my_priority_queue q;
		distances.set(_start, 0);
		q.emplace(_start, _start.distanceFrom(_end));
		while (!q.empty()) {
			auto curr = q.top().conf; q.pop();
			if (!is_in_set(wasHere, curr)) {
				wasHere.insert(curr);
				if (curr == _end) {
					vector<pair<Point_2, Point_2>> res;
					res.pb((pair<Point_2, Point_2>)_end);
					while (curr != _start) {
						curr = comeFrom.at(curr);
						res.pb((pair<Point_2, Point_2>)curr);
					}
					reverse(all(res));
					return res;
				}
				for (auto &edge : curr.getNeighbors()) {
					auto new_dist = distances.get(curr) + curr.distanceFrom(edge.getTarget());
					auto old_dist = distances.get(edge.getTarget());
					if (new_dist < old_dist && edge.verify(arr, trap_pl)) {
						distances.set(edge.getTarget(), new_dist);
						comeFrom.emplace(edge.getTarget(), curr);
						auto huristic_distance = new_dist + curr.distanceFrom(_end);
						q.emplace(edge.getTarget(), huristic_distance);
					}
				}
			}
		}
		return {};
	}


};