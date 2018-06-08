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
		double huristicDistance;
		PairedConfiguration conf;
		Pair_PairedConfiguration_AStarHuristic(const PairedConfiguration &pc, const DistancesMap<PairedConfiguration> &distances,
			const PairedConfiguration &goal) : conf(pc) {
			huristicDistance = distances.get(pc) + pc.distanceFrom(goal);
		}
		bool operator<(const Pair_PairedConfiguration_AStarHuristic& other) const { return other.huristicDistance < huristicDistance; }
	};

	typedef priority_queue<Pair_PairedConfiguration_AStarHuristic> my_priority_queue;
	ConfigurationKdTree &_manager;
	PairedConfiguration _start, _end;

public:
	PathFinder(ConfigurationKdTree &manager, const shared_ptr<Configuration> &start1, const shared_ptr<Configuration> &end1,
		const shared_ptr<Configuration> &start2, const shared_ptr<Configuration> &end2) :
		_manager(manager), _start(start1, start2), _end(end1, end2) {}

	vector<pair<Point_2, Point_2>> findPath() {
		DistancesMap<PairedConfiguration> distances;
		map<PairedConfiguration, PairedConfiguration> comeFrom;
		set<PairedConfiguration> wasHere;
		my_priority_queue q;
		distances.set(_start, 0);
		q.emplace(_start, distances, _end);
		while (!q.empty()) {
			auto curr = q.top().conf; q.pop();
			if ((curr < _end) == (_end < curr)) {
				vector<pair<Point_2, Point_2>> res;
				res.pb((pair<Point_2, Point_2>)_end);
				while ((curr < _start) != (_start < curr)) {
					curr = comeFrom.at(curr);
					res.pb((pair<Point_2, Point_2>)curr);
				}
				reverse(all(res));
				return res;
			}
			if (!is_in_set(wasHere, curr)) {
				wasHere.insert(curr);
				for (auto &neighbor : curr.getNeighbors()) {
					if (distances.get(curr) + curr.distanceFrom(neighbor) < distances.get(neighbor)) {
						distances.set(neighbor, distances.get(curr) + curr.distanceFrom(neighbor));
						comeFrom.insert(mp(neighbor, curr));
						q.emplace(neighbor, distances, _end);
					}
				}
			}
		}
		return {};
	}
};