#define _PLINE_  cout<<__LINE__<<endl;
#define _PLINE_BU 

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"
#include <time.h>
#include <stdlib.h>
#include <map>
#include "node_configuration.h"
#include "configuration_kd_tree.h"
#include "bounding_box.h"
#include "path_finder.h"

using namespace std;

//
//vector<Kernel::Vector_2> robotDirection = { { 0,0 },{ 0,1 },{ 1,1 },{ 1,0 } };
//
//double randDouble() {
//	return ((double)rand()) / ((double)RAND_MAX);
//}
//
//
//struct edge {
//	double distance;
//	int index;
//	bool checked;
//};
//
//bool operator<(const edge& x, const edge& y) {
//	return x.index < y.index;
//}
//double dist(const Point_2 &p, const Point_2 &q) {
//	return sqrt((p - q).squared_length().to_double());
//}
//
//
//vector<Point_2> confPoint;
//vector<map<int, edge>> neighbors;
//Arrangement_2 arr;
//Trap_pl trap_pl;
//KdTree kdTree;
//double boundingBoxMaximum;
//vector<Point_2> addToKdTree;
//CGAL::Bbox_2 boundingBox;
//map<Point_2, int> point_to_conf;
//int getPoint(const Point_2 &p) {
//	return point_to_conf.find(p)->second;
//}
//
//vector<Point_2> getNeighbors(const Point_2 &p) {
//	FT connectionRadius = 10;// *(log((double)confPoint.size()) / ((FT)confPoint.size())) * boundingBoxMaximum;
//	Kernel::Vector_2 vec(connectionRadius, connectionRadius);
//	CGAL::Fuzzy_iso_box<CGAL::Search_traits_2<Kernel>> fuzzyBox(p - vec, p + vec);
//	vector<Point_2> res;
//	kdTree.search(back_inserter(res), fuzzyBox);
//	return res;
//}
//
//vector<pair<double, int>> getNeightborsAndDistance(const Point_2 &p) {
//	auto neighs = getNeighbors(p);
//	vector<pair<double, int>> res;
//	for (auto neigh : neighs) {
//		res.push_back(make_pair(dist(p, neigh), getPoint(neigh)));
//
//	}
//	return res;
//}
//
//
//double dist(pair<int, int> q, pair<int, int> p) {
//	return dist(confPoint[q.first], confPoint[p.first]) + dist(confPoint[q.second], confPoint[p.second]);
//}
//
//bool connectTwoPoint(const Point_2 &p, const Point_2 &q) {
//	Kernel::Segment_2 seg(p, q);
//	std::vector<CGAL::Object> zone_elems;
//	CGAL::zone(arr, seg, std::back_inserter(zone_elems), trap_pl);
//	Polygon_set_2::Arrangement_2::Face_handle face;
//	for (auto obj : zone_elems) {
//		if (assign(face, obj)) {
//			if (!face->contained()) {
//
//				return false;
//
//			}
//
//		}
//
//	}
//
//	return true;
//}
//
//void removeEdge(int index, edge &e) {
//	neighbors[index].erase(e.index);
//	neighbors[e.index].erase(index);
//}
//
//bool checkEdge(int index, edge &e) {
//	if (e.checked) return true;
//	if (connectTwoPoint(confPoint[index], confPoint[e.index])) {
//		neighbors[index][e.index].checked = true;
//		neighbors[e.index][index].checked = true;
//		return true;
//
//	}
//	else {
//		removeEdge(index, e);
//		return false;
//
//	}
//}
//
//vector<Segment_2> getSegmentNoMovement(Point_2 p1) {
//	vector<Point_2> res_points;
//	for (auto v : robotDirection) {
//		res_points.push_back(p1 + v);
//
//	}
//	vector<Segment_2> res;
//	for (size_t i = 0; i < res_points.size(); ++i) {
//		res.emplace_back(res_points[i], res_points[(i + 1) % res_points.size()]);
//
//	}
//	return res;
//}
//
//vector<Segment_2> getSegmentMovement(Point_2 p1, Point_2 p2) {
//	vector<Point_2> res_points;
//	for (auto v : robotDirection) {
//		res_points.push_back(p1 + v);
//		res_points.push_back(p2 + v);
//
//	}
//	vector<Segment_2> res;
//	for (size_t i = 1; i < res_points.size(); ++i) {
//		for (size_t j = 0; j < i; ++j) {
//			res.emplace_back(res_points[i], res_points[j]);
//
//		}
//
//	}
//	return res;
//}
//
//
//bool checkPairToPair(int index1, edge e1, int index2) {
//	return true;
//	if (!checkEdge(index1, e1)) {
//		return false;
//	}
//	//TODO: make this more efficient
//	/*    	vector<Segment_2> segents_1 = getSegmentMovement(confPoint[index1], confPoint[e1.index]);
//			vector<Segment_2> segents_2 = getSegmentNoMovement(confPoint[index2]);
//			for (auto seg : segents_1) {
//						for (auto seg2 : segents_2) {
//								if (CGAL::do_intersect(seg, seg2)) {
//					 return false;
//			}
//
//		}
//
//	}*/
//	return true;
//}
//
//
//
//Point_2 loadPoint_2(std::ifstream &is) {
//	Kernel::FT x, y;
//	is >> x >> y;
//	Point_2 point(x, y);
//	return point;
//}
//
//Polygon_2 loadPolygon(ifstream &is) {
//	size_t polygon_size = 0;
//	is >> polygon_size;
//	Polygon_2 ret;
//	while (polygon_size--)
//		ret.push_back(loadPoint_2(is));
//	CGAL::Orientation orient = ret.orientation();
//	if (CGAL::COUNTERCLOCKWISE == orient)
//		ret.reverse_orientation();
//	return ret;
//}
//
//vector<Polygon_2> loadPolygons(ifstream &is) {
//	size_t number_of_polygons = 0;
//	is >> number_of_polygons;
//	vector<Polygon_2> ret;
//	while (number_of_polygons--)
//		ret.push_back(loadPolygon(is));
//	return ret;
//}
//
//
//int addPoint(const Point_2 &p) {
//	auto point_to_conf_res = point_to_conf.find(p);
//	if (point_to_conf_res != point_to_conf.end()) {
//		return point_to_conf_res->second;
//	}
//	int index = confPoint.size();
//	point_to_conf.insert(make_pair(p, index));
//	confPoint.push_back(p);
//	addToKdTree.push_back(p);
//	neighbors.emplace_back();
//	for (auto neigtbor : getNeightborsAndDistance(p)) {
//		neighbors[index][neigtbor.second] = { neigtbor.first, neigtbor.second, false };
//		neighbors[neigtbor.second][index] = { neigtbor.first, index, false };
//
//	}
//	return index;
//}
//
//
//
//void addRandomPoint() {
//	Point_2 p(boundingBox.xmin() + randDouble()*(boundingBox.xmax() - boundingBox.xmin()), boundingBox.ymin() + randDouble()*(boundingBox.ymax() - boundingBox.ymin()));
//	Polygon_set_2::Arrangement_2::Face_handle face;
//	auto res = trap_pl.locate(p);
//	if (!assign(face, res) || !face->contained()) {
//		addPoint(p);
//		return;
//
//	}
//}
//
//void addPoints(int n) {
//	kdTree.insert(all(addToKdTree));
//	addToKdTree.clear();
//	while (n--) {
//		addRandomPoint();
//
//	}
//}
//
//
//
//
//
//
//
//CGAL::Bbox_2 getObstaclesBoundingBox(const Point_2 &start1, const Point_2 &end1, const Point_2 &start2, const Point_2 &end2, const vector<Polygon_2> &obstacles) {
//	FT minX = min(min(start1.x(), start2.x()), min(end1.x(), end2.x())),
//		minY = min(min(start1.y(), start2.y()), min(end1.y(), end2.y())),
//		maxX = max(max(start1.x(), start2.x()), max(end1.x(), end2.x())),
//		maxY = max(max(start1.y(), start2.y()), max(end1.y(), end2.y()));
//
//	for (auto obs : obstacles) {
//		for (auto p = obs.vertices_begin(); p != obs.vertices_end(); ++p) {
//			minX = min(minX, p->x());
//			minY = min(minY, p->y());
//			maxX = max(maxX, p->x());
//			maxY = max(maxY, p->y());
//
//		}
//
//	}
//	return CGAL::Bbox_2(minX.to_double() - 2, minY.to_double() - 2, maxX.to_double() + 2, maxY.to_double() + 2);
//}
//
//
//
//
//inline double getMinDist(map<pair<int, int>, double> &bestStart, pair<int, int> p) {
//	auto res = bestStart.find(p);
//	if (res != bestStart.end()) {
//		return res->second;
//
//	}
//	return numeric_limits<double>::max();
//}
//
//vector<pair<Point_2, Point_2>> returnResultFromAStarCameFrom(pair<int, int> start, pair<int, int> goal, map<pair<int, int>, pair<int, int>> &cameFrom) {
//	vector<pair<Point_2, Point_2>> res;
//	auto curr = goal;
//	while (curr != start) {
//		res.push_back(make_pair(confPoint[curr.first], confPoint[curr.second]));
//		curr = cameFrom[curr];
//	}
//	std::reverse(all(res));
//	return res;
//}
//
//bool pathExist(set < pair<int, int>> &v, const pair<int, int> index, const pair<int, int> &goal) {
//	if (v.find(index) != v.end()) {
//		return false;
//	}
//	v.insert(index);
//	for (auto e : neighbors[index.first]) {
//		if (checkPairToPair(index.first, e.second, index.second)) {
//			if (pathExist(v, make_pair(e.first, index.second), goal)) {
//				return true;
//			}
//		}
//	}
//	for (auto e : neighbors[index.second]) {
//		if (checkPairToPair(index.second, e.second, index.first)) {
//			if (pathExist(v, make_pair(index.first, e.first), goal)) {
//				return true;
//			}
//		}
//	}
//	return false;
//}
//
//vector<pair<Point_2, Point_2>> aStarPath(pair<int, int> start, pair<int, int> goal) {
//	const double best_path_factor = 10;
//	map<pair<int, int>, pair<int, int>> cameFrom;
//	priority_queue < pair<double, pair<int, int>>, vector<pair<double, pair<int, int>>>, greater<pair<double, pair<int, int>>>> q;
//	set<pair<int, int>> wasHere;
//	map<pair<int, int>, double> bestStart;
//	bestStart[start] = 0;
//	q.push(make_pair(dist(start, goal), start));
//	while (!q.empty()) {
//		auto curr = q.top(); q.pop();
//		if (wasHere.find(curr.second) != wasHere.end()) {
//			continue;
//
//		}
//		wasHere.insert(curr.second);
//		auto currentPair = curr.second;
//		auto pos1 = currentPair.first;
//		auto pos2 = currentPair.second;
//		auto currDistHuristic = curr.first;
//		auto currDistFromStart = getMinDist(bestStart, currentPair);
//		if (currentPair == goal) {
//			cout << "FOUND!" << endl;
//			return returnResultFromAStarCameFrom(start, goal, cameFrom);
//
//		}
//		for (auto e : neighbors[pos1]) {
//			pair<int, int> new_conf(e.first, pos2);
//			double neigh_curr_dist = getMinDist(bestStart, new_conf);
//			double neigh_new_dist = e.second.distance + currDistFromStart;
//			if (neigh_curr_dist > neigh_new_dist*best_path_factor) {
//				if (checkPairToPair(e.first, e.second, pos2)) {
//					bestStart[new_conf] = neigh_new_dist;
//					cameFrom[new_conf] = currentPair;
//					q.push(make_pair(neigh_new_dist + dist(new_conf, goal), new_conf));
//
//				}
//
//			}
//
//		}
//		for (auto e : neighbors[pos2]) {
//			pair<int, int> new_conf(pos1, e.first);
//			double neigh_curr_dist = getMinDist(bestStart, new_conf);
//			double neigh_new_dist = e.second.distance + currDistFromStart;
//			if (neigh_curr_dist > neigh_new_dist*best_path_factor) {
//				if (checkPairToPair(e.first, e.second, pos1)) {
//					bestStart[new_conf] = neigh_new_dist;
//					cameFrom[new_conf] = currentPair;
//					q.push(make_pair(neigh_new_dist + dist(new_conf, goal), new_conf));
//
//				}
//
//			}
//
//		}
//
//	}
//	cout << "numberOfMatchedLookedAt:" << bestStart.size() << endl;
//
//	return {};
//}
//
//
//
//vector<pair<Point_2, Point_2>> findPath(const Point_2 &start1, const Point_2 &end1, const Point_2 &start2, const Point_2 &end2,
//	const Polygon_2 &outer_obstacle, vector<Polygon_2> &obstacles) {
//	Polygon_with_holes_2 space(outer_obstacle, all(obstacles));
//	if (outer_obstacle.is_empty()) {
//		boundingBox = getObstaclesBoundingBox(start1, end1, start2, end2, obstacles);
//
//	}
//	else {
//		boundingBox = space.bbox();
//		cout << boundingBox << endl;
//
//	}
//	boundingBoxMaximum = max(boundingBox.xmax() - boundingBox.xmin(), boundingBox.ymax() - boundingBox.ymin());
//	Polygon_set_2 space_ps2;
//	space_ps2.insert(space);
//	arr = space_ps2.arrangement();
//	trap_pl.attach(arr);
//	auto startP1 = addPoint(start1);
//	auto endP1 = addPoint(end1);
//	auto startP2 = addPoint(start2);
//	auto endP2 = addPoint(end2);
//	int n = 2;
//	bool pathFound = false;
//	vector<pair<Point_2, Point_2>> res;
//	while (!pathFound) {
//		cout << n << endl;
//		//auto res = aStarPath({ startP1,startP2 }, { endP1,endP2 });
//		pathFound = pathExist(set<pair<int, int>>(), { startP1,startP2 }, { endP1,endP2 });//res.empty() == false;
//		addPoints(n);
//		n <<= 1;
//
//	}
//	cout << "FOUND!!!" << endl;
//	while (true) {}
//	return aStarPath({ startP1,startP2 }, { endP1,endP2 });
//}
//
//


//bool testPath(const vector<pair<Point_2, Point_2>> &path, const Point_2 &start1, const Point_2 &end1, const Point_2 &start2, const Point_2 &end2,
//	const Polygon_2 &outer_obstacle, vector<Polygon_2> &obstacles) {
//	const int numberOfChecks = 100;
//	Polygon_set_2 ps;
//	Polygon_set_2 boundary;
//	boundary.join(outer_obstacle);
//	ps.join(obstacles.begin(), obstacles.end());
//	ps.complement();
//	ps.intersection(boundary);
//	Polygon_with_holes_2 tempP;
//	if (path.length == 0) return false;
//	if (path[0].first != start1 || path[0].second != start2 || path[path.length - 1].first != end1 || path[path.length - 1].second != end2) return false;
//	for (int i = 1; i < path.length; i++) {
//		auto p1 = path[i - 1].first;
//		auto v1 = path[i].first - p1;
//		auto p2 = path[i - 1].second;
//		auto v2 = path[i].second - p2;
//		for (int jj = 0; jj < numberOfChecks; jj++) {
//			FT x = randDouble();
//			auto p1_1 = p1 + v1 * x;
//			auto p1_2 = p2 + v2 * x;
//			auto x1 = p1_1.x() - p1_2.x();
//			auto y1 = p1_1.y() - p1_2.y();
//			if (-1 <= x1 && x1 <= 1 && -1 <= y1 && y1 <= 1) return false;
//			auto isInside1 = ps.oriented_side(p1_1);
//			auto isInside2 = ps.oriented_side(p1_2);
//			if (isInside1 == CGAL::ON_NEGATIVE_SIDE || isInside == CGAL::ON_NEGATIVE_SIDE) return false;
//		}
//	}
//	return true;
//}

Arrangement_2 createArrangment(Polygon_2 outer_obstacle, vector<Polygon_2> &obstacles) {
	vector<Polygon_2> obs;
	for (auto p : obstacles) {
		if (!p.is_empty()) {
			if (!p.is_clockwise_oriented())
				p.reverse_orientation();
			obs.pb(p);
		}
	}
	Polygon_set_2 ps2;
	if (outer_obstacle.is_empty()) {
		if (!obs.empty()) {
			for (auto &p : obs) p.reverse_orientation();
			ps2.insert(all(obs));
		}
		ps2.complement();
	}
	else
	{
		if (outer_obstacle.is_clockwise_oriented())outer_obstacle.reverse_orientation();
		Polygon_with_holes_2 space(outer_obstacle, all(obstacles));
		ps2.insert(space);
	}
	return ps2.arrangement();
}

vector<pair<Point_2, Point_2>> findPath(const Point_2 &start1, const Point_2 &end1, const Point_2 &start2, const Point_2 &end2,
	const Polygon_2 &outer_obstacle, vector<Polygon_2> &obstacles) {
	Arrangement_2 arr = createArrangment(outer_obstacle, obstacles);
	Trap_pl trap_pl(arr);
	BoundingBox bbox(start1); bbox.addPoint(end1); bbox.addPoint(start2); bbox.addPoint(end2);
	bbox.addPolygons(obstacles);
	bbox.addPolygon(outer_obstacle);
	ConfigurationKdTree manager(trap_pl, bbox.getMaximumDiameter());
	manager.insert(start1);
	manager.insert(start2);
	manager.insert(end1);
	manager.insert(end2);
	PathFinder pathFinder(manager, manager.getConfiguration(start1), manager.getConfiguration(end1),
		manager.getConfiguration(start2), manager.getConfiguration(end2));
	int n = 4;
	vector<pair<Point_2, Point_2>> res;
	while (res.empty()) {
		res = pathFinder.findPath(arr, trap_pl);
		vector<Point_2> points;
		points.reserve(n);
		for (int i = 0; i < n; ++i)
			points.pb(bbox.getRandomPoint());
		manager.insert(points);
		n <<= 1;
	}
	return pathFinder.findPath(arr, trap_pl);
}


Point_2 loadPoint_2(std::ifstream &is) {
	Kernel::FT x, y;
	is >> x >> y;
	Point_2 point(x, y);
	return point;
}

Polygon_2 loadPolygon(ifstream &is) {
	size_t polygon_size = 0;
	is >> polygon_size;
	Polygon_2 ret;
	if (polygon_size == 0) return ret;
	while (polygon_size--)
		ret.push_back(loadPoint_2(is));
	CGAL::Orientation orient = ret.orientation();
	if (CGAL::COUNTERCLOCKWISE == orient)
		ret.reverse_orientation();
	return ret;
}

vector<Polygon_2> loadPolygons(ifstream &is) {
	size_t number_of_polygons = 0;
	is >> number_of_polygons;
	vector<Polygon_2> ret;
	while (number_of_polygons--)
		ret.push_back(loadPolygon(is));
	return ret;
}

double dist(const Point_2 &p, const Point_2 &q) {
	return sqrt((p - q).squared_length());
}


double getPathLength(vector<pair<Point_2, Point_2>> path) {
	double res = 0;
	for (size_t i = 1; i < path.size(); i++) {
		res += dist(path[i].first, path[i - 1].first) + dist(path[i].second, path[i - 1].second);

	}
	return res;
}

int main(int argc, char *argv[]) {
	argc = 4;
	argv = new char*[4]{ "","C:\\Users\\t-idkess\\Documents\\Robots\\Clion\\HW\\5\\tests\\ou.txt","C:\\Users\\t-idkess\\Documents\\Robots\\Clion\\HW\\5\\tests\\rob.txt","C:\\Users\\t-idkess\\Documents\\Robots\\Clion\\HW\\5\\tests\\out1.txt" };
	if (argc != 4) {
		cerr << "[USAGE]: inputRobots inputObstacles outputFile" << endl;
		return 1;

	}
	srand((unsigned int)time(NULL));   // should only be called once

	ifstream inputRobotsFile(argv[1]), inputObstaclesFile(argv[2]);
	if (!inputRobotsFile.is_open() || !inputObstaclesFile.is_open()) {
		if (!inputRobotsFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[1] << endl;
		if (!inputObstaclesFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[2] << endl;
		return -1;

	}

	auto startPoint1 = loadPoint_2(inputRobotsFile);
	auto endPoint1 = loadPoint_2(inputRobotsFile);
	auto startPoint2 = loadPoint_2(inputRobotsFile);
	auto endPoint2 = loadPoint_2(inputRobotsFile);
	inputRobotsFile.close();

	auto outer_obstacle = loadPolygon(inputObstaclesFile);
	auto obstacles = loadPolygons(inputObstaclesFile);
	inputObstaclesFile.close();

	boost::timer timer;
	for (int i = 0; i < 10; i++) {
		findPath(startPoint1, endPoint1, startPoint2, endPoint2, outer_obstacle, obstacles);
	}
	auto result = findPath(startPoint1, endPoint1, startPoint2, endPoint2, outer_obstacle, obstacles);
	auto secs = timer.elapsed();
	cout << "Path created:      " << secs << " secs" << endl;
	cout << "Path legnth:      " << getPathLength(result) << endl;
	ofstream outputFile;
	outputFile.open(argv[3]);
	if (!outputFile.is_open()) {
		cerr << "ERROR: Couldn't open file: " << argv[3] << endl;
		return -1;

	}
	outputFile << result.size() << endl;
	for (auto &p : result) {
		outputFile << p.first.x() << " " << p.first.y() << " " << p.second.x()
			<< " " << p.second.y() << endl;
	}
	outputFile.close();
	getchar();
	return 0;
}