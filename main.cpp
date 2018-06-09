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

	int n = 500;
	vector<pair<Point_2, Point_2>> res;
	while (res.empty()) {
		ConfigurationKdTree manager(trap_pl, bbox.getMaximumDiameter());
		manager.insert(start1);
		manager.insert(start2);
		manager.insert(end1);
		manager.insert(end2);
		PathFinder pathFinder(manager.getConfiguration(start1), manager.getConfiguration(end1),
			manager.getConfiguration(start2), manager.getConfiguration(end2));
		vector<Point_2> points;
		points.reserve(n);
		for (int i = 0; i < n; ++i)
			points.pb(bbox.getRandomPoint());
		manager.insert(points);
		res = pathFinder.findPath(arr, trap_pl);
		n += 20;
	}
	return res;
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
	for (int i = 0; i < 29; i++) {
		findPath(startPoint1, endPoint1, startPoint2, endPoint2, outer_obstacle, obstacles);
	}
	auto result = findPath(startPoint1, endPoint1, startPoint2, endPoint2, outer_obstacle, obstacles);
	auto secs = timer.elapsed();
	cout << "Path created:      " << secs/30 << " secs" << endl;
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