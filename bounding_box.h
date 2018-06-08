#include "CGAL_defines.h"
#include <random>

using namespace std;
class BoundingBox {
private:
	FT minX, minY, maxX, maxY;
	mt19937 gen;
	uniform_real_distribution<double> dis;
public:
	BoundingBox(const Point_2 &p) {
		minX = p.x();
		minY = p.y();
		maxX = p.x();
		maxY = p.y();
		addPoint(p);
		random_device rd;
		gen = mt19937(rd());
	}

	FT getMaximumDiameter() const {
		return max(maxX - minX, maxY - minY);
	}

	void addPoint(const Point_2 &p) {
		minX = min(minX, p.x()-3);
		minY = min(minY, p.y()-3);
		maxX = max(maxX, p.x()+3);
		maxY = max(maxY, p.y()+3);
	}

	void addPolygon(const Polygon_2 &p) {
		for (auto ver = p.vertices_begin(); ver != p.vertices_end(); ++ver)
			addPoint(*ver);
	}

	void addPolygons(const vector<Polygon_2> &p) {
		for (auto &pol : p)
			addPolygon(pol);
	}

	Point_2 getRandomPoint() {
		FT x, y;
		x = minX + dis(gen) * (maxX - minX);
		y = minY + dis(gen) * (maxY - minY);
		return Point_2(x, y);
	}
};