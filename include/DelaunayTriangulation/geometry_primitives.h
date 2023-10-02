#ifndef DELAUNAY_Triangulation__DELAUNAY_Triangulation_H
#define DELAUNAY_Triangulation__DELAUNAY_Triangulation_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <stack>
#include <iostream>

namespace delaunay_triangulation
{

struct Point
{
    explicit Point(double x, double y) : x(x), y(y) {}
    double x;
    double y;

};
bool operator==(const Point &lhs, const Point &rhs);
std::ostream& operator<<(std::ostream &os, const Point &p);

using PointPtr = std::shared_ptr<Point>;
bool operator==(const PointPtr &lhs, const PointPtr &rhs);
std::ostream& operator<<(std::ostream &os, const PointPtr &p);

struct Edge
{
    explicit Edge(const PointPtr &p1, const PointPtr &p2) : p1(p1), p2(p2) {}
    const PointPtr p1;
    const PointPtr p2;
};

struct Circle
{
    Circle(){};
    Circle(const Point center, double radius)
        : center(center), radius(radius) {}
    Point center = Point{0, 0};
    double radius = 0.0;
};

class Triangle
{
public:
    explicit Triangle(const PointPtr &p1,
                      const PointPtr &p2,
                      const PointPtr &p3);

    void computeCircumcircle();
    [[nodiscard]] bool includePoint(const PointPtr &p) const;
    [[nodiscard]] bool includeEdge(const PointPtr &p1, const PointPtr &p2) const;

    friend std::ostream& operator<<(std::ostream &os, const Triangle &t);

    const PointPtr p1;
    const PointPtr p2;
    const PointPtr p3;
    Circle circumcircle;

private:
    void validate() const;
};
bool operator==(const Triangle &lhs, const Triangle &rhs);

using TrianglePtr = std::shared_ptr<Triangle>;
bool operator==(const TrianglePtr &lhs, const TrianglePtr &rhs);
std::ostream& operator<<(std::ostream &os, const TrianglePtr &t);

} // namespace delaunay_Triangulation

#endif