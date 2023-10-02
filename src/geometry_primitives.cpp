#include <iostream>
#include <array>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.h"

namespace delaunay_triangulation
{

bool operator==(const Point &lhs, const Point &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator==(const PointPtr &lhs, const PointPtr &rhs)
{
    if(!lhs || !rhs) throw std::invalid_argument("nullptr");
    return *lhs == *rhs;
}

std::ostream& operator<<(std::ostream &os, const Point &p)
{
    os << "(x, y) = (" << p.x << ", " << p.y << ")";
    return os;
}

std::ostream& operator<<(std::ostream &os, const PointPtr &p)
{
    os << *p;
    return os;
}

Triangle::Triangle(const PointPtr &p1, const PointPtr &p2, const PointPtr &p3)
    : p1(p1), p2(p2), p3(p3)
{
    this->validate();
    this->computeCircumcircle();
}

void Triangle::validate() const
{
    auto are_collinear = [](const PointPtr &a, const PointPtr &b, const PointPtr &c) -> bool
    {
        const std::array<double, 2> vec12 = {b->x - a->x, b->y - a->y};
        const std::array<double, 2> vec13 = {c->x - a->x, c->y - a->y};
        const double cross = vec12.at(0) * vec13.at(1) - vec12.at(1) * vec13.at(0);
        return cross == 0;
    };

    if (p1 == nullptr || p2 == nullptr || p3 == nullptr)
        throw std::runtime_error("Triangle is not valid: nullptr");

    if (p1 == p2 || p2 == p3 || p3 == p1)
        throw std::runtime_error("Triangle is not valid: duplicate points");

    if (are_collinear(p1, p2, p3))
        throw std::runtime_error("Triangle is not valid: collinear points");
}

void Triangle::computeCircumcircle()
{
    auto midpoint = [](const PointPtr &a, const PointPtr &b) -> Point
    {
        return Point{(a->x + b->x) / 2.0, (a->y + b->y) / 2.0};
    };

    auto slope = [](const PointPtr &a, const PointPtr &b) -> std::optional<double>
    {
        if (b->x == a->x)
            return std::nullopt; // vertical
        return (b->y - a->y) / (b->x - a->x);
    };

    auto compute_perpendicular_bisector = [&](const Point &midpoint,
                                              const std::optional<double> &slope)
                                          -> std::function<double(double)>
    {
        if (slope) // not vertical
            return [midpoint, slope = *slope](double x)
            {
                return -(1.0 / slope) * (x - midpoint.x) + midpoint.y;
            };
        return [midpoint](double) { return midpoint.y; };
    };

    Point m12 = midpoint(p1, p2);
    Point m13 = midpoint(p1, p3);

    auto bisector12 = compute_perpendicular_bisector(m12, slope(p1, p2));
    auto bisector13 = compute_perpendicular_bisector(m13, slope(p1, p3));

    double center_x;

    // if both bisectors are not vertical
    if (auto slope12 = slope(p1, p2), slope13 = slope(p1, p3); slope12 && slope13)
    {
        center_x = (m13.y - m12.y + (*slope12) * m12.x - (*slope13) * m13.x) / (*slope12 - *slope13);
    }
    else
    {
        center_x = slope12 ? m13.x : m12.x;
    }
    double center_y = bisector12(center_x);

    circumcircle.center = Point{center_x, center_y};
    circumcircle.radius = std::hypot(p1->x - center_x, p1->y - center_y);
}

bool Triangle::includePoint(const PointPtr &p) const
{
    return std::pow(p->x - circumcircle.center.x, 2.) + std::pow(p->y - circumcircle.center.y, 2.) <
           std::pow(circumcircle.radius, 2.);
}

bool Triangle::includeEdge(const PointPtr &p1, const PointPtr &p2) const
{
    return this->includePoint(p1) && this->includePoint(p2);
}

std::ostream& operator<<(std::ostream &os, const Triangle &t)
{
    os << "\n- p1: " << t.p1 << "\n- p2: " << t.p2 << "\n- p3: " << t.p3;
    return os;
}

std::ostream& operator<<(std::ostream &os, const TrianglePtr &t)
{
    os << *t;
    return os;
}

bool operator==(const Triangle &lhs, const Triangle &rhs)
{
    return (lhs.p1 == rhs.p1 && lhs.p2 == rhs.p2 && lhs.p3 == rhs.p3) ||
           (lhs.p1 == rhs.p1 && lhs.p2 == rhs.p3 && lhs.p3 == rhs.p2) ||
           (lhs.p1 == rhs.p2 && lhs.p2 == rhs.p1 && lhs.p3 == rhs.p3) ||
           (lhs.p1 == rhs.p2 && lhs.p2 == rhs.p3 && lhs.p3 == rhs.p1) ||
           (lhs.p1 == rhs.p3 && lhs.p2 == rhs.p1 && lhs.p3 == rhs.p2) ||
           (lhs.p1 == rhs.p3 && lhs.p2 == rhs.p2 && lhs.p3 == rhs.p1);
}

bool operator==(const TrianglePtr &lhs, const TrianglePtr &rhs)
{
    if(!lhs || !rhs)
        throw std::invalid_argument("nullptr");
    return *lhs == *rhs;
}

} // namespace delaunay_triangulation