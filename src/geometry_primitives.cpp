#include <iostream>
#include <array>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/geometry_primitives.h"

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

void Point::draw(cv::Mat &img) const
{
    cv::circle(img, cv::Point(x, y), point_radius_, point_color_, -1, cv::LINE_AA);
}

void Circle::draw(cv::Mat &img, bool draw_center) const
{
    cv::circle(img, cv::Point(center.x, center.y), radius, circle_color_, 1, cv::LINE_AA);
    if (draw_center)
    {
        cv::circle(img, cv::Point(center.x, center.y), 2, center_color_, -1, cv::LINE_AA);
    }
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

    const Point m12 = midpoint(p1, p2);
    const Point m13 = midpoint(p1, p3);

    double center_x, center_y;
    if (p1->y == p2->y)
    {
        center_x = m12.x;
        const double perpendicular_slope13 = -(p3->x - p1->x) / (p3->y - p1->y);
        center_y = m13.y + perpendicular_slope13 * (center_x - m13.x);
    }
    else if(p1->y == p3->y)
    {
        center_x = m13.x;
        const double perpendicular_slope12 = -(p2->x - p1->x) / (p2->y - p1->y);
        center_y = m12.y + perpendicular_slope12 * (center_x - m12.x);
    }
    else
    {
        const double perpendicular_slope12 = -(p2->x - p1->x) / (p2->y - p1->y);
        const double perpendicular_slope13 = -(p3->x - p1->x) / (p3->y - p1->y);
        center_x = (m13.y - m12.y +
                    perpendicular_slope12 * m12.x -
                    perpendicular_slope13 * m13.x) /
                    (perpendicular_slope12 - perpendicular_slope13);
        center_y = m12.y + perpendicular_slope12 * (center_x - m12.x);
    }

    circumcircle.center.x = center_x;
    circumcircle.center.y = center_y;
    circumcircle.radius = std::hypot(p1->x - center_x, p1->y - center_y);
}


void Triangle::draw(cv::Mat &img, bool draw_circumcircle) const
{
    cv::line(img, cv::Point(p1->x, p1->y), cv::Point(p2->x, p2->y),
             triangle_color_, 1, cv::LINE_AA);
    cv::line(img, cv::Point(p2->x, p2->y), cv::Point(p3->x, p3->y),
             triangle_color_, 1, cv::LINE_AA);
    cv::line(img, cv::Point(p3->x, p3->y), cv::Point(p1->x, p1->y),
             triangle_color_, 1, cv::LINE_AA);

    if (draw_circumcircle)
    {
        circumcircle.draw(img, true);
    }
}

bool Triangle::includePoint(const PointPtr &p) const
{
    return std::pow(p->x - circumcircle.center.x, 2.) + std::pow(p->y - circumcircle.center.y, 2.) <
           std::pow(circumcircle.radius, 2.);
}

bool Triangle::hasPoint(const PointPtr &p) const
{
    return (p == p1 || p == p2 || p == p3);
}

bool Triangle::hasEdge(const Edge &e) const
{
    return this->hasPoint(e.p1) && this->hasPoint(e.p2);
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