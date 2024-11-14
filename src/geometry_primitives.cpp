#include <iostream>
#include <array>
#include <utility>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/geometry_primitives.hpp"

namespace delaunay_triangulation
{

Vertex& Vertex::operator=(Vertex &other)
{
    x = other.x;
    y = other.y;
    return *this;
}

Vertex& Vertex::operator=(const Vertex &other)
{
    x = other.x;
    y = other.y;
    return *this;
}

bool operator==(const Vertex &lhs, const Vertex &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

std::ostream& operator<<(std::ostream &os, const Vertex &p)
{
    os << "(x, y) = (" << p.x << ", " << p.y << ")";
    return os;
}

void Vertex::draw(cv::Mat &img) const
{
    cv::circle(img, cv::Point(x, y), radius_, color_, -1, cv::LINE_AA);
}

void Circle::draw(cv::Mat &img, bool draw_center) const
{
    cv::circle(img, cv::Point(center.x, center.y), radius, circle_color_, 1, cv::LINE_AA);
    if (draw_center)
    {
        cv::circle(img, cv::Point(center.x, center.y), 2, center_color_, -1, cv::LINE_AA);
    }
}

Triangle::Triangle(const Vertex &_v1, const Vertex &_v2, const Vertex &_v3)
{
    auto sort_points_counter_clockwise = [this, &_v1, &_v2, &_v3](){
        Vertex a = _v1;
        Vertex b = _v2;
        Vertex c = _v3;

        // If the points are not sorted counter clockwise, swap them
        double area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
        if (area < 0)
        {
            const auto tmp = b;
            b = c;
            c = tmp;
        }

        this->v1 = a;
        this->v2 = b;
        this->v3 = c;
    };

    sort_points_counter_clockwise();
    this->validate();
    this->computeCircumCircle();
}

void Triangle::validate() const
{
    auto are_collinear = [](const Vertex &a, const Vertex &b, const Vertex &c) -> bool
    {
        const std::array<double, 2> vec12 = {b.x - a.x, b.y - a.y};
        const std::array<double, 2> vec13 = {c.x - a.x, c.y - a.y};
        const double cross = vec12.at(0) * vec13.at(1) - vec12.at(1) * vec13.at(0);
        return cross == 0;
    };

    if (v1 == v2 || v2 == v3 || v3 == v1)
        throw std::runtime_error("Triangle is not valid: duplicate vertices");

    if (are_collinear(v1, v2, v3))
        throw std::runtime_error("Triangle is not valid: collinear vertices");
}

void Triangle::computeCircumCircle()
{
    auto midpoint = [](const Vertex &a, const Vertex &b) -> Vertex
    {
        return Vertex{(a.x + b.x) / 2.0, (a.y + b.y) / 2.0};
    };

    const Vertex m12 = midpoint(v1, v2);
    const Vertex m13 = midpoint(v1, v3);

    double center_x, center_y;
    if (v1.y == v2.y)
    {
        center_x = m12.x;
        const double perpendicular_slope13 = -(v3.x - v1.x) / (v3.y - v1.y);
        center_y = m13.y + perpendicular_slope13 * (center_x - m13.x);
    }
    else if(v1.y == v3.y)
    {
        center_x = m13.x;
        const double perpendicular_slope12 = -(v2.x - v1.x) / (v2.y - v1.y);
        center_y = m12.y + perpendicular_slope12 * (center_x - m12.x);
    }
    else
    {
        const double perpendicular_slope12 = -(v2.x - v1.x) / (v2.y - v1.y);
        const double perpendicular_slope13 = -(v3.x - v1.x) / (v3.y - v1.y);
        center_x = (m13.y - m12.y +
                    perpendicular_slope12 * m12.x -
                    perpendicular_slope13 * m13.x) /
                    (perpendicular_slope12 - perpendicular_slope13);
        center_y = m12.y + perpendicular_slope12 * (center_x - m12.x);
    }

    circum_circle.center.x = center_x;
    circum_circle.center.y = center_y;
    circum_circle.radius = std::hypot(v1.x - center_x, v1.y - center_y);
}


void Triangle::draw(cv::Mat &img, bool draw_circum_circle) const
{
    cv::line(img, cv::Point(v1.x, v1.y), cv::Point(v2.x, v2.y),
             triangle_color_, 1, cv::LINE_AA);
    cv::line(img, cv::Point(v2.x, v2.y), cv::Point(v3.x, v3.y),
             triangle_color_, 1, cv::LINE_AA);
    cv::line(img, cv::Point(v3.x, v3.y), cv::Point(v1.x, v1.y),
             triangle_color_, 1, cv::LINE_AA);

    if (draw_circum_circle)
    {
        circum_circle.draw(img, true);
    }
}

bool Triangle::contains(const Vertex &v) const
{
    double ax = v1.x - v.x;
    double ay = v1.y - v.y;
    double bx = v2.x - v.x;
    double by = v2.y - v.y;
    double cx = v3.x - v.x;
    double cy = v3.y - v.y;

    double det = (ax * ax + ay * ay) * (bx * cy - cx * by) -
                 (bx * bx + by * by) * (ax * cy - cx * ay) +
                 (cx * cx + cy * cy) * (ax * by - bx * ay);

    // In the triangle is counterclockwise
    return det > 0;
}

bool Triangle::has(const Vertex &v) const
{
    return (v == v1 || v == v2 || v == v3);
}

bool Triangle::has(const Edge &e) const
{
    return this->has(e.v1) && this->has(e.v2);
}

std::ostream& operator<<(std::ostream &os, const Triangle &t)
{
    os << "\n- v1: " << t.v1 << "\n- v2: " << t.v2 << "\n- v3: " << t.v3;
    return os;
}

std::ostream& operator<<(std::ostream &os, const TrianglePtr &t)
{
    os << *t;
    return os;
}

bool operator==(const Triangle &lhs, const Triangle &rhs)
{
    return (lhs.v1 == rhs.v1 && lhs.v2 == rhs.v2 && lhs.v3 == rhs.v3) ||
           (lhs.v1 == rhs.v1 && lhs.v2 == rhs.v3 && lhs.v3 == rhs.v2) ||
           (lhs.v1 == rhs.v2 && lhs.v2 == rhs.v1 && lhs.v3 == rhs.v3) ||
           (lhs.v1 == rhs.v2 && lhs.v2 == rhs.v3 && lhs.v3 == rhs.v1) ||
           (lhs.v1 == rhs.v3 && lhs.v2 == rhs.v1 && lhs.v3 == rhs.v2) ||
           (lhs.v1 == rhs.v3 && lhs.v2 == rhs.v2 && lhs.v3 == rhs.v1);
}

bool operator==(const TrianglePtr &lhs, const TrianglePtr &rhs)
{
    if(!lhs || !rhs)
        throw std::invalid_argument("nullptr");
    return *lhs == *rhs;
}

} // namespace delaunay_triangulation