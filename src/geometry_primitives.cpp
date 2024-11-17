#include <iostream>
#include <array>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/geometry_primitives.hpp"

namespace delaunay_triangulation
{

namespace // anonymous namespace
{
    bool equal(double a, double b, double epsilon = 1e-6)
    {
        return std::abs(a - b) < epsilon;
    }
}


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
    return equal(lhs.x,rhs.x) && equal(lhs.y,rhs.y);
}

std::ostream& operator<<(std::ostream &os, const Vertex &p)
{
    os << "(x, y) = (" << p.x << ", " << p.y << ")";
    return os;
}

Vertex operator+(const Vertex& lhs, const Vertex& rhs)
{
    return Vertex{lhs.x + rhs.x, lhs.y + rhs.y};
}

Vertex operator/(const Vertex& v, double scalar)
{
    if (equal(scalar, 0.0))
        throw std::invalid_argument("Division by zero");
    return Vertex{v.x / scalar, v.y / scalar};
}

void Vertex::draw(cv::Mat &img, const bool draw_coordinate_value) const
{
    cv::circle(img, cv::Point(x, y), radius_, color_, -1, cv::LINE_AA);

    if (draw_coordinate_value)
    {
        std::stringstream ss;
        ss << "(" << x << ", " << y << ")";
        std::string coordText = ss.str();

        // Font settings
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;

        // Get the text size
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(coordText, fontFace, fontScale, thickness, &baseline);
        baseline += thickness;

        // Calculate the position to draw the text
        cv::Point textOrg(x + radius_ + 5, y - radius_ - 5);

        // Check if the text fits in the image
        if (textOrg.x + textSize.width > img.cols)
            textOrg.x = img.cols - textSize.width;
        if (textOrg.y - textSize.height < 0)
            textOrg.y = textSize.height;

        cv::putText(img, coordText, textOrg, fontFace, fontScale, color_, thickness, cv::LINE_AA);
    }

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
        const double area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);

        constexpr double EPSILON = 1e-6;
        if (std::abs(area) < EPSILON)
        {
            throw std::runtime_error("Triangle is degenerate: area is zero");
        }

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
        constexpr double EPSILON = 1e-6;
        return std::abs(cross) < EPSILON;
    };

    if (v1 == v2 || v2 == v3 || v3 == v1)
        throw std::runtime_error("Triangle is not valid: duplicate vertices");

    if (are_collinear(v1, v2, v3))
        throw std::runtime_error("Triangle is not valid: collinear vertices");
}

void Triangle::computeCircumCircle()
{
    // 座標の差を計算
    double dA = v1.x * v1.x + v1.y * v1.y;
    double dB = v2.x * v2.x + v2.y * v2.y;
    double dC = v3.x * v3.x + v3.y * v3.y;

    double aux1 = (dA*(v3.y - v2.y) + dB*(v1.y - v3.y) + dC*(v2.y - v1.y));
    double aux2 = -(dA*(v3.x - v2.x) + dB*(v1.x - v3.x) + dC*(v2.x - v1.x));
    double div = (2*(v1.x*(v3.y - v2.y) + v2.x*(v1.y - v3.y) + v3.x*(v2.y - v1.y)));

    constexpr double EPSILON = 1e-6;
    if (std::abs(div) < EPSILON)
    {
        throw std::runtime_error("Cannot compute circumcircle: division by zero");
    }

    double center_x = aux1 / div;
    double center_y = aux2 / div;

    circum_circle.center.x = center_x;
    circum_circle.center.y = center_y;
    circum_circle.radius = std::hypot(v1.x - center_x, v1.y - center_y);
}


void Triangle::draw(cv::Mat &img, bool draw_circum_circle, const cv::Scalar &color) const
{
    const cv::Point p1(static_cast<int>(std::round(v1.x)), static_cast<int>(std::round(v1.y)));
    const cv::Point p2(static_cast<int>(std::round(v2.x)), static_cast<int>(std::round(v2.y)));
    const cv::Point p3(static_cast<int>(std::round(v3.x)), static_cast<int>(std::round(v3.y)));
    cv::line(img, p1, p2, color, 1, cv::LINE_AA);
    cv::line(img, p2, p3, color, 1, cv::LINE_AA);
    cv::line(img, p3, p1, color, 1, cv::LINE_AA);

    if (draw_circum_circle)
    {
        circum_circle.draw(img, true);
    }
}

bool Triangle::isInCircumCircle(const Vertex &v) const
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

bool operator==(const Triangle &lhs, const Triangle &rhs)
{
    return (lhs.v1 == rhs.v1 && lhs.v2 == rhs.v2 && lhs.v3 == rhs.v3) ||
           (lhs.v1 == rhs.v1 && lhs.v2 == rhs.v3 && lhs.v3 == rhs.v2) ||
           (lhs.v1 == rhs.v2 && lhs.v2 == rhs.v1 && lhs.v3 == rhs.v3) ||
           (lhs.v1 == rhs.v2 && lhs.v2 == rhs.v3 && lhs.v3 == rhs.v1) ||
           (lhs.v1 == rhs.v3 && lhs.v2 == rhs.v1 && lhs.v3 == rhs.v2) ||
           (lhs.v1 == rhs.v3 && lhs.v2 == rhs.v2 && lhs.v3 == rhs.v1);
}

} // namespace delaunay_triangulation