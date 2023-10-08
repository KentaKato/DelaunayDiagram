#ifndef DELAUNAY_TRIANGULATION__GEOMETRY_PRIMITIVES_H
#define DELAUNAY_TRIANGULATION__GEOMETRY_PRIMITIVES_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <stack>
#include <iostream>

namespace delaunay_triangulation
{

class Point
{
public:
    explicit Point(double x, double y) : x(x), y(y) {}
    void draw(cv::Mat &img) const;

    double x;
    double y;

private:
    int point_radius_ = 2;
    cv::Scalar point_color_ = cv::Scalar(200, 0, 0);
};

bool operator==(const Point &lhs, const Point &rhs);
std::ostream& operator<<(std::ostream &os, const Point &p);

using PointPtr = std::shared_ptr<Point>;
bool operator==(const PointPtr &lhs, const PointPtr &rhs);
std::ostream& operator<<(std::ostream &os, const PointPtr &p);

class Edge
{
public:
    Edge(const PointPtr &p1, const PointPtr &p2) : p1(p1), p2(p2) {}
    void draw(cv::Mat &img) const;

    PointPtr p1;
    PointPtr p2;
};

class Circle
{
public:
    Circle(){};
    explicit Circle(const Point center, double radius)
        : center(center), radius(radius) {}
    void draw(cv::Mat &img, bool draw_center = false) const;

    Point center = Point{0, 0};
    double radius = 0.0;

private:
    cv::Scalar circle_color_ = cv::Scalar(0, 0, 200);
    cv::Scalar center_color_ = cv::Scalar(0, 0, 200);
};

class Triangle
{
public:
    explicit Triangle(const PointPtr &p1,
                      const PointPtr &p2,
                      const PointPtr &p3);

    void computeCircumcircle();
    void draw(cv::Mat &img, bool draw_circumcircle=false) const;
    [[nodiscard]] bool includePoint(const PointPtr &p) const;
    [[nodiscard]] bool hasEdge(const Edge &e) const;
    [[nodiscard]] bool hasPoint(const PointPtr &p) const;

    friend std::ostream& operator<<(std::ostream &os, const Triangle &t);

    PointPtr p1;
    PointPtr p2;
    PointPtr p3;
    Circle circumcircle;

private:
    void validate() const;

    cv::Scalar triangle_color_ = cv::Scalar(0, 0, 0);
};
bool operator==(const Triangle &lhs, const Triangle &rhs);

using TrianglePtr = std::shared_ptr<Triangle>;
bool operator==(const TrianglePtr &lhs, const TrianglePtr &rhs);
std::ostream& operator<<(std::ostream &os, const TrianglePtr &t);

} // namespace delaunay_Triangulation

#endif