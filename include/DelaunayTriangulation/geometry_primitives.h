#ifndef DELAUNAY_Triangulation__DELAUNAY_Triangulation_H
#define DELAUNAY_Triangulation__DELAUNAY_Triangulation_H

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
    const int point_radius_ = 2;
    const cv::Scalar point_color_ = cv::Scalar(200, 0, 0);
};

bool operator==(const Point &lhs, const Point &rhs);
std::ostream& operator<<(std::ostream &os, const Point &p);

using PointPtr = std::shared_ptr<Point>;
bool operator==(const PointPtr &lhs, const PointPtr &rhs);
std::ostream& operator<<(std::ostream &os, const PointPtr &p);

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
    const cv::Scalar circle_color_ = cv::Scalar(0, 0, 200);
    const cv::Scalar center_color_ = cv::Scalar(0, 0, 200);
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
    [[nodiscard]] bool includeEdge(const PointPtr &p1, const PointPtr &p2) const;

    friend std::ostream& operator<<(std::ostream &os, const Triangle &t);

    const PointPtr p1;
    const PointPtr p2;
    const PointPtr p3;
    Circle circumcircle;

private:
    void validate() const;

    const cv::Scalar triangle_color_ = cv::Scalar(0, 0, 0);
};
bool operator==(const Triangle &lhs, const Triangle &rhs);

using TrianglePtr = std::shared_ptr<Triangle>;
bool operator==(const TrianglePtr &lhs, const TrianglePtr &rhs);
std::ostream& operator<<(std::ostream &os, const TrianglePtr &t);

} // namespace delaunay_Triangulation

#endif