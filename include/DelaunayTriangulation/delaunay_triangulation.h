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

    bool operator==(const Point& other) const
    {
        return (x == other.x && y == other.y);
    }
};
std::ostream& operator<<(std::ostream &os, const Point &p);

using PointPtr = std::shared_ptr<Point>;
bool operator==(const PointPtr& lhs, const PointPtr& rhs);
std::ostream& operator<<(std::ostream &os, const PointPtr &p);

struct Edge
{
    explicit Edge(const PointPtr &p1, const PointPtr &p2) : p1(p1), p2(p2) {}
    PointPtr p1;
    PointPtr p2;
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
    explicit Triangle(const PointPtr &p1, const PointPtr &p2, const PointPtr &p3);

    void computeCircumcircle();
    [[nodiscard]] bool includePoint(const PointPtr &p) const;
    [[nodiscard]] bool includeEdge(const PointPtr &p1, const PointPtr &p2) const;

    friend std::ostream& operator<<(std::ostream &os, const Triangle &t);

    PointPtr p1;
    PointPtr p2;
    PointPtr p3;
    Circle circumcircle;

private:
    void validate() const;
};
using TrianglePtr = std::shared_ptr<Triangle>;
bool operator==(const Triangle &lhs, const Triangle &rhs);
bool operator==(const TrianglePtr &lhs, const TrianglePtr &rhs);

class DelaunayTriangulation
{
public:
    DelaunayTriangulation();
    void addPoint(double x, double y);
    void addPoint(const PointPtr &p);
    void createDelaunayTriangules();
    void draw() const;

private:
    const cv::Scalar background_color_ = cv::Scalar(255,255,255);
    const int image_width_ = 500;
    const int image_height_ = 500;


    const cv::Scalar circumcircle_color_ = cv::Scalar(100,100,100);
    const double circumcircle_thickness_ = 1.;
    std::vector<TrianglePtr> triangles_;

    std::vector<PointPtr> points_;
    const double point_radius_ = 5.;
    const cv::Scalar point_color_ = cv::Scalar(200,0,0);

    std::stack<Edge> edge_stack_;
    const double edge_thickness_ = 3.;
    const cv::Scalar edge_color_ = cv::Scalar(0,0,200);

    cv::Mat img_;

    void drawPoint(const PointPtr &p) const;
    void drawPoints() const;
    // void drawEdge(const Edge &e);
    // void drawEdges();
    void drawTriangle(const TrianglePtr &t) const;
    void drawTriangles() const;
    void drawCircumcircle(const TrianglePtr &t) const;
    void drawCircumcircles() const;

    void addBoundingTriangle();
    PointPtr findUnsharedVertex(const TrianglePtr &t1, const TrianglePtr &t2) const;
    Edge findSharedEdge(const TrianglePtr &t1, const TrianglePtr &t2) const;
    void flip(const TrianglePtr &t1, const TrianglePtr &t2);

};

} // namespace delaunay_Triangulation

#endif