#ifndef DELAUNAY_Triangulation__DELAUNAY_Triangulation_H
#define DELAUNAY_Triangulation__DELAUNAY_Triangulation_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <stack>

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
using PointPtr = std::shared_ptr<Point>;
bool operator==(const PointPtr& lhs, const PointPtr& rhs)
{
    if(!lhs || !rhs)
        return lhs == rhs;
    return *lhs == *rhs;
}

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

    bool operator==(const Triangle& other) const
    {
        return (p1 == other.p1 && p2 == other.p2 && p3 == other.p3) ||
               (p1 == other.p1 && p2 == other.p3 && p3 == other.p2) ||
               (p1 == other.p2 && p2 == other.p1 && p3 == other.p3) ||
               (p1 == other.p2 && p2 == other.p3 && p3 == other.p1) ||
               (p1 == other.p3 && p2 == other.p1 && p3 == other.p2) ||
               (p1 == other.p3 && p2 == other.p2 && p3 == other.p1);
    }

    PointPtr p1;
    PointPtr p2;
    PointPtr p3;
    Circle circumcircle;

private:
    void validate() const;
};
using TrianglePtr = std::shared_ptr<Triangle>;
bool operator==(const TrianglePtr& lhs, const TrianglePtr& rhs)
{
    if(!lhs || !rhs)
        return lhs == rhs;
    return *lhs == *rhs;
}

class DelaunayTriangulation
{
public:
    DelaunayTriangulation();
    void addPoint(double x, double y);
    void addPoint(const PointPtr &p);
    void createDelaunayTriangules();
    void draw();

private:
    const cv::Scalar background_color_ = cv::Scalar(255,255,255);
    const int image_width_ = 500;
    const int image_height_ = 500;


    const cv::Scalar circumcircle_color_ = cv::Scalar(100,100,100);
    const double circumcircle_thickness_ = 1.;
    std::vector<Triangle> triangles_;

    std::vector<PointPtr> points_;
    const double point_radius_ = 5.;
    const cv::Scalar point_color_ = cv::Scalar(200,0,0);

    std::stack<Edge> edge_stack_;
    const double edge_thickness_ = 3.;
    const cv::Scalar edge_color_ = cv::Scalar(0,0,200);

    cv::Mat img_;

    void drawPoint(const PointPtr &p);
    void drawPoints();
    // void drawEdge(const Edge &e);
    // void drawEdges();
    void drawTriangle(const Triangle &t);
    void drawTriangles();
    void drawCircumcircle(const Triangle &t);
    void drawCircumcircles();
    void addBoundingTriangle();
    PointPtr findUnsharedVertex(const TrianglePtr &t1, const TrianglePtr &t2) const;
    Edge findSharedEdge(const TrianglePtr &t1, const TrianglePtr &t2) const;
    void flip(const TrianglePtr &t1, const TrianglePtr &t2);

};

} // namespace delaunay_Triangulation

#endif