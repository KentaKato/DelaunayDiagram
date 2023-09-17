#ifndef DELAUNAY_Triangulation__DELAUNAY_Triangulation_H
#define DELAUNAY_Triangulation__DELAUNAY_Triangulation_H

#include <opencv2/opencv.hpp>
#include <memory>

namespace delaunay_triangulation
{

struct Point
{
    explicit Point(double x, double y) : x(x), y(y) {}
    double x;
    double y;
};
using PointPtr = std::shared_ptr<Point>;

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
    double radius;
};

struct Triangle
{
    explicit Triangle(const PointPtr &p1, const PointPtr &p2, const PointPtr &p3)
        : p1(p1), p2(p2), p3(p3) {}
    PointPtr p1;
    PointPtr p2;
    PointPtr p3;
    Circle circumcircle;
};


class DelaunayTriangulation
{
public:
    DelaunayTriangulation();
    void addPoint(double x, double y);
    void addPoint(const PointPtr &p);
    void createDelaunayTriangulation();
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

    std::vector<Edge> edges_;
    const double edge_thickness_ = 3.;
    const cv::Scalar edge_color_ = cv::Scalar(0,0,200);

    cv::Mat img_;

    void drawPoint(const PointPtr &p);
    void drawPoints();
    void drawEdge(const Edge &e);
    void drawEdges();
    void drawCircumcircle(const Triangle &t);
    void drawCircumcircles();
    void addBoundingTriangle();
    void computeCircumcircle(const Triangle &t, Circle &c);

};

} // namespace delaunay_Triangulation

#endif