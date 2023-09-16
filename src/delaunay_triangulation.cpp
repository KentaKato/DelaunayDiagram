#include <iostream>
#include <stack>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.h"

namespace delaunay_triangulation
{

DelaunayTriangulation::DelaunayTriangulation()
    : img_(cv::Mat(image_height_, image_width_, CV_8UC3, background_color_))
{
    this->addPoint(100, 100);
    this->addPoint(400, 105);
    this->addPoint(100, 200);
    this->addPoint(400, 205);
    this->addPoint(100, 300);

    edges_.emplace_back(points_[0], points_[1]);
    edges_.emplace_back(points_[2], points_[3]);
    edges_.emplace_back(points_[4], points_[0]);
}

void DelaunayTriangulation::addPoint(double x, double y)
{
    points_.push_back(std::make_shared<Point>(x, y));
}

void DelaunayTriangulation::addPoint(const PointPtr &p)
{
    points_.push_back(p);
}

void DelaunayTriangulation::createDelaunayTriangulation()
{
    this->addBoundingTriangle();

    for (const auto& point : points_)
    {
        std::vector<Triangle> new_triangles;
        std::stack<Edge> edge_stack;

}

void DelaunayTriangulation::draw()
{
    this->drawEdges();
    this->drawPoints();

    cv::namedWindow("drawing", cv::WINDOW_AUTOSIZE|cv::WINDOW_FREERATIO);
    cv::imshow("drawing", img_);
    cv::waitKey(0);
}

void DelaunayTriangulation::drawPoint(const PointPtr &p)
{
    cv::circle(img_, cv::Point(p->x, p->y), point_radius_, point_color_, -1, cv::LINE_AA);
}

void DelaunayTriangulation::drawPoints()
{
    for (const auto &p : points_)
    {
        drawPoint(p);
    }
}

void DelaunayTriangulation::drawEdge(const Edge &e)
{
    cv::line(img_, cv::Point(e.p1->x, e.p1->y), cv::Point(e.p2->x, e.p2->y), edge_color_, edge_thickness_, cv::LINE_AA);

}

void DelaunayTriangulation::drawEdges()
{
    for (const auto &e : edges_)
    {
        drawEdge(e);
    }
}

void DelaunayTriangulation::addBoundingTriangle()
{
    const PointPtr p1 = std::make_shared<Point>(-1e9, -1e9);
    const PointPtr p2 = std::make_shared<Point>(1e9, -1e9);
    const PointPtr p3 = std::make_shared<Point>(0, 1e9);
    triangles_.emplace_back(p1, p2, p3);
}

void DelaunaryTriangulation::computeCircumcircle(const Triangle &t, Circle &c)
{
}

} // namespace delaunay_Triangulation