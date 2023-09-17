#include <iostream>
#include <stack>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.h"

namespace delaunay_triangulation
{

DelaunayTriangulation::DelaunayTriangulation()
    : img_(cv::Mat(image_height_, image_width_, CV_8UC3, background_color_))
{
    this->addPoint(110, 100);
    this->addPoint(430, 105);
    this->addPoint(150, 230);
    this->addPoint(400, 205);
    this->addPoint(200, 330);

    edges_.emplace_back(points_[0], points_[1]);
    edges_.emplace_back(points_[2], points_[3]);
    edges_.emplace_back(points_[4], points_[0]);

    triangles_.emplace_back(points_[0], points_[1], points_[2]);
    triangles_.emplace_back(points_[0], points_[2], points_[4]);
    this->computeCircumcircle(triangles_[0], triangles_[0].circumcircle);
    this->computeCircumcircle(triangles_[1], triangles_[1].circumcircle);
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

}

void DelaunayTriangulation::draw()
{
    this->drawEdges();
    this->drawPoints();
    this->drawCircumcircles();

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
    cv::line(img_, cv::Point(e.p1->x, e.p1->y), cv::Point(e.p2->x, e.p2->y),
             edge_color_, edge_thickness_, cv::LINE_AA);

}

void DelaunayTriangulation::drawEdges()
{
    for (const auto &e : edges_)
    {
        drawEdge(e);
    }
}

void DelaunayTriangulation::drawCircumcircle(const Triangle &t)
{
    cv::circle(img_, cv::Point(t.circumcircle.center.x, t.circumcircle.center.y),
               t.circumcircle.radius, circumcircle_color_, circumcircle_thickness_, cv::LINE_AA);
}

void DelaunayTriangulation::drawCircumcircles()
{
    for (const auto &t : triangles_)
    {
        drawCircumcircle(t);
    }
}

void DelaunayTriangulation::addBoundingTriangle()
{
    const PointPtr p1 = std::make_shared<Point>(-1e9, -1e9);
    const PointPtr p2 = std::make_shared<Point>(1e9, -1e9);
    const PointPtr p3 = std::make_shared<Point>(0, 1e9);
    triangles_.emplace_back(p1, p2, p3);
}

void DelaunayTriangulation::computeCircumcircle(const Triangle &t, Circle &c)
{
    // Lambda function to compute the midpoint between two points
    auto midpoint = [](const PointPtr &a, const PointPtr &b) -> Point
    {
        return Point{(a->x + b->x) / 2.0, (a->y + b->y) / 2.0};
    };

    // Lambda function to compute the slope of the line between two points
    auto slope = [](const PointPtr &a, const PointPtr &b) -> double
    {
        if (b->x == a->x)
        {
            // vertical line case
            return INFINITY;
        }
        return (b->y - a->y) / (b->x - a->x);
    };
    // midpoints for two edges of the triangle
    const Point m12 = midpoint(t.p1, t.p2);
    const Point m13 = midpoint(t.p1, t.p3);

    // slopes for two edges of the triangle
    const double slope12 = slope(t.p1, t.p2);
    const double slope13 = slope(t.p1, t.p3);

    // the slopes of the perpendicular bisectors
    double perpSlope12 = (slope12 == 0) ? INFINITY : (-1.0 / slope12);
    double perpSlope13 = (slope13 == 0) ? INFINITY : (-1.0 / slope13);

    // the intersection of the two perpendicular bisectors
    double x = (m13.y - m12.y - perpSlope13 * m13.x + perpSlope12 * m12.x) / (perpSlope12 - perpSlope13);
    double y = m12.y + perpSlope12 * (x - m12.x);

    c.center = Point{x, y};
    c.radius = std::sqrt(std::pow(t.p1->x - x, 2) + std::pow(t.p1->y - y, 2));
}

} // namespace delaunay_Triangulation