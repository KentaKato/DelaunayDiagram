#include <iostream>
#include <opencv2/opencv.hpp>

#include "DelaunayDiagram/delaunay_diagram.h"

namespace delaunay_diagram
{

DelaunayDiagram::DelaunayDiagram()
    : img_(cv::Mat(image_height_, image_width_, CV_8UC3, background_color_))
{
    points_.push_back({100, 100});
    points_.push_back({400, 105});
    points_.push_back({100, 200});
    points_.push_back({400, 205});
    points_.push_back({100, 300});

    edges_.push_back({points_[0], points_[1]});
    edges_.push_back({points_[2], points_[3]});
    edges_.push_back({points_[4], points_[0]});
}

void DelaunayDiagram::createDelaunayDiagram()
{

}

void DelaunayDiagram::draw()
{
    this->drawPoints();
    this->drawEdges();

    cv::namedWindow("drawing", cv::WINDOW_AUTOSIZE|cv::WINDOW_FREERATIO);
    cv::imshow("drawing", img_);
    cv::waitKey(0);
}

void DelaunayDiagram::drawPoint(const Pont &p)
{
    cv::circle(img_, cv::Point(p.x, p.y), point_radius_, point_color_, -1, cv::LINE_AA);
}

void DelaunayDiagram::drawPoints()
{
    for (const auto &p : points_)
    {
        drawPoint(p);
    }
}

void DelaunayDiagram::drawEdge(const Edge &e)
{
    cv::line(img_, cv::Point(e.p1.x, e.p1.y), cv::Point(e.p2.x, e.p2.y), edge_color_, edge_thickness_, cv::LINE_AA);

}

void DelaunayDiagram::drawEdges()
{
    for (const auto &e : edges_)
    {
        drawEdge(e);
    }
}

} // namespace delaunay_diagram