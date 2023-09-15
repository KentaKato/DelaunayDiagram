#ifndef DELAUNAY_DIAGRAM__DELAUNAY_DIAGRAM_H
#define DELAUNAY_DIAGRAM__DELAUNAY_DIAGRAM_H

#include <opencv2/opencv.hpp>

namespace delaunay_diagram
{

struct Pont
{
    int x;
    int y;
};

struct Triangle
{
    Pont p1;
    Pont p2;
    Pont p3;
};

struct Edge
{
    Pont p1;
    Pont p2;
};

class DelaunayDiagram
{
public:
    DelaunayDiagram();
    void createDelaunayDiagram();
    void draw();

private:
    const cv::Scalar background_color_ = cv::Scalar(255,255,255);
    const int image_width_ = 500;
    const int image_height_ = 500;

    std::vector<Triangle> triangles_;

    std::vector<Pont> points_;
    const double point_radius_ = 5.;
    const cv::Scalar point_color_ = cv::Scalar(200,0,0);

    std::vector<Edge> edges_;
    const double edge_thickness_ = 3.;
    const cv::Scalar edge_color_ = cv::Scalar(0,0,200);

    cv::Mat img_;

    void drawPoint(const Pont &p);
    void drawPoints();
    void drawEdge(const Edge &e);
    void drawEdges();


};

} // namespace delaunay_diagram

#endif