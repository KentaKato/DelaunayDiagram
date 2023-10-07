#ifndef DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_H
#define DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <stack>
#include <iostream>

// Project
#include "DelaunayTriangulation/geometry_primitives.h"

namespace delaunay_triangulation
{

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

    std::vector<Triangle> triangles_;
    std::vector<PointPtr> points_;
    std::stack<Edge> edge_stack_;

    cv::Mat img_;

    void drawPoints();
    void drawTriangles();
    void drawCircumcircles();

    void addBoundingTriangle();
    PointPtr findUnsharedVertex(const Triangle &t1, const Triangle &t2) const;
    Edge findSharedEdge(const Triangle &t1, const Triangle &t2) const;
    void flip(const Triangle &t1, const Triangle &t2);

};

} // namespace delaunay_Triangulation

#endif