#ifndef DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_DRAWER_HPP
#define DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_DRAWER_HPP

#include <opencv2/opencv.hpp>


// Project
#include "DelaunayTriangulation/geometry_primitives.hpp"
#include "DelaunayTriangulation/delaunay_triangulation.hpp"

namespace delaunay_triangulation
{

class DelaunayTriangulationDrawer
{
public:
    explicit DelaunayTriangulationDrawer(const DelaunayTriangulation &delaunay);
    void draw(cv::Mat &img);
    void switchDrawCircumCircles();
    void switchDrawSuperTriangles();
    void switchDrawVertexCoordinate();
    void switchFillTriangle();
    bool isFillTriangle() const { return fill_triangle_; }

private:
    const DelaunayTriangulation& delaunay_;
    const cv::Scalar white_ = cv::Scalar(255, 255, 255);

    bool draw_circum_circles_ = false;
    bool draw_super_triangles_ = false;
    bool draw_vertex_coordinate_ = false;
    bool fill_triangle_ = true;

    void drawVertices(cv::Mat &img);
    void drawTriangles(cv::Mat &img);
    void erase(const Triangle &t);
    void erase(const std::vector<Triangle> &triangles);
};

} // namespace delaunay_Triangulation

#endif