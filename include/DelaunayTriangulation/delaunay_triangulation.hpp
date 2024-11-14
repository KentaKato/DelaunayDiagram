#ifndef DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_HPP
#define DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_HPP

#include <opencv2/opencv.hpp>
#include <array>


// Project
#include "DelaunayTriangulation/geometry_primitives.hpp"

namespace delaunay_triangulation
{

class DelaunayTriangulation
{
public:
    DelaunayTriangulation();
    void addVertex(double x, double y);
    void addVertex(const Vertex &p);
    void createDelaunayTriangles();
    void draw();

private:
    const cv::Scalar background_color_ = cv::Scalar(255,255,255);
    const int image_width_ = 500;
    const int image_height_ = 500;

    std::vector<Triangle> triangles_;
    std::vector<Vertex> vertices_;
    std::array<Vertex, 3> super_triangle_vertices_;

    cv::Mat img_;

    void drawVertices();
    void drawTriangles();
    void drawCircumCircles();
    void erase(const Triangle &t);

    void setupSuperTriangle();
    void deleteSuperTriangle();
    std::vector<Vertex> findSharedVertices(const Triangle &t1, const Triangle &t2) const;
    std::vector<Vertex> findUnsharedVertices(const Triangle &t1, const Triangle &t2) const;

};

} // namespace delaunay_Triangulation

#endif