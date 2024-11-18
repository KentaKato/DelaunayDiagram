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
    void removeLastVertex();
    void createDelaunayTriangles(cv::Mat &img);
    void draw(cv::Mat &img);
    void switchDrawCircumCircles();
    void switchDrawSuperTriangles();
    void switchDrawVertexCoordinate();

private:
    std::vector<Triangle> triangles_;
    std::vector<Vertex> vertices_;
    std::array<Vertex, 3> super_triangle_vertices_;
    bool draw_circum_circles_ = false;
    bool draw_super_triangles_ = false;
    bool draw_vertex_coordinate_ = false;

    void reset();

    void drawVertices(cv::Mat &img);
    void drawTriangles(cv::Mat &img);
    void erase(const Triangle &t);
    void erase(const std::vector<Triangle> &triangles);

    void setupSuperTriangle();
    void deleteSuperTriangle();
    bool isSuperTriangle(const Triangle &t) const;
    std::vector<Edge> parseUnsharedEdges(const std::vector<Triangle> &triangles) const;

};

} // namespace delaunay_Triangulation

#endif