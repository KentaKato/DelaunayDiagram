#ifndef DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_HPP
#define DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_HPP

#include <opencv2/opencv.hpp>


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
    void createDelaunayTriangles();
    std::vector<Triangle> getTriangles() const;
    std::vector<Triangle> getSuperTriangles() const;
    std::vector<Triangle> getAllTriangles() const;
    std::vector<Vertex> getVertices() const { return vertices_; }

private:
    std::vector<Triangle> triangles_;
    std::vector<Vertex> vertices_;
    std::vector<Vertex> super_triangle_vertices_;

    void reset();
    void erase(const Triangle &t);
    void erase(const std::vector<Triangle> &triangles);
    void setupSuperTriangle();
    bool isSuperTriangle(const Triangle &t) const;
    std::vector<Edge> parseUnsharedEdges(const std::vector<Triangle> &triangles) const;

};

} // namespace delaunay_Triangulation

#endif