#ifndef DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_HPP
#define DELAUNAY_TRIANGULATION__DELAUNAY_TRIANGULATION_HPP

#include <cstddef>
#include <opencv2/opencv.hpp>


// Project
#include "DelaunayTriangulation/geometry_primitives.hpp"

namespace delaunay_triangulation
{

class DelaunayTriangulation
{
public:
    using Point = Vertex;

    DelaunayTriangulation();
    void addVertex(double x, double y);
    void addVertex(const Vertex &p);
    void removeLastVertex();
    bool hasVertex(const Vertex &v) const;
    void reserveVerticesVector(const size_t size);
    void createDelaunayTriangles();
    void clear();
    std::vector<Triangle> getTriangles() const { return triangles_without_super_triangles_; };
    std::vector<Triangle> getSuperTriangles() const;
    std::vector<Triangle> getAllTriangles() const;
    std::vector<Vertex> getVertices() const { return vertices_; }
    Vertex findNearestVertex(
        const Point & p,
        const std::optional<Vertex> seed_vertex_opt = std::nullopt) const;
    Vertex findNearestVertex(
        const Point & p,
        std::vector<Vertex> &trace,
        const std::optional<Vertex> seed_vertex_opt = std::nullopt) const;
    std::vector<Triangle> findTrianglesContainingVertex(const Vertex &v) const;

private:
    std::vector<Triangle> triangles_, triangles_without_super_triangles_;
    std::vector<Vertex> vertices_;
    std::vector<Vertex> super_triangle_vertices_;

    void reset();
    void erase(const Triangle &t);
    void erase(const std::vector<Triangle> &triangles);
    void setupSuperTriangle();
    bool isSuperTriangle(const Triangle &t) const;
    std::vector<Edge> parseUnsharedEdges(const std::vector<Triangle> &triangles) const;

    Vertex findNearestVertex(
        const Point & p,
        std::vector<Vertex> &trace,
        const std::optional<Vertex> seed_vertex_opt,
        const bool record_trace) const;

};

} // namespace delaunay_Triangulation

#endif