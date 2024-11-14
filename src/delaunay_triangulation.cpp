#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.hpp"

#include <numeric>

namespace delaunay_triangulation
{

DelaunayTriangulation::DelaunayTriangulation()
{
}

void DelaunayTriangulation::addVertex(double x, double y)
{
    vertices_.emplace_back(x, y);
}

void DelaunayTriangulation::addVertex(const Vertex &v)
{
    vertices_.push_back(v);
    super_triangle_vertices_.fill(Vertex{0, 0});
}

void DelaunayTriangulation::reset()
{
    triangles_.clear();
}

void DelaunayTriangulation::createDelaunayTriangles()
{
    reset();

    if (vertices_.size() < 3)
    {
        return;
    }

    this->setupSuperTriangle();

    for (const auto &v : vertices_)
    {
        auto find_triangle_containing_vertex = [this](const Vertex &v) -> std::vector<Triangle>
        {
            std::vector<Triangle> containing_triangles;
            for (const auto &t : triangles_)
            {
                if (t.contains(v))
                {
                    containing_triangles.push_back(t);
                    std::cout << "Triangle contains the vertex: " << t << std::endl;
                }
            }
            return containing_triangles;
        };

        const auto containing_triangles = find_triangle_containing_vertex(v);
        if (containing_triangles.empty())
        {
            // std::cout << "No triangle contains the vertex: " << v << std::endl;
            continue;
        }

        if (containing_triangles.size() == 1)
        {
            const auto &t_contains_v = containing_triangles.at(0);
            this->erase(t_contains_v);
            triangles_.emplace_back(t_contains_v.v1, t_contains_v.v2, v);
            triangles_.emplace_back(t_contains_v.v2, t_contains_v.v3, v);
            triangles_.emplace_back(t_contains_v.v3, t_contains_v.v1, v);
        }
        if (containing_triangles.size() > 1)
        {
            std::vector<Vertex> polygon_vertices;
            this->parseUniqueVertices(containing_triangles, polygon_vertices);
            this->sortCounterClockwise(polygon_vertices);

            std::cout << "num triangles: " << triangles_.size() << std::endl;
            std::cout << "num containing vertices: " << containing_triangles.size() << std::endl;
            // Delete the triangles that contain the vertex
            for (const auto &t : containing_triangles)
            {
                this->erase(t);
            }
            std::cout << "num triangles: " << triangles_.size() << std::endl;

            // Create new triangles in counterclockwise order
            for (size_t i = 0, n = polygon_vertices.size(); i < n; ++i)
            {
                const auto &v1 = polygon_vertices.at(i);
                const auto &v2 = polygon_vertices.at((i + 1) % polygon_vertices.size());
                triangles_.emplace_back(v1, v2, v);
            }

        }
    }
    // this->deleteSuperTriangle();
}


void DelaunayTriangulation::erase(const Triangle &t)
{
    triangles_.erase(std::remove(triangles_.begin(), triangles_.end(), t),
                     triangles_.end());
}

void DelaunayTriangulation::draw(cv::Mat &img)
{
    this->drawVertices(img);
    this->drawTriangles(img);
}

void DelaunayTriangulation::drawVertices(cv::Mat &img)
{
    for (const auto &v : vertices_)
    {
        v.draw(img);
    }
}

void DelaunayTriangulation::drawTriangles(cv::Mat &img)
{
    for (const auto &t : triangles_)
    {
        // t.draw(img, true);
        t.draw(img, false);
    }
}

void DelaunayTriangulation::setupSuperTriangle()
{
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    for (const auto &v : vertices_)
    {
        if (v.x < min_x) min_x = v.x;
        if (v.x > max_x) max_x = v.x;
        if (v.y < min_y) min_y = v.y;
        if (v.y > max_y) max_y = v.y;
    }

    // Rectangle that contains all the points
    const double center_x = (min_x + max_x) / 2.0;
    const double height = max_y - min_y;
    const double width = max_x - min_x;

    const Vertex v1{center_x, min_y + 2.0 * height};
    const Vertex v2{center_x - 1.5 * width, min_y - 0.5 * height};
    const Vertex v3{center_x + 1.5 * width, min_y - 0.5 * height};
    super_triangle_vertices_[0] = v1;
    super_triangle_vertices_[1] = v2;
    super_triangle_vertices_[2] = v3;
    triangles_.emplace_back(v1, v2, v3);
}

void DelaunayTriangulation::deleteSuperTriangle()
{
    // en: Remove triangles that contain the super triangle vertices
    triangles_.erase(
        std::remove_if(triangles_.begin(), triangles_.end(),
            [this](const Triangle& triangle) {
                for (const auto& v : super_triangle_vertices_) {
                    if (triangle.has(v)) {
                        return true; // Remove the triangle
                    }
                }
                return false; // Keep the triangle
            }),
        triangles_.end()
    );
    super_triangle_vertices_.fill(Vertex{0, 0});
}

void DelaunayTriangulation::parseUniqueVertices(const std::vector<Triangle> &triangles, std::vector<Vertex> &unique_vertices) const
{
    unique_vertices.clear();
    for (const auto &t : triangles)
    {
        for (const auto &v : {t.v1, t.v2, t.v3})
        {
            if (std::find(
                    unique_vertices.begin(),
                    unique_vertices.end(),
                    v) == unique_vertices.end())
            {
                unique_vertices.push_back(v);
            }
        }
    }
}

void DelaunayTriangulation::sortCounterClockwise(std::vector<Vertex> &polygon_vertices) const
{
    const auto centroid = std::accumulate(polygon_vertices.begin(), polygon_vertices.end(), Vertex{0, 0}) / static_cast<double>(polygon_vertices.size());

    // Sort the vertices in counterclockwise order
    std::sort(
        polygon_vertices.begin(),
        polygon_vertices.end(),
        [centroid](const Vertex &v1, const Vertex &v2) -> bool {
            double dx1 = v1.x - centroid.x;
            double dy1 = v1.y - centroid.y;
            double dx2 = v2.x - centroid.x;
            double dy2 = v2.y - centroid.y;

            // Calculate the cross product
            double cross = dx1 * dy2 - dy1 * dx2;

            if (cross > 0)
                return true; // v1 comes before v2
            if (cross < 0)
                return false; // v1 comes after v2

            // TODO: Handle the case when cross product is zero properly
            // If cross product is zero, points are colinear with the centroid
            // In this case, sort by distance from the centroid
            double d1 = dx1 * dx1 + dy1 * dy1;
            double d2 = dx2 * dx2 + dy2 * dy2;
            return d1 < d2;
        });
}

std::vector<Vertex> DelaunayTriangulation::findSharedVertices(const Triangle &ref, const Triangle &target) const
{
    std::vector<Vertex> shared_vertices;
    for (const auto &v1 : {ref.v1, ref.v2, ref.v3})
    {
        for (const auto &v2 : {target.v1, target.v2, target.v3})
        {
            if (v1 == v2)
            {
                shared_vertices.push_back(v1);
            }
        }
    }
    if (shared_vertices.size() == 0)
    {
        throw std::runtime_error("No shared vertices found");
    }
    else if (shared_vertices.size() == 1)
    {
        throw std::runtime_error("Only one shared vertex found");
    }
    else if (shared_vertices.size() > 2)
    {
        throw std::runtime_error("More than two shared vertices found");
    }
    return shared_vertices;
};

std::vector<Vertex> DelaunayTriangulation::findUnsharedVertices(const Triangle &t1, const Triangle &t2) const
{
    std::vector<Vertex> unshared_points;

    // Search for t1's unshared vertices
    for (const auto &v1 : {t1.v1, t1.v2, t1.v3})
    {
        if (!(v1 == t2.v1 || v1 == t2.v2 || v1 == t2.v3))
        {
            unshared_points.push_back(v1);
        }
    }

    // Search for t2's unshared vertices
    for (const auto &v2 : {t2.v1, t2.v2, t2.v3})
    {
        if (!(v2 == t1.v1 || v2 == t1.v2 || v2 == t1.v3))
        {
            unshared_points.push_back(v2);
        }
    }

    // Validation
    if (unshared_points.size() == 0)
    {
        throw std::runtime_error("No unshared vertices found");
    }
    else if (unshared_points.size() == 1)
    {
        throw std::runtime_error("Only one unshared vertex found");
    }
    else if (unshared_points.size() > 2)
    {
        throw std::runtime_error("More than two unshared vertices found");
    }
    return unshared_points;
}

} // namespace delaunay_Triangulation