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

void DelaunayTriangulation::removeLastVertex()
{
    if (!vertices_.empty())
    {
        vertices_.pop_back();
    }
}

void DelaunayTriangulation::switchDrawCircumCircles(){
    draw_circum_circles_ = !draw_circum_circles_;
}

void DelaunayTriangulation::switchDrawSuperTriangles(){
    draw_super_triangles_ = !draw_super_triangles_;
}

void DelaunayTriangulation::switchDrawVertexCoordinate(){
    draw_vertex_coordinate_ = !draw_vertex_coordinate_;
}

void DelaunayTriangulation::reset()
{
    triangles_.clear();
}

void DelaunayTriangulation::createDelaunayTriangles(cv::Mat &img)
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
                if (t.isInCircumCircle(v))
                {
                    containing_triangles.push_back(t);
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

            std::cout << "num containing triangles: " << containing_triangles.size() << std::endl;
            // Delete the triangles that contain the vertex
            for (const auto &t : containing_triangles)
            {
                std::cout << "erasing triangle: " << t << std::endl;
                this->erase(t);
            }

            // Create new triangles in counterclockwise order
            for (size_t i = 0, n = polygon_vertices.size(); i < n; ++i)
            {
                const auto &v1 = polygon_vertices.at(i);
                const auto &v2 = polygon_vertices.at((i + 1) % polygon_vertices.size());
                triangles_.emplace_back(v1, v2, v);
            }

        }
    }
    if (!draw_super_triangles_)
    {
        this->deleteSuperTriangle();
    }
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
        v.draw(img, draw_vertex_coordinate_);
    }
}

void DelaunayTriangulation::drawTriangles(cv::Mat &img)
{
    for (const auto &t : triangles_)
    {
        if (draw_super_triangles_ && isSuperTriangle(t))
        {
            const cv::Scalar green{0, 255, 0};
            t.draw(img, draw_circum_circles_, green);
        }
        else
        {
            t.draw(img, draw_circum_circles_);
        }
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
    // Remove triangles that contain the super triangle vertices
    triangles_.erase(
        std::remove_if(triangles_.begin(), triangles_.end(),
            [this](const Triangle& triangle) {
                return this->isSuperTriangle(triangle);
            }),
        triangles_.end()
    );
    super_triangle_vertices_.fill(Vertex{0, 0});
}

bool DelaunayTriangulation::isSuperTriangle(const Triangle &t) const
{
    for (const auto &v : super_triangle_vertices_)
    {
        if (t.has(v))
        {
            return true;
        }
    }
    return false;
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
    if (polygon_vertices.empty()) {
        return;
    }

    // centroid of the polygon
    const auto centroid = std::accumulate(polygon_vertices.begin(), polygon_vertices.end(), Vertex{0, 0}) / static_cast<double>(polygon_vertices.size());

    struct VertexWithAngle {
        Vertex vertex;
        double angle;
    };

    std::vector<VertexWithAngle> vertices_with_angles;
    vertices_with_angles.reserve(polygon_vertices.size());

    for (const auto& v : polygon_vertices) {
        const double angle = std::atan2(v.y - centroid.y, v.x - centroid.x);
        vertices_with_angles.push_back({v, angle});
    }

    // Sort the vertices by angle
    std::sort(vertices_with_angles.begin(), vertices_with_angles.end(),
        [](const VertexWithAngle& a, const VertexWithAngle& b) {
            return a.angle < b.angle;
        });

    // Replace the original vertices with the sorted vertices
    for (size_t i = 0; i < polygon_vertices.size(); ++i) {
        polygon_vertices[i] = vertices_with_angles[i].vertex;
    }
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