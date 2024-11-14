#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.hpp"

namespace delaunay_triangulation
{

DelaunayTriangulation::DelaunayTriangulation()
{
    this->addVertex(151, 150);
    this->addVertex(293, 160);
    this->addVertex(165, 273);
    this->addVertex(290, 290);
    this->addVertex(270, 263);
}

void DelaunayTriangulation::addVertex(double x, double y)
{
    vertices_.emplace_back(x, y);
}

void DelaunayTriangulation::addVertex(const Vertex &v)
{
    vertices_.push_back(v);
}

void DelaunayTriangulation::createDelaunayTriangles()
{
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
                }
            }
            return containing_triangles;
        };

        // std::cout << "Finding triangle that contains vertex" << std::endl;
        // this->draw();
        const auto containing_triangles = find_triangle_containing_vertex(v);
        if (containing_triangles.empty())
        {
            // std::cout << "No triangle contains the vertex: " << v << std::endl;
            continue;
        }
        if (containing_triangles.size() > 2)
        {
            throw std::runtime_error("More than 2 triangles contain the vertex");
        }

        if (containing_triangles.size() == 1)
        {
            const auto &t_contains_v = containing_triangles.at(0);
            this->erase(t_contains_v);
            // this->draw();
            triangles_.emplace_back(t_contains_v.v1, t_contains_v.v2, v);
            triangles_.emplace_back(t_contains_v.v2, t_contains_v.v3, v);
            triangles_.emplace_back(t_contains_v.v3, t_contains_v.v1, v);
            // this->draw();
        }
        if (containing_triangles.size() == 2)
        {
            const auto &t1 = containing_triangles.at(0);
            const auto &t2 = containing_triangles.at(1);
            const auto shared_vertices = this->findSharedVertices(t1, t2);
            const auto unshared_vertices = this->findUnsharedVertices(t1, t2);

            this->erase(t1);
            this->erase(t2);

            for (const auto &shared_v : shared_vertices)
            {
                if (shared_v != v)
                {
                    triangles_.emplace_back(shared_v, unshared_vertices.at(0), v);
                    triangles_.emplace_back(shared_v, unshared_vertices.at(1), v);
                }
            }
        }
    }
    this->deleteSuperTriangle();
}


void DelaunayTriangulation::erase(const Triangle &t)
{
    triangles_.erase(std::remove(triangles_.begin(), triangles_.end(), t),
                     triangles_.end());
}

void DelaunayTriangulation::draw()
{
    img_ = cv::Mat(image_height_, image_width_, CV_8UC3, background_color_);

    this->drawVertices();
    this->drawTriangles();

    cv::namedWindow("drawing", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
    cv::imshow("drawing", img_);
    cv::waitKey(0);
}

void DelaunayTriangulation::drawVertices()
{
    for (const auto &v : vertices_)
    {
        v.draw(img_);
    }
}

void DelaunayTriangulation::drawTriangles()
{
    for (const auto &t : triangles_)
    {
        t.draw(img_, false);
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