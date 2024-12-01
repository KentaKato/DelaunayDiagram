#ifndef DELAUNAY_TRIANGULATION__VERTEXED_POLYGONS_HPP
#define DELAUNAY_TRIANGULATION__VERTEXED_POLYGONS_HPP

#include <cstddef>
#include <string>
#include <vector>
#include <cmath>
#include <random>

#include "DelaunayTriangulation/geometry_primitives.hpp"

namespace {
    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    std::normal_distribution<double> dist(0.0, 2.0);
}

namespace delaunay_triangulation
{

class VertexedPolygonBase {
public:
    using SharedPtr = std::shared_ptr<VertexedPolygonBase>;

    explicit VertexedPolygonBase(const std::string &name, const Vertex &center)
        : name_(name), center_(center) {}
    virtual ~VertexedPolygonBase() {}

    const std::string& name() const { return name_; }
    const std::vector<Vertex>& vertices() { return vertices_; }
    const Vertex& center() { return center_; }

    bool has_vertex(const Vertex& v) const
    {
        for (const auto& vertex : vertices_)
        {
            if (vertex == v)
            {
                return true;
            }
        }
        return false;
    }

    virtual void draw(cv::Mat& image, const cv::Scalar& color) const = 0;

protected:
    std::string name_;
    Vertex center_;
    std::vector<Vertex> vertices_;
};

class VertexedRectangle : public VertexedPolygonBase {
public:
    using SharedPtr = std::shared_ptr<VertexedRectangle>;

    explicit VertexedRectangle(
        const std::string &name,
        const Vertex& center,
        const double width,
        const double height,
        const double angle_deg = 0.0,
        const double interval_length = 5.0,
        const bool noise_enabled = false)
        : VertexedPolygonBase(name, center)
    {

        const double half_w = width / 2.0;
        const double half_h = height / 2.0;

        std::vector<Vertex> corners = {
            Vertex(-half_w, -half_h),
            Vertex(half_w, -half_h),
            Vertex(half_w, half_h),
            Vertex(-half_w, half_h)
        };

        // degree to radian
        const double rad = angle_deg * M_PI / 180.0;
        const double cos_a = cos(rad);
        const double sin_a = sin(rad);

        std::vector<Vertex> rotated_corners;
        rotated_corners.reserve(corners.size());
        for (const auto& corner : corners) {
            double x_rot = corner.x * cos_a - corner.y * sin_a;
            double y_rot = corner.x * sin_a + corner.y * cos_a;
            rotated_corners.emplace_back(center_.x + x_rot, center_.y + y_rot);
        }
        corners_ = rotated_corners;

        // Add vertices on each edge
        for (size_t i = 0; i < 4; ++i) {
            const Vertex& start = rotated_corners[i];
            const Vertex& end = rotated_corners[(i + 1) % 4];

            const double dx = end.x - start.x;
            const double dy = end.y - start.y;
            const double edge_length = std::sqrt(dx * dx + dy * dy);

            size_t num_intervals = std::max(1, static_cast<int>(std::ceil(edge_length / interval_length)));

            // Add vertices on the edge
            for (size_t j = 0; j < num_intervals; ++j) {
                const double t = static_cast<double>(j) / num_intervals;
                const double x = start.x + t * dx + dist(engine) * noise_enabled;
                const double y = start.y + t * dy + dist(engine) * noise_enabled;
                vertices_.emplace_back(x, y);
            }
        }
    }

    void draw(cv::Mat& image, const cv::Scalar& color) const override
    {
        std::vector<cv::Point> points;
        for (const auto& vertex : vertices_) {
            points.emplace_back(cv::Point(static_cast<int>(vertex.x), static_cast<int>(vertex.y)));
        }
        if (!points.empty()) {
            cv::polylines(image, points, true, color, 2, cv::LINE_AA);
        }
    }

    static SharedPtr create_ptr(
        const std::string &name,
        const Vertex& center,
        const double width,
        const double height,
        const double angle_deg = 0.0,
        const double interval_length = 5.0)
    {
        return std::make_shared<VertexedRectangle>(name, center, width, height, angle_deg, interval_length);
    }

private:
    std::vector<Vertex> corners_;
};

class VertexedTriangle : public VertexedPolygonBase {
public:
    using SharedPtr = std::shared_ptr<VertexedTriangle>;

    explicit VertexedTriangle(
        const std::string &name,
        const Vertex& center,
        const double size,
        const double angle_deg = 0.0,
        const double interval_length = 5.0,
        const bool noise_enabled = false)
        : VertexedPolygonBase(name, center)
    {

        const double h = size * std::sqrt(3) / 2.0;

        // Vertices before rotation
        std::vector<Vertex> corners = {
            Vertex(-size / 2.0, -h / 3.0),
            Vertex(size / 2.0, -h / 3.0),
            Vertex(0.0, 2.0 * h / 3.0)
        };

        // degree to radian
        const double rad = angle_deg * M_PI / 180.0;
        const double cos_a = cos(rad);
        const double sin_a = sin(rad);

        // Rotate vertices
        std::vector<Vertex> rotated_corners;
        for (const auto& corner : corners) {
            const double x_rot = corner.x * cos_a - corner.y * sin_a;
            const double y_rot = corner.x * sin_a + corner.y * cos_a;
            rotated_corners.emplace_back(center_.x + x_rot, center_.y + y_rot);
        }
        corners_ = rotated_corners;

        // Add vertices on each edge
        for (int i = 0; i < 3; ++i) {
            const Vertex& start = rotated_corners[i];
            const Vertex& end = rotated_corners[(i + 1) % 3];

            const double dx = end.x - start.x;
            const double dy = end.y - start.y;
            const double edge_length = std::sqrt(dx * dx + dy * dy);

            int num_intervals = std::max(1, static_cast<int>(std::ceil(edge_length / interval_length)));

            // Add vertices on the edge
            for (int j = 0; j < num_intervals; ++j) {
                double t = static_cast<double>(j) / num_intervals;
                double x = start.x + t * dx + dist(engine) * noise_enabled;
                double y = start.y + t * dy + dist(engine) * noise_enabled;
                vertices_.emplace_back(x, y);
            }
        }
    }

    void draw(cv::Mat& image, const cv::Scalar& color) const override
    {
        std::vector<cv::Point> points;
        for (const auto& vertex : vertices_) {
            points.emplace_back(cv::Point(static_cast<int>(vertex.x), static_cast<int>(vertex.y)));
        }
        if (!points.empty()) {
            cv::polylines(image, points, true, color, 2, cv::LINE_AA);
        }
    }

    static SharedPtr create_ptr(
        const std::string &name,
        const Vertex& center,
        const double size,
        const double angle_deg = 0.0,
        const double interval_length = 5.0)
    {
        return std::make_shared<VertexedTriangle>(name, center, size, angle_deg, interval_length);
    }

private:
    std::vector<Vertex> corners_;
};

class VertexedCircle : public VertexedPolygonBase {
public:
    using SharedPtr = std::shared_ptr<VertexedCircle>;

    explicit VertexedCircle(
        const std::string &name,
        const Vertex& center,
        const double radius,
        const double interval_length = 1.0,
        const bool noise_enabled = false)
        : VertexedPolygonBase(name, center), radius_(radius)
    {

        const double circumference = 2.0 * M_PI * radius;
        const int num_vertices = std::max(3, static_cast<int>(std::ceil(circumference / interval_length)));

        for (int i = 0; i < num_vertices; ++i) {
            const double theta = 2.0 * M_PI * i / num_vertices;
            const double x = center_.x + radius * cos(theta) + dist(engine) * noise_enabled;
            const double y = center_.y + radius * sin(theta) + dist(engine) * noise_enabled;
            vertices_.emplace_back(x, y);
        }
    }

    void draw(cv::Mat& image, const cv::Scalar& color) const override {
        std::vector<cv::Point> points;
        for (const auto& vertex : vertices_) {
            points.emplace_back(cv::Point(static_cast<int>(vertex.x), static_cast<int>(vertex.y)));
        }
        if (!points.empty()) {
            cv::polylines(image, points, true, color, 2, cv::LINE_AA);
        }
    }

    static SharedPtr create_ptr(
        const std::string &name,
        const Vertex& center,
        const double radius,
        const double interval_length = 5.0)
    {
        return std::make_shared<VertexedCircle>(name, center, radius, interval_length);
    }

private:
    double radius_;

};

} // namespace delaunay_triangulation

#endif