#include <opencv2/opencv.hpp>
#include <map>

#include "DelaunayTriangulation/voronoi_diagram.hpp"
#include "DelaunayTriangulation/geometry_primitives.hpp"


namespace delaunay_triangulation
{

std::map<Vertex, std::vector<Vertex>> VoronoiDiagram::create(const std::vector<Triangle> & delaunay_triangles)
{
    std::map <Vertex, std::vector<Vertex>> voronoi_cells;

    // Store circumcenters around each vertex
    for (const auto &t : delaunay_triangles)
    {
        voronoi_cells[t.v1].push_back(t.circum_circle.center);
        voronoi_cells[t.v2].push_back(t.circum_circle.center);
        voronoi_cells[t.v3].push_back(t.circum_circle.center);
    }

    // Sort circumcenters in counterclockwise order
    for (auto& [vertex, circumcenters] : voronoi_cells) {

        std::sort(circumcenters.begin(), circumcenters.end(),
            [&vertex](const Vertex& a, const Vertex& b) {
                double angle_a = atan2(a.y - vertex.y, a.x - vertex.x);
                double angle_b = atan2(b.y - vertex.y, b.x - vertex.x);
                return angle_a < angle_b;
            }
        );
    }

    return voronoi_cells;
}

void VoronoiDiagram::draw(
    cv::Mat &img,
    const std::map<Vertex, std::vector<Vertex>> &voronoi_cells,
    const cv::Scalar &color)
{
    for (const auto& [vertex, circumcenters] : voronoi_cells) {
        std::vector<cv::Point> points;
        for (const auto& circumcenter : circumcenters) {
            points.push_back(cv::Point(circumcenter.x, circumcenter.y));
            cv::circle(img, cv::Point(circumcenter.x, circumcenter.y), 4, color, -1);
        }
        cv::polylines(img, points, true, color, 1, cv::LINE_AA);
    }
}

} // namespace delaunay_Triangulation