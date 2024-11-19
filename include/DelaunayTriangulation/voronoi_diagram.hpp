#ifndef DELAUNAY_TRIANGULATION__VORONOI_DIAGRAM_HPP
#define DELAUNAY_TRIANGULATION__VORONOI_DIAGRAM_HPP

#include <opencv2/opencv.hpp>


// Project
#include "DelaunayTriangulation/geometry_primitives.hpp"

namespace delaunay_triangulation
{

class VoronoiDiagram
{
public:
    static std::map<Vertex, std::vector<Vertex>> create(const std::vector<Triangle> & delaunay_triangles);
    static void draw(
        cv::Mat &img,
        const std::map<Vertex, std::vector<Vertex>> &voronoi_cells,
        const cv::Scalar &color = cv::Scalar(30, 105, 210));
};

} // namespace delaunay_Triangulation

#endif