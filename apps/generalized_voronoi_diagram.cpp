#include "DelaunayTriangulation/vertexed_polygons.hpp"
#include "DelaunayTriangulation/geometry_primitives.hpp"
#include "DelaunayTriangulation/delaunay_triangulation.hpp"
#include "DelaunayTriangulation/delaunay_triangulation_drawer.hpp"
#include "DelaunayTriangulation/voronoi_diagram.hpp"

#include <cstdlib>

namespace delaunay_triangulation
{

class GeneralizedVoronoiDiagram
{

public:
    explicit GeneralizedVoronoiDiagram(
        const int image_width,
        const int image_height,
        const std::vector<VertexedPolygonBase::SharedPtr> &polygons)
        :  image_width_(image_width),
          image_height_(image_height),
          polygons_(polygons),
          delaunay_(),
          drawer_(delaunay_)
    {
        img_ = cv::Mat(image_height, image_width, CV_8UC3, bg_color_);

        for (const auto &polygon : polygons_)
        {
            for (const auto &vertex : polygon->vertices())
            {
                delaunay_.addVertex(vertex);
            }
        }

        delaunay_.createDelaunayTriangles();
    }

    void run()
    {
        drawer_.setFillTriangle(false);
        draw_polygons();
        cv::imshow("Generalized Voronoi Diagram ", img_);
        cv::waitKey(0);

        drawer_.draw(img_);
        draw_polygons();
        cv::imshow("Generalized Voronoi Diagram ", img_);
        cv::waitKey(0);

        delaunay_triangulation::VoronoiDiagram::draw(
            img_,
            delaunay_triangulation::VoronoiDiagram::create(
                delaunay_.getAllTriangles()
            )
        );
        cv::imshow("Generalized Voronoi Diagram ", img_);
        cv::waitKey(0);

        auto all_triangles = delaunay_.getTriangles();
        std::vector<Triangle> path_triangles;
        for (const auto &triangle : all_triangles)
        {
            size_t belonging_count = 0;
            for (const auto &polygon : polygons_)
            {
                for (const auto &vertex : triangle.vertices())
                {
                    if (polygon->has_vertex(vertex))
                    {
                        belonging_count++;
                        break;
                    }
                }
            }
            if (belonging_count > 1)
            {
                path_triangles.push_back(triangle);
            }
        }

        drawer_.reset(img_);
        draw_polygons();
        for (const auto &triangle : path_triangles)
        {
            triangle.draw(img_, false, cv::Scalar(0, 0, 0));
        }
        cv::imshow("Generalized Voronoi Diagram ", img_);
        cv::waitKey(0);

        for (size_t i = 0; i < path_triangles.size(); ++i)
        {
            for (size_t j = i + 1; j < path_triangles.size(); ++j)
            {
                if (path_triangles[i].hasSharedEdge(path_triangles[j]))
                {
                    const auto circumcenter_i = path_triangles[i].circum_circle.center;
                    const auto circumcenter_j = path_triangles[j].circum_circle.center;
                    cv::line(
                        img_,
                        cv::Point(circumcenter_i.x, circumcenter_i.y),
                        cv::Point(circumcenter_j.x, circumcenter_j.y),
                        cv::Scalar(0, 0, 255),
                        2,
                        cv::LINE_AA);
                }
            }
        }
        cv::imshow("Generalized Voronoi Diagram ", img_);
        cv::waitKey(0);

    }

private:
    const cv::Scalar bg_color_{255, 255, 255};
    const int image_width_, image_height_;

    std::vector<VertexedPolygonBase::SharedPtr> polygons_;

    DelaunayTriangulation delaunay_;
    DelaunayTriangulationDrawer drawer_;
    VoronoiDiagram voronoi_;

    cv::Mat img_;

    void draw_polygons()
    {
        for (const auto &polygon : polygons_)
        {
            polygon->draw(img_, cv::Scalar(255, 0, 0));
        }
    }

};

} // namespace delaunay_triangulation

int main()
{
    using namespace delaunay_triangulation;

    const double room_width = 1500;
    const double room_height = 1000;
    const double corridor_width = 50;
    const double rack_width = 50;
    const double rack_height = 350;
    const double surroundings_margin = 50;


    std::vector<VertexedPolygonBase::SharedPtr> polygons;
    polygons.push_back(VertexedRectangle::create_ptr(
        "wall",
        Vertex{room_width / 2.0, room_height / 2.0},
        room_width - surroundings_margin * 2,
        room_height - surroundings_margin * 2,
        0.0,
        10.0));

    const size_t num_rack_columns = 8;

    for (int i = 1; i <= num_rack_columns; ++i)
    {
        polygons.push_back(VertexedRectangle::create_ptr(
            "line_" + std::to_string(i),
            Vertex{
                surroundings_margin + corridor_width * i + rack_width * (i - 1) + rack_width / 2.0,
                surroundings_margin + corridor_width + rack_height / 2.0},
            rack_width,
            rack_height,
            0.0,
            10.0));
    }

    for (int i = 1; i <= num_rack_columns; ++i)
    {
        polygons.push_back(VertexedRectangle::create_ptr(
            "line_" + std::to_string(i),
            Vertex{
                surroundings_margin + corridor_width * i + rack_width * (i - 1) + rack_width / 2.0,
                room_height - surroundings_margin - corridor_width - rack_height / 2.0},
            rack_width,
            rack_height,
            0.0,
            10.0));
    }

    polygons.push_back(VertexedCircle::create_ptr(
        "circle",
        Vertex{1100, 300},
        200.0,
        10.0));

    polygons.push_back(VertexedTriangle::create_ptr(
        "triangle",
        Vertex{1200, 700},
        400.0,
        -45.0,
        10.0));

    GeneralizedVoronoiDiagram voronoi(room_width, room_height, polygons);
    voronoi.run();


    return EXIT_SUCCESS;
}

