#include "DelaunayTriangulation/delaunay_triangulation.hpp"
#include "DelaunayTriangulation/delaunay_triangulation_drawer.hpp"
#include "DelaunayTriangulation/voronoi_diagram.hpp"

#include <random>

using namespace delaunay_triangulation;

void addRandomVertices(DelaunayTriangulation &delaunay, int image_width, int image_height, size_t num_vertices)
{
    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    std::uniform_real_distribution<> width_dist(0, image_width);
    std::uniform_real_distribution<> height_dist(0, image_height);
    for (size_t i = 0; i < num_vertices; ++i)
    {
        Vertex v(width_dist(engine), height_dist(engine));
        if (!delaunay.hasVertex(v))
        {
            delaunay.addVertex(v);
        }
        else {
            --i;
        }
    }
}


int main()
{
    // Initial image
    int image_width = 1000;
    int image_height = 1000;
    cv::Scalar white(255, 255, 255);
    cv::Mat img = cv::Mat(image_height, image_width, CV_8UC3, white);

    DelaunayTriangulation delaunay;
    DelaunayTriangulationDrawer drawer(delaunay);
    drawer.setFillTriangle(false);

    size_t num_vertices = 500;
    addRandomVertices(delaunay, image_width, image_height, num_vertices);


    // initialize belonging cells
    std::map<Point, Site> belonging_cells;
    for (const int &x : {0, image_width})
    {
        for (const int &y : {0, image_height})
        {
            Point p{static_cast<double>(x), static_cast<double>(y)};
            Site s;
            belonging_cells[p] = s;
        }
    }

    // initialize weight map
    std::map<Point, double> weight_map;
    int step = 5;
    for (int x = 0; x < image_width; x += step)
    {
        for (int y = 0; y < image_height; y += step)
        {
            weight_map[Point{static_cast<double>(x), static_cast<double>(y)}] = 1.0;
        }
    }

    while (true)
    {
        delaunay.createDelaunayTriangles();
        img.setTo(white);
        auto voronoi_cells = VoronoiDiagram::create(delaunay.getAllTriangles());
        VoronoiDiagram::draw( img, voronoi_cells);

        std::vector<Site> sites;
        sites.reserve(voronoi_cells.size());
        for (const auto & [site, _] : voronoi_cells)
        {
            sites.push_back(site);
        }

        for (const auto & [point, _]: belonging_cells)
        {
            VoronoiDiagram::findBelongingCell(sites, point, belonging_cells[point]);
        }

        std::map<Site, Centroid> voronoi_centroids;
        VoronoiDiagram::computeVoronoiCentroids(
            sites,
            weight_map,
            voronoi_centroids);

        cv::Scalar centroid_color(0, 255, 0);
        for (const auto & [site, centroid] : voronoi_centroids)
        {
            centroid.draw(img, false, centroid_color);
            site.draw(img, false);
        }

        std::cout << "show" << std::endl;
        cv::imshow("Centroid Voronoi Diagram", img);

        cv::waitKey(1);

        delaunay.clear();
        delaunay.reserveVerticesVector(num_vertices);
        for (const auto & [_, centroid] : voronoi_centroids)
        {
            delaunay.addVertex(centroid);
        }
    }
}

