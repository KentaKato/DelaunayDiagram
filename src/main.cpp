#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.hpp"
#include "DelaunayTriangulation/delaunay_triangulation_drawer.hpp"
#include "DelaunayTriangulation/voronoi_diagram.hpp"


const cv::Scalar WHITE(255,255, 255);
const cv::Scalar ORANGE(30, 105, 210);

struct MouseCallbackData {
    delaunay_triangulation::DelaunayTriangulation* delaunay;
    delaunay_triangulation::DelaunayTriangulationDrawer* drawer;
    cv::Mat* img;
};

bool voronoi_enabled = false;
cv::Scalar voronoi_color;

void draw_voronoi_diagram(
    cv::Mat &img,
    const delaunay_triangulation::DelaunayTriangulation &delaunay,
    const cv::Scalar &color)
{
    if (!voronoi_enabled)
    {
        return;
    }
    delaunay_triangulation::VoronoiDiagram::draw(
        img,
        delaunay_triangulation::VoronoiDiagram::create(
            delaunay.getAllTriangles()
        ),
        voronoi_color
    );
}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    MouseCallbackData* data = static_cast<MouseCallbackData*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        const auto &delaunay = data->delaunay;
        const auto &drawer = data->drawer;
        delaunay->addVertex(x, y);
        delaunay->createDelaunayTriangles();
        drawer->draw(*data->img);
        draw_voronoi_diagram(*data->img, *delaunay, voronoi_color);
    }
}

int main()
{
    // Initial image
    int image_width = 1000;
    int image_height = 1000;
    cv::Scalar white(255, 255, 255);
    cv::Mat img = cv::Mat(image_height, image_width, CV_8UC3, white);

    // Window
    cv::namedWindow("Delaunay Triangulation", cv::WINDOW_AUTOSIZE);

    delaunay_triangulation::DelaunayTriangulation delaunay;
    delaunay_triangulation::DelaunayTriangulationDrawer drawer(delaunay);

    MouseCallbackData callbackData;
    callbackData.delaunay = &delaunay;
    callbackData.drawer = &drawer;
    callbackData.img = &img;

    cv::setMouseCallback("Delaunay Triangulation", onMouse, &callbackData);

    constexpr int ESC_KEY = 27;
    constexpr int z_KEY = 'z'; // undo
    constexpr int c_KEY = 'c'; // draw circum circles
    constexpr int s_KEY = 's'; // draw super triangles
    constexpr int t_KEY = 't'; // draw vertex coordinate
    constexpr int f_KEY = 'f'; // fill triangle
    constexpr int v_KEY = 'v'; // voronoi diagram

    auto switch_voronoi_color = [&drawer]() -> void
    {
        if (drawer.isFillTriangle())
        {
            voronoi_color = WHITE;
        }
        else
        {
            voronoi_color = ORANGE;
        }
    };

    bool is_running = true;
    while (is_running) {
        cv::imshow("Delaunay Triangulation", img);
        int key = cv::waitKey(1);
        switch (key)
        {
        case ESC_KEY:
            is_running = false;
            break;

        case z_KEY:
            delaunay.removeLastVertex();
            delaunay.createDelaunayTriangles();
            drawer.draw(img);
            draw_voronoi_diagram(img, delaunay, voronoi_color);
            break;

        case c_KEY:
            drawer.switchDrawCircumCircles();
            drawer.draw(img);
            draw_voronoi_diagram(img, delaunay, voronoi_color);
            break;

        case s_KEY:
            drawer.switchDrawSuperTriangles();
            delaunay.createDelaunayTriangles();
            drawer.draw(img);
            draw_voronoi_diagram(img, delaunay, voronoi_color);
            break;

        case t_KEY:
            drawer.switchDrawVertexCoordinate();
            drawer.draw(img);
            draw_voronoi_diagram(img, delaunay, voronoi_color);
            break;

        case f_KEY:
            drawer.switchFillTriangle();
            drawer.draw(img);
            switch_voronoi_color();
            draw_voronoi_diagram(img, delaunay, voronoi_color);
            break;

        case v_KEY:
            voronoi_enabled = !voronoi_enabled;
            drawer.draw(img);
            switch_voronoi_color();
            draw_voronoi_diagram(img, delaunay, voronoi_color);
            break;

        default:
            break;
        }
    }

    return EXIT_SUCCESS;
}