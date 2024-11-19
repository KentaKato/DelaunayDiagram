#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.hpp"
#include "DelaunayTriangulation/delaunay_triangulation_drawer.hpp"



struct MouseCallbackData {
    delaunay_triangulation::DelaunayTriangulation* delaunay;
    delaunay_triangulation::DelaunayTriangulationDrawer* drawer;
    cv::Mat* img;
};

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    MouseCallbackData* data = static_cast<MouseCallbackData*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        const auto &delaunay = data->delaunay;
        const auto &drawer = data->drawer;
        delaunay->addVertex(x, y);
        delaunay->createDelaunayTriangles();
        drawer->draw(*data->img);
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
    while (true) {
        cv::imshow("Delaunay Triangulation", img);
        int key = cv::waitKey(1);
        if (key == ESC_KEY)
        {
            // If the user presses the ESC key, exit the loop
            break;
        }
        else if (key == z_KEY)
        {
            delaunay.removeLastVertex();
            delaunay.createDelaunayTriangles();
            drawer.draw(img);
        }
        else if (key == c_KEY)
        {
            drawer.switchDrawCircumCircles();
            drawer.draw(img);
        }
        else if (key == s_KEY)
        {
            drawer.switchDrawSuperTriangles();
            delaunay.createDelaunayTriangles();
            drawer.draw(img);
        }
        else if (key == t_KEY)
        {
            drawer.switchDrawVertexCoordinate();
            drawer.draw(img);
        }
        else if (key == f_KEY)
        {
            drawer.switchFillTriangle();
            drawer.draw(img);
        }
    }

    return EXIT_SUCCESS;
}