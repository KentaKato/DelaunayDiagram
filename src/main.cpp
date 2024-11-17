#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.hpp"


cv::Scalar WHITE_BG(255, 255, 255);

struct MouseCallbackData {
    delaunay_triangulation::DelaunayTriangulation* delaunay;
    cv::Mat* img;
};

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    MouseCallbackData* data = static_cast<MouseCallbackData*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        data->img->setTo(WHITE_BG);
        data->delaunay->addVertex(x, y);
        data->delaunay->createDelaunayTriangles(*data->img);
        data->delaunay->draw(*data->img);
    }
}

int main()
{
    // Initial image
    int image_width = 1000;
    int image_height = 1000;
    cv::Mat img = cv::Mat(image_height, image_width, CV_8UC3, WHITE_BG);

    // Window
    cv::namedWindow("Delaunay Triangulation", cv::WINDOW_AUTOSIZE);

    delaunay_triangulation::DelaunayTriangulation delaunay;

    MouseCallbackData callbackData;
    callbackData.delaunay = &delaunay;
    callbackData.img = &img;

    cv::setMouseCallback("Delaunay Triangulation", onMouse, &callbackData);

    constexpr int ESC_KEY = 27;
    constexpr int z_KEY = 'z';
    constexpr int c_KEY = 'c';
    constexpr int s_KEY = 's';
    constexpr int t_KEY = 't';
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
            delaunay.createDelaunayTriangles(img);
            img.setTo(WHITE_BG);
            delaunay.draw(img);
        }
        else if (key == c_KEY)
        {
            delaunay.switchDrawCircumCircles();
            img.setTo(WHITE_BG);
            delaunay.draw(img);
        }
        else if (key == s_KEY)
        {
            delaunay.switchDrawSuperTriangles();
            delaunay.createDelaunayTriangles(img);
            img.setTo(WHITE_BG);
            delaunay.draw(img);
        }
        else if (key == t_KEY)
        {
            delaunay.switchDrawVertexCoordinate();
            img.setTo(WHITE_BG);
            delaunay.draw(img);
        }
    }

    return EXIT_SUCCESS;
}