#include <iostream>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/geometry_primitives.h"

using namespace delaunay_triangulation;

int main()
{
    const cv::Scalar background_color = cv::Scalar(255,255,255);
    const int image_width = 500;
    const int image_height = 500;
    cv::Mat img(image_height, image_width, CV_8UC3, background_color);

    PointPtr p1 = std::make_shared<Point>(Point(100, 100));
    PointPtr p2 = std::make_shared<Point>(Point(100, 200));
    PointPtr p3 = std::make_shared<Point>(Point(300, 100));
    PointPtr p4 = std::make_shared<Point>(Point(430, 250));

    TrianglePtr t1 = std::make_shared<Triangle>(Triangle(p1, p2, p3));
    TrianglePtr t2 = std::make_shared<Triangle>(Triangle(p2, p3, p4));

    p1->draw(img);
    p2->draw(img);
    p3->draw(img);
    p4->draw(img);
    t1->draw(img, true);
    t2->draw(img, true);

    cv::namedWindow("drawing", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
    cv::imshow("drawing", img);
    cv::waitKey(0);
    return 0;
}