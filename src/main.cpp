#include <iostream>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.hpp"

using namespace delaunay_triangulation;

int main()
{
    DelaunayTriangulation triangulation;
    triangulation.createDelaunayTriangles();


    triangulation.draw();

    return 0;
}