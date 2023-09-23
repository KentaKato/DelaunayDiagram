#include <iostream>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.h"

using namespace delaunay_triangulation;

int main()
{
    DelaunayTriangulation triangulation;
    triangulation.createDelaunayTriangules();


    triangulation.draw();

    return 0;
}