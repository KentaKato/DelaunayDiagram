#include <iostream>
#include <opencv2/opencv.hpp>

#include "DelaunayDiagram/delaunay_diagram.h"

using namespace delaunay_diagram;

int main()
{
    DelaunayDiagram diagram;
    diagram.draw();

    return 0;
}