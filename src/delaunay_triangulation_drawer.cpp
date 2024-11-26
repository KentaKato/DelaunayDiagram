#include "DelaunayTriangulation/delaunay_triangulation_drawer.hpp"
#include <opencv2/opencv.hpp>


namespace delaunay_triangulation
{

DelaunayTriangulationDrawer::DelaunayTriangulationDrawer(const DelaunayTriangulation &delaunay)
    : delaunay_(delaunay)
{
}

void DelaunayTriangulationDrawer::switchDrawCircumCircles(){
    draw_circum_circles_ = !draw_circum_circles_;
}

void DelaunayTriangulationDrawer::switchDrawSuperTriangles(){
    draw_super_triangles_ = !draw_super_triangles_;
}

void DelaunayTriangulationDrawer::switchDrawVertexCoordinate(){
    draw_vertex_coordinate_ = !draw_vertex_coordinate_;
}

void DelaunayTriangulationDrawer::switchFillTriangle(){
    fill_triangle_ = !fill_triangle_;
}

void DelaunayTriangulationDrawer::reset(cv::Mat &img)
{
    img.setTo(white_);
}

void DelaunayTriangulationDrawer::draw(cv::Mat &img)
{
    this->reset(img);
    this->drawTriangles(img);
    this->drawVertices(img);
}

void DelaunayTriangulationDrawer::drawVertices(cv::Mat &img)
{
    for (const auto &v : delaunay_.getVertices())
    {
        v.draw(img, draw_vertex_coordinate_);
    }
}

void DelaunayTriangulationDrawer::drawTriangles(cv::Mat &img)
{
    static const cv::Scalar super_triangle_edge_color{0, 255, 0};

    const auto triangles = delaunay_.getTriangles();
    std::vector<Triangle> super_triangles{};
    if (draw_super_triangles_)
    {
        super_triangles = delaunay_.getSuperTriangles();
    }

    for (const auto &t: triangles)
    {
        t.draw(img, fill_triangle_);
        if (draw_circum_circles_) t.draw_circum_circle(img);
    }
    for (const auto &t: super_triangles)
    {
        t.draw(img, fill_triangle_, super_triangle_edge_color);
        if (draw_circum_circles_) t.draw_circum_circle(img);
    }


}

} // namespace delaunay_Triangulation