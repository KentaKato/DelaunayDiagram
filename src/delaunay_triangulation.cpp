#include <iostream>
#include <array>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.h"

namespace delaunay_triangulation
{

DelaunayTriangulation::DelaunayTriangulation()
{
    this->addPoint(111, 110);
    this->addPoint(143, 110);
    this->addPoint(115, 123);
    this->addPoint(140, 120);
    this->addPoint(120, 133);
}

void DelaunayTriangulation::addPoint(double x, double y)
{
    points_.push_back(std::make_shared<Point>(x, y));
}

void DelaunayTriangulation::addPoint(const PointPtr &p)
{
    points_.push_back(p);
}

void DelaunayTriangulation::createDelaunayTriangules()
{
    this->addBoundingTriangle();

    this->draw();
    for (const auto &p : points_)
    {
        std::cout << "Add new point: " << p << std::endl;

        auto find_triangle_containing_point = [this](const PointPtr &p) -> Triangle
        {
            for (const auto &t : triangles_)
            {
                if (t.includePoint(p))
                    return t;
            }
            throw std::runtime_error("No triangle found that includes the point");
        };

        const auto t_contains_p = find_triangle_containing_point(p);
        triangles_.erase(std::remove(triangles_.begin(), triangles_.end(), t_contains_p),
                         triangles_.end());

        std::cout << "Adding new triangles" << std::endl;
        const Triangle t1 = Triangle(t_contains_p.p1, t_contains_p.p2, p);
        triangles_.push_back(t1);
        const Triangle t2 = Triangle(t_contains_p.p2, t_contains_p.p3, p);
        triangles_.push_back(t2);
        const Triangle t3 = Triangle(t_contains_p.p3, t_contains_p.p1, p);
        triangles_.push_back(t3);
        this->draw();

        std::cout << "Adding edges to stack" << std::endl;
        // add the edges of the triangle to the edge list
        edge_stack_.emplace(t_contains_p.p1, t_contains_p.p2);
        edge_stack_.emplace(t_contains_p.p2, t_contains_p.p3);
        edge_stack_.emplace(t_contains_p.p3, t_contains_p.p1);

        while (!edge_stack_.empty())
        {
            const Edge e = edge_stack_.top();
            edge_stack_.pop();

            // find the triangles that include this edge
            std::vector<Triangle> edge_triangles;
            for (const auto &t : triangles_)
            {
                if (t.includeEdge(e.p1, e.p2))
                {
                    std::cout << "Found triangle that includes edge" << std::endl;
                    edge_triangles.push_back(t);
                    if (edge_triangles.size() == 2)
                    {
                        std::cout << "Found two triangles that include edge" << std::endl;
                        break;
                    }
                }
            }

            if (edge_triangles.size() == 2)
            {
                auto &et1 = edge_triangles.at(0);
                auto &et2 = edge_triangles.at(1);
                if (et1 == et2)
                    throw std::runtime_error("Duplicate triangle found");

                const PointPtr unshared_p = this->findUnsharedVertex(et1, et2);
                if (et1.includePoint(unshared_p))
                {
                    this->flip(et1, et2);
                }
            }


        }
    }

}


void DelaunayTriangulation::draw()
{
    std::cout << "Num triangles: "<< triangles_.size() << std::endl;
    img_ = cv::Mat(image_height_, image_width_, CV_8UC3, background_color_);

    this->drawPoints();
    this->drawTriangles();

    cv::namedWindow("drawing", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
    cv::imshow("drawing", img_);
    cv::waitKey(0);
}

void DelaunayTriangulation::drawPoints()
{
    for (const auto &p : points_)
    {
        p->draw(img_);
    }
}

void DelaunayTriangulation::drawTriangles()
{
    for (const auto &t : triangles_)
    {
        t.draw(img_, true);
    }
}

void DelaunayTriangulation::addBoundingTriangle()
{
    // const PointPtr p1 = std::make_shared<Point>(-1e5, -1e5);
    // const PointPtr p2 = std::make_shared<Point>(1e5, -1e5);
    // const PointPtr p3 = std::make_shared<Point>(0, 1e5);
    const PointPtr p1 = std::make_shared<Point>(0, 0);
    const PointPtr p2 = std::make_shared<Point>(image_width_, 0);
    const PointPtr p3 = std::make_shared<Point>(0, image_height_);
    const Triangle t = Triangle(p1, p2, p3);
    points_.push_back(p1);
    points_.push_back(p2);
    points_.push_back(p3);
    triangles_.push_back(t);
}

PointPtr DelaunayTriangulation::findUnsharedVertex(const Triangle &ref, const Triangle &target) const
{
    if (target.p1 != ref.p1 && target.p1 != ref.p2 && target.p1 != ref.p3)
        return target.p1;
    if (target.p2 != ref.p1 && target.p2 != ref.p2 && target.p2 != ref.p3)
        return target.p2;
    if (target.p3 != ref.p1 && target.p3 != ref.p2 && target.p3 != ref.p3)
        return target.p3;
    throw std::runtime_error("No unshared vertex found");
};

Edge DelaunayTriangulation::findSharedEdge(const Triangle &t1,
                                           const Triangle &t2) const
{
    if (t1 == t2)
        throw std::runtime_error("Duplicate triangles are given");

    std::array<PointPtr, 2> shared_points;
    size_t index = 0;
    for (const auto &p1 : {t1.p1, t1.p2, t1.p3})
    {
        for (const auto &p2 : {t2.p1, t2.p2, t2.p3})
        {
            if (p1 == p2)
            {
                shared_points.at(index) = p1;
                if (++index == 2)
                    return Edge(shared_points.at(0), shared_points.at(1));
            }
        }
    }
    throw std::runtime_error("No shared edge found");
}

void DelaunayTriangulation::flip(const Triangle &t1, const Triangle &t2)
{
    const auto shared_edge = this->findSharedEdge(t1, t2);
    const auto unshared_p1 = this->findUnsharedVertex(t1, t2);
    const auto unshared_p2 = this->findUnsharedVertex(t2, t1);

    // create the new triangles
    std::cout << "Creating flipped triangles" << std::endl;
    triangles_.push_back(Triangle(unshared_p1, shared_edge.p1, unshared_p2));
    triangles_.push_back(Triangle(unshared_p1, shared_edge.p2, unshared_p2));

    // remove the triangles from the lists
    triangles_.erase(std::remove(triangles_.begin(), triangles_.end(), t1), triangles_.end());
    triangles_.erase(std::remove(triangles_.begin(), triangles_.end(), t2), triangles_.end());

    edge_stack_.emplace(unshared_p1, shared_edge.p1);
    edge_stack_.emplace(unshared_p1, shared_edge.p2);
    edge_stack_.emplace(unshared_p2, shared_edge.p1);
    edge_stack_.emplace(unshared_p2, shared_edge.p2);
}


} // namespace delaunay_Triangulation