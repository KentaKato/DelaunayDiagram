#include <iostream>
#include <array>
#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.h"

namespace delaunay_triangulation
{

std::ostream& operator<<(std::ostream &os, const Point &p)
{
    os << "(x, y) = (" << p.x << ", " << p.y << ")";
    return os;
}

std::ostream& operator<<(std::ostream &os, const PointPtr &p)
{
    os << *p;
    return os;
}

DelaunayTriangulation::DelaunayTriangulation()
    : img_(cv::Mat(image_height_, image_width_, CV_8UC3, background_color_))
{
    this->addPoint(110, 100);
    this->addPoint(430, 105);
    this->addPoint(150, 230);
    this->addPoint(400, 205);
    this->addPoint(200, 330);
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

        const auto t = find_triangle_containing_point(p);



        triangles_.erase(std::remove(triangles_.begin(), triangles_.end(), t),
                            triangles_.end());

        std::cout << "Adding new triangles" << std::endl;
        std::cout << "p1: " << t.p1 << std::endl;
        std::cout << "p2: " << t.p2 << std::endl;
        std::cout << "p3: " << t.p3 << std::endl;
        std::cout << "p: " << p << std::endl;

        triangles_.emplace_back(t.p1, t.p2, p);
        std::cout << "p1: " << t.p1 << std::endl;
        std::cout << "p2: " << t.p2 << std::endl;
        std::cout << "p3: " << t.p3 << std::endl;
        std::cout << "p: " << p << std::endl;
        triangles_.emplace_back(t.p2, t.p3, p);
        std::cout << "p1: " << t.p1 << std::endl;
        std::cout << "p2: " << t.p2 << std::endl;
        std::cout << "p3: " << t.p3 << std::endl;
        std::cout << "p: " << p << std::endl;
        triangles_.emplace_back(t.p3, t.p1, p);


        std::cout << "Adding edges to stack" << std::endl;
        // add the edges of the triangle to the edge list
        edge_stack_.emplace(t.p1, t.p2);
        edge_stack_.emplace(t.p2, t.p3);
        edge_stack_.emplace(t.p3, t.p1);

        while (!edge_stack_.empty())
        {
            const Edge e = edge_stack_.top();
            edge_stack_.pop();

            // find the triangles that include this edge
            std::vector<TrianglePtr> edge_triangles;
            for (const auto &t : triangles_)
            {
                if (t.includeEdge(e.p1, e.p2))
                {
                    std::cout << "Found triangle that includes edge" << std::endl;
                    edge_triangles.emplace_back(std::make_shared<Triangle>(t));
                    if (edge_triangles.size() == 2)
                        break;
                }
            }

            if (edge_triangles.size() == 2)
            {
                auto &t1 = edge_triangles.at(0);
                auto &t2 = edge_triangles.at(1);
                if (t1 == t2)
                    throw std::runtime_error("Duplicate triangle found");

                const PointPtr unshared_p = this->findUnsharedVertex(t1, t2);
                if (t1->includePoint(unshared_p))
                {
                    this->flip(t1, t2);
                }
            }


        }
    }

}


void DelaunayTriangulation::draw()
{
    std::cout << triangles_.size() << std::endl;
    // this->drawEdges();
    // this->drawCircumcircles();
    this->drawPoints();
    this->drawTriangles();

    cv::namedWindow("drawing", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
    cv::imshow("drawing", img_);
    cv::waitKey(0);
}


void DelaunayTriangulation::drawPoint(const PointPtr &p)
{
    cv::circle(img_, cv::Point(p->x, p->y), point_radius_, point_color_, -1, cv::LINE_AA);
}

void DelaunayTriangulation::drawPoints()
{
    for (const auto &p : points_)
    {
        drawPoint(p);
    }
}

// void DelaunayTriangulation::drawEdge(const Edge &e)
// {
//     cv::line(img_, cv::Point(e.p1->x, e.p1->y), cv::Point(e.p2->x, e.p2->y),
//              edge_color_, edge_thickness_, cv::LINE_AA);

// }

// void DelaunayTriangulation::drawEdges()
// {
//     for (const auto &e : edge_stack_)
//     {
//         drawEdge(e);
//     }
// }

void DelaunayTriangulation::drawTriangle(const Triangle &t)
{
    cv::line(img_, cv::Point(t.p1->x, t.p1->y), cv::Point(t.p2->x, t.p2->y),
             edge_color_, edge_thickness_, cv::LINE_AA);
    cv::line(img_, cv::Point(t.p2->x, t.p2->y), cv::Point(t.p3->x, t.p3->y),
             edge_color_, edge_thickness_, cv::LINE_AA);
    cv::line(img_, cv::Point(t.p3->x, t.p3->y), cv::Point(t.p1->x, t.p1->y),
             edge_color_, edge_thickness_, cv::LINE_AA);
}

void DelaunayTriangulation::drawTriangles()
{
    for (const auto &t : triangles_)
    {
        drawTriangle(t);
    }
}

void DelaunayTriangulation::drawCircumcircle(const Triangle &t)
{
    cv::circle(img_, cv::Point(t.circumcircle.center.x, t.circumcircle.center.y),
               t.circumcircle.radius, circumcircle_color_, circumcircle_thickness_, cv::LINE_AA);
}

void DelaunayTriangulation::drawCircumcircles()
{
    for (const auto &t : triangles_)
    {
        drawCircumcircle(t);
    }
}

void DelaunayTriangulation::addBoundingTriangle()
{
    const PointPtr p1 = std::make_shared<Point>(-1e5 + 1, -1e5);
    const PointPtr p2 = std::make_shared<Point>(1e5, -1e5 + 1);
    const PointPtr p3 = std::make_shared<Point>(10, 1e5 + 10);
    points_.push_back(p1);
    points_.push_back(p2);
    points_.push_back(p3);
    triangles_.emplace_back(p1, p2, p3);
}

PointPtr DelaunayTriangulation::findUnsharedVertex(const TrianglePtr &ref, const TrianglePtr &target) const
{
    if (target->p1 != ref->p1 && target->p1 != ref->p2 && target->p1 != ref->p3)
        return target->p1;
    if (target->p2 != ref->p1 && target->p2 != ref->p2 && target->p2 != ref->p3)
        return target->p2;
    if (target->p3 != ref->p1 && target->p3 != ref->p2 && target->p3 != ref->p3)
        return target->p3;
    throw std::runtime_error("No unshared vertex found");
};

Edge DelaunayTriangulation::findSharedEdge(const TrianglePtr &t1,
                                                              const TrianglePtr &t2) const
{
    if (t1 == t2)
        throw std::runtime_error("Duplicate triangles are given");

    std::array<PointPtr, 2> shared_points;
    size_t index = 0;
    for (const auto &p1 : {t1->p1, t1->p2, t1->p3})
    {
        for (const auto &p2 : {t2->p1, t2->p2, t2->p3})
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

void DelaunayTriangulation::flip(const TrianglePtr &t1, const TrianglePtr &t2)
{
    const auto shared_edge = this->findSharedEdge(t1, t2);

    // find the unshared vertices
    const auto unshared_p1 = this->findUnsharedVertex(t1, t2);
    const auto unshared_p2 = this->findUnsharedVertex(t2, t1);

    // create the new triangles
    std::cout << "Creating flipped triangles" << std::endl;
    triangles_.push_back(Triangle(unshared_p1, shared_edge.p1, unshared_p2));
    triangles_.push_back(Triangle(unshared_p1, shared_edge.p2, unshared_p2));

    // remove the triangles from the lists
    triangles_.erase(std::remove(triangles_.begin(), triangles_.end(), *t1), triangles_.end());
    triangles_.erase(std::remove(triangles_.begin(), triangles_.end(), *t2), triangles_.end());


    edge_stack_.emplace(unshared_p1, shared_edge.p1);
    edge_stack_.emplace(unshared_p1, shared_edge.p2);
    edge_stack_.emplace(unshared_p2, shared_edge.p1);
    edge_stack_.emplace(unshared_p2, shared_edge.p2);
}


Triangle::Triangle(const PointPtr &p1, const PointPtr &p2, const PointPtr &p3)
    : p1(p1), p2(p2), p3(p3)
{
    this->validate();
    this->computeCircumcircle();
}

void Triangle::validate() const
{
    std::cout << "Validating triangle: " << *this << std::endl;

    if (p1 == nullptr || p2 == nullptr || p3 == nullptr)
        throw std::runtime_error("Triangle is not valid: nullptr");

    if (p1 == p2 || p2 == p3 || p3 == p1)
    {
        throw std::runtime_error("Triangle is not valid: duplicate points");
    }

    const std::array<double, 2> vec12 = {p2->x - p1->x, p2->y - p1->y};
    const std::array<double, 2> vec13 = {p3->x - p1->x, p3->y - p1->y};
    auto cross = [](const std::array<double, 2> &a, const std::array<double, 2> &b) -> double
    {
        return a[0] * b[1] - a[1] * b[0];
    };
    if (cross(vec12, vec13) == 0)
        throw std::runtime_error("Triangle is not valid: collinear points");
}

void Triangle::computeCircumcircle()
{
    // Lambda function to compute the midpoint between two points
    auto midpoint = [](const PointPtr &a, const PointPtr &b) -> Point
    {
        return Point{(a->x + b->x) / 2.0, (a->y + b->y) / 2.0};
    };

    // Lambda function to compute the slope of the line between two points
    auto slope = [](const PointPtr &a, const PointPtr &b) -> double
    {
        if (b->x == a->x)
        {
            // vertical line case
            return INFINITY;
        }
        return (b->y - a->y) / (b->x - a->x);
    };
    // midpoints for two edges of the triangle
    const Point m12 = midpoint(p1, p2);
    const Point m13 = midpoint(p1, p3);

    // slopes for two edges of the triangle
    const double slope12 = slope(p1, p2);
    const double slope13 = slope(p1, p3);

    // the slopes of the perpendicular bisectors
    const double perpSlope12 = (slope12 == 0) ? INFINITY : (-1.0 / slope12);
    const double perpSlope13 = (slope13 == 0) ? INFINITY : (-1.0 / slope13);

    // the intersection of the two perpendicular bisectors
    const double x = (m13.y - m12.y - perpSlope13 * m13.x + perpSlope12 * m12.x) / (perpSlope12 - perpSlope13);
    const double y = m12.y + perpSlope12 * (x - m12.x);

    circumcircle.center = Point{x, y};
    circumcircle.radius = std::sqrt(std::pow(p1->x - x, 2) + std::pow(p1->y - y, 2));
}

bool Triangle::includePoint(const PointPtr &p) const
{
    return std::pow(p->x - circumcircle.center.x, 2) + std::pow(p->y - circumcircle.center.y, 2) <
           std::pow(circumcircle.radius, 2);
}

bool Triangle::includeEdge(const PointPtr &p1, const PointPtr &p2) const
{
    return this->includePoint(p1) && this->includePoint(p2);
}

bool Triangle::operator==(const Triangle& other) const
{
    return (p1 == other.p1 && p2 == other.p2 && p3 == other.p3) ||
            (p1 == other.p1 && p2 == other.p3 && p3 == other.p2) ||
            (p1 == other.p2 && p2 == other.p1 && p3 == other.p3) ||
            (p1 == other.p2 && p2 == other.p3 && p3 == other.p1) ||
            (p1 == other.p3 && p2 == other.p1 && p3 == other.p2) ||
            (p1 == other.p3 && p2 == other.p2 && p3 == other.p1);
}

std::ostream& operator<<(std::ostream &os, const Triangle &t)
{
    os << "\n- p1: " << t.p1 << "\n- p2: " << t.p2 << "\n- p3: " << t.p3;
    return os;
}

bool operator==(const TrianglePtr& lhs, const TrianglePtr& rhs)
{
    if(!lhs || !rhs)
        return lhs == rhs;
    return *lhs == *rhs;
}

bool operator==(const PointPtr& lhs, const PointPtr& rhs)
{
    if(!lhs || !rhs)
        return lhs == rhs;
    return *lhs == *rhs;
}

} // namespace delaunay_Triangulation