#ifndef DELAUNAY_TRIANGULATION__GEOMETRY_PRIMITIVES_HPP
#define DELAUNAY_TRIANGULATION__GEOMETRY_PRIMITIVES_HPP

#include <opencv2/opencv.hpp>
#include <iostream>

namespace delaunay_triangulation
{

class Vertex
{
public:
    Vertex() : x(0), y(0) {}
    explicit Vertex(const double x, const double y) : x(x), y(y) {}

    void draw(cv::Mat &img , const bool draw_coordinate_value=false) const;

    Vertex& operator=(Vertex &other);
    Vertex& operator=(const Vertex &other);

    double x;
    double y;

private:
    const int radius_ = 2;
    const cv::Scalar color_ = cv::Scalar(0, 0, 0);
};

bool operator==(const Vertex &lhs, const Vertex &rhs);
std::ostream& operator<<(std::ostream &os, const Vertex &p);

Vertex operator+(const Vertex& lhs, const Vertex& rhs);

Vertex operator/(const Vertex& v, double scalar);

class Edge
{
public:
    Edge(const Vertex &_v1, const Vertex &_v2) : v1(_v1), v2(_v2) {}
    void draw(cv::Mat &img) const;

    Vertex v1;
    Vertex v2;
};

class Circle
{
public:
    Circle(){};
    explicit Circle(const Vertex center, double radius)
        : center(center), radius(radius) {}
    void draw(cv::Mat &img, bool draw_center = false) const;

    Vertex center = Vertex{0, 0};
    double radius = 0.0;

private:
    cv::Scalar circle_color_ = cv::Scalar(0, 0, 200);
    cv::Scalar center_color_ = cv::Scalar(0, 0, 200);
};

class Triangle
{
public:
    explicit Triangle(const Vertex &v1,
                      const Vertex &v2,
                      const Vertex &v3);

    void computeCircumCircle();
    void draw(cv::Mat &img, const bool fill, const cv::Scalar &edge_color = cv::Scalar(0, 0, 0)) const;
    void draw_circum_circle(cv::Mat &img) const;
    [[nodiscard]] bool isInCircumCircle(const Vertex &v) const;
    [[nodiscard]] bool has(const Edge &e) const;
    [[nodiscard]] bool has(const Vertex &v) const;
    friend std::ostream& operator<<(std::ostream &os, const Triangle &t);

    Vertex v1, v2, v3;
    Circle circum_circle;

private:
    void validate() const;
};
bool operator==(const Triangle &lhs, const Triangle &rhs);

} // namespace delaunay_Triangulation

#endif