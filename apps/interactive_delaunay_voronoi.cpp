#include <opencv2/opencv.hpp>

#include "DelaunayTriangulation/delaunay_triangulation.hpp"
#include "DelaunayTriangulation/delaunay_triangulation_drawer.hpp"
#include "DelaunayTriangulation/voronoi_diagram.hpp"


namespace delaunay_triangulation
{

class InteractiveDelaunayVoronoi
{
public:
    explicit InteractiveDelaunayVoronoi(int image_width, int image_height)
        : image_width_(image_width), image_height_(image_height), delaunay_(), drawer_(delaunay_)
    {
        img_ = cv::Mat(image_height, image_width, CV_8UC3, white_);
    }

    void run()
    {
        cv::namedWindow("Delaunay Triangulation", cv::WINDOW_AUTOSIZE);

        MouseCallbackData callbackData;
        callbackData.delaunay = &delaunay_;
        callbackData.img = &img_;
        callbackData.finding_nearest_vertex_trace = &finding_nearest_vertex_trace_;
        cv::setMouseCallback("Delaunay Triangulation", onMouse, &callbackData);

        auto switch_voronoi_color = [this]() -> void
        {
            if (drawer_.isFillTriangle())
            {
                voronoi_edge_color_ = white_;
            }
            else
            {
                voronoi_edge_color_ = orange_;
            }
        };

        bool is_running = true;
        while (is_running) {
            cv::imshow("Delaunay Triangulation", img_);
            int key = cv::waitKey(1);

            switch (key)
            {
            case ESC_KEY:
                is_running = false;
                continue;

            case z_KEY:
                delaunay_.removeLastVertex();
                delaunay_.createDelaunayTriangles();
                break;

            case c_KEY:
                drawer_.switchDrawCircumCircles();
                break;

            case s_KEY:
                drawer_.switchDrawSuperTriangles();
                delaunay_.createDelaunayTriangles();
                break;

            case t_KEY:
                drawer_.switchDrawVertexCoordinate();
                break;

            case f_KEY:
                drawer_.switchFillTriangle();
                switch_voronoi_color();
                break;

            case v_KEY:
                voronoi_draw_enabled_ = !voronoi_draw_enabled_;
                switch_voronoi_color();
                break;

            default:
                break;
            }
            drawer_.draw(img_);
            draw_voronoi_diagram(img_, delaunay_, voronoi_edge_color_, voronoi_draw_enabled_);
            draw_trace();
        }
    }

private:
    static constexpr int ESC_KEY = 27; // exit
    static constexpr int z_KEY = 'z'; // undo
    static constexpr int c_KEY = 'c'; // draw circum circles
    static constexpr int s_KEY = 's'; // draw super triangles
    static constexpr int t_KEY = 't'; // draw vertex coordinate
    static constexpr int f_KEY = 'f'; // fill triangle
    static constexpr int v_KEY = 'v'; // voronoi diagram

    struct MouseCallbackData {
        delaunay_triangulation::DelaunayTriangulation* delaunay;
        cv::Mat* img;
        std::vector<Vertex>* finding_nearest_vertex_trace;
    };

    const cv::Scalar white_{255,255, 255};
    const cv::Scalar orange_{30, 105, 210};
    const int image_width_, image_height_;;

    DelaunayTriangulation delaunay_;
    DelaunayTriangulationDrawer drawer_;

    bool voronoi_draw_enabled_ = false;
    bool delaunay_draw_enabled_ = true;
    cv::Mat img_;
    cv::Scalar voronoi_edge_color_;

    std::vector<Vertex> finding_nearest_vertex_trace_;

    void draw_trace()
    {
        if (finding_nearest_vertex_trace_.empty())
        {
            return;
        }
        std::vector<cv::Point> points;
        for (const auto &p : finding_nearest_vertex_trace_)
        {
            points.push_back(cv::Point(p.x, p.y));
        }
        const cv::Scalar red{255, 0, 0};
        cv::polylines(img_, points, false, red, 3, cv::LINE_AA);
        cv::circle(img_, cv::Point(points.back().x, points.back().y), 10, red, -1, cv::LINE_AA);
    }

    static void draw_voronoi_diagram(
        cv::Mat &img,
        const delaunay_triangulation::DelaunayTriangulation &delaunay,
        const cv::Scalar &voronoi_edge_color, const bool voronoi_draw_enabled)
    {
        if (!voronoi_draw_enabled)
        {
            return;
        }
        delaunay_triangulation::VoronoiDiagram::draw(
            img,
            delaunay_triangulation::VoronoiDiagram::create(
                delaunay.getAllTriangles()
            ),
            voronoi_edge_color
        );
    }

    static void onMouse(int event, int x, int y, int flags, void* userdata)
    {
        MouseCallbackData* data = static_cast<MouseCallbackData*>(userdata);
        const auto &delaunay = data->delaunay;
        auto &finding_nearest_vertex_trace = *data->finding_nearest_vertex_trace;
        if (event == cv::EVENT_LBUTTONDOWN) {
            delaunay->addVertex(x, y);
            delaunay->createDelaunayTriangles();
            finding_nearest_vertex_trace.clear();
        }
        else if (event == cv::EVENT_MBUTTONDOWN) {
            // Find the nearest vertex to the clicked point
            const Point p(x, y);
            delaunay->findNearestVertex(p, finding_nearest_vertex_trace, std::nullopt);
        }
    }
};

} // namespace delaunay_triangulation

int main()
{
    delaunay_triangulation::InteractiveDelaunayVoronoi interactive_delaunay_voronoi(1000, 1000);
    interactive_delaunay_voronoi.run();
    return EXIT_SUCCESS;
}