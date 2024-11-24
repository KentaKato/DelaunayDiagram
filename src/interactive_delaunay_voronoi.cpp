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
        callbackData.drawer = &drawer_;
        callbackData.img = &img_;
        callbackData.voronoi_edge_color = &voronoi_edge_color_;
        callbackData.voronoi_draw_enabled = voronoi_draw_enabled_;
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
        delaunay_triangulation::DelaunayTriangulationDrawer* drawer;
        cv::Mat* img;
        cv::Scalar* voronoi_edge_color;
        bool voronoi_draw_enabled;
    };

    const cv::Scalar white_{255,255, 255};
    const cv::Scalar orange_{30, 105, 210};
    const int image_width_, image_height_;;

    DelaunayTriangulation delaunay_;
    DelaunayTriangulationDrawer drawer_;

    bool voronoi_draw_enabled_ = false;
    cv::Mat img_;
    cv::Scalar voronoi_edge_color_;

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
        if (event == cv::EVENT_LBUTTONDOWN) {
            const auto &delaunay = data->delaunay;
            const auto &drawer = data->drawer;
            delaunay->addVertex(x, y);
            delaunay->createDelaunayTriangles();
            drawer->draw(*data->img);
            draw_voronoi_diagram(
                *data->img,
                *delaunay,
                *data->voronoi_edge_color,
                data->voronoi_draw_enabled);
        }
    }


};


}







int main()
{
    delaunay_triangulation::InteractiveDelaunayVoronoi interactive_delaunay_voronoi(1000, 1000);
    interactive_delaunay_voronoi.run();
    return EXIT_SUCCESS;
}