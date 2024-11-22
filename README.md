# DelaunayVoronoiCpp

A C++ implementation of the Delaunay Triangulation algorithm and Voronoi Diagram construction. Delaunay Triangulation uses the Bowyer-Watson method.

## Demo

<p float="left">
  <img src=".readme/delaunay.gif" width="400" alt="Delaunay Triangulation Demo"/>
  <img src=".readme/voronoi.gif" width="400" alt="Voronoi Diagram Demo"/>
</p>

## Key Functions

- **`v`**: Draw Voronoi Diagram.
- **`c`**: Draw circumcircles around triangles.
- **`t`**: Display the coordinates of vertices.
- **`s`**: Show super triangles used in the algorithm.
- **`z`**: Remove the last added vertex.
- **`f`**: Fill triangles with color.
- **`Esc`**: Exit.

## Installation

### Prerequisites

Ensure you have the following installed:

- **CMake**
- **YAML-CPP library** (`libyaml-cpp-dev`)
- **OpenCV library** (`libopencv-dev`)
- **C++ Compiler** (supporting C++20 or later)

### Installation Steps

1. **Update Package Lists**

   ```bash
   sudo apt update
   ```

2. **Install Dependencies**

   ```bash
   sudo apt install cmake libyaml-cpp-dev libopencv-dev
   ```

3. **Clone the Repository**

   ```bash
   git clone https://github.com/KentaKato/DelaunayVoronoiCpp.git
   ```

4. **Build the Project**

   ```bash
   cd DelaunayVoronoiCpp
   mkdir build
   cd build
   cmake ..
   make
   ```

## Usage

After building the project, run the executable:

```bash
./delaunay_voronoi
```

- **Add Vertex**: Click on the canvas to add a new vertex.
- **Keyboard Shortcuts**:
  - **`v`**: Toggle drawing of Voronoi Diagram.
  - **`c`**: Toggle drawing of circumcircles.
  - **`t`**: Toggle display of vertex coordinates.
  - **`s`**: Toggle display of super triangles.
  - **`z`**: Undo the last added vertex.
  - **`f`**: Toggle filling of triangles.
  - **`Esc`**: Exit.

## License

This project is licensed under the MIT License.
