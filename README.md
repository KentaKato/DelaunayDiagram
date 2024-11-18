# DelaunayTriangulation

A C++ implementation of the Delaunay Triangulation algorithm using the Bowyer-Watson method.

## Demo

<p align="center">
  <img src=".readme/delaunay.gif" width="400" alt="Delaunay Triangulation Demo"/>
</p>

## Key Functions

- **`f`**: Fill triangles with color.
- **`z`**: Remove the last added vertex.
- **`c`**: Draw circumcircles around triangles.
- **`t`**: Display the coordinates of vertices.
- **`s`**: Show super triangles used in the algorithm.

## Installation

### Prerequisites

Ensure you have the following installed:

- **CMake**
- **YAML-CPP library** (`libyaml-cpp-dev`)
- **OpenCV library** (`libopencv-dev`)
- **C++ Compiler** (supporting C++23 or later)

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
   git clone https://github.com/KentaKato/DelaunayTriangulation.git
   ```

4. **Build the Project**

   ```bash
   cd DelaunayTriangulation
   mkdir build
   cd build
   cmake ..
   make
   ```

## Usage

After building the project, run the executable:

```bash
./delaunay_triangulation
```

- **Add Vertex**: Click on the canvas to add a new vertex.
- **Keyboard Shortcuts**:
  - **`f`**: Toggle filling of triangles.
  - **`z`**: Undo the last added vertex.
  - **`c`**: Toggle drawing of circumcircles.
  - **`t`**: Toggle display of vertex coordinates.
  - **`s`**: Toggle display of super triangles.

## License

This project is licensed under the MIT License.
