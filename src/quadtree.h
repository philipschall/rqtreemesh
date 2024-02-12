#ifndef QUADTREE_H
#define QUADTREE_H

#include<pybind11/stl.h>
#include<pybind11/numpy.h>

struct Heightmap {
    std::int64_t maxDepth;
    std::int64_t vertDimX;
    std::int64_t vertDimY;
    std::int64_t minVertDim;
    std::size_t maxTris;
    std::vector<float> vertexHeights;
    std::vector<std::array<std::int64_t, 3>> initTris;
    std::vector<std::int64_t> initVerts;
    Heightmap(const pybind11::array_t<float>& array, const std::int64_t& maxDepth, const std::int64_t& vertDimX, const std::int64_t& vertDimY, const std::int64_t& minVertDim);
    static void RecursiveTriangulate(std::vector<bool>& marked, std::vector<std::array<std::int64_t, 3>>& triangles,
        const std::array<std::int64_t, 3>& triangle, const int& level, const int& maxLevel);
    static std::vector<std::array<std::int64_t, 3>> CreateMesh(std::vector<bool>& marked, const int& maxLevel, const std::vector<std::array<std::int64_t, 3>>& initTris, const std::size_t& maxTris);

};

struct Level {
    std::int64_t width;
    std::int64_t depth;
    const Heightmap* heightmap;;
    Level(const std::int64_t& depth, const Heightmap* heightmap);
    std::vector<std::int64_t> CenterVerts() const;
    std::vector<std::int64_t> BoundaryVerts() const;
    std::vector<float> Triangulation(const std::int64_t& offset) const;
};

struct Vertex {
    std::int64_t index;
    std::int64_t row;
    std::int64_t col;
    const Level* level;
    bool isCenter;
    Vertex(std::int64_t index, bool isCenter, const Level* level);
    bool IsHorizontal() const;
    std::int64_t ParentQuad(const std::int64_t& coord) const;
    std::int64_t ToIndex(const std::int64_t& row, const std::int64_t& col) const;
    std::int64_t FixCoord(const std::int64_t& coord, const std::int64_t& dim) const;
    std::array<std::int64_t, 2> Neighbours() const;
    float Error(const std::vector<float>& triangulation) const;
};

#endif /* QUADTREE_H */
