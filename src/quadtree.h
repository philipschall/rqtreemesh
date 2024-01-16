#ifndef QUADTREE_H
#define QUADTREE_H

#include<pybind11/stl.h>
#include<pybind11/numpy.h>

struct Heightmap {
    std::int32_t maxDepth;
    std::int32_t vertDimX;
    std::int32_t vertDimY;
    std::int32_t minVertDim;
    std::size_t maxTris;
    std::vector<float> vertexHeights;
    std::vector<std::array<std::int32_t, 3>> initTris;
    std::vector<std::int32_t> initVerts;
    Heightmap(const pybind11::array_t<float>& array, const std::int32_t& maxDepth, const std::int32_t& vertDimX, const std::int32_t& vertDimY, const std::int32_t& minVertDim);
    static void RecursiveTriangulate(std::vector<bool>& marked, std::vector<std::array<std::int32_t, 3>>& triangles,
        const std::array<std::int32_t, 3>& triangle, const int& level, const int& maxLevel);
    static std::vector<std::array<std::int32_t, 3>> CreateMesh(std::vector<bool>& marked, const int& maxLevel, const std::vector<std::array<std::int32_t, 3>>& initTris, const std::size_t& maxTris);

};

struct Level {
    std::int32_t width;
    std::int32_t depth;
    const Heightmap* heightmap;;
    Level(const std::int32_t& depth, const Heightmap* heightmap);
    std::vector<std::int32_t> CenterVerts() const;
    std::vector<std::int32_t> BoundaryVerts() const;
    std::vector<float> Triangulation(const std::int32_t& offset) const;
};

struct Vertex {
    std::int32_t index;
    std::int32_t row;
    std::int32_t col;
    const Level* level;
    bool isCenter;
    Vertex(std::int32_t index, bool isCenter, const Level* level);
    bool IsHorizontal() const;
    std::int32_t ParentQuad(const std::int32_t& coord) const;
    std::int32_t ToIndex(const std::int32_t& row, const std::int32_t& col) const;
    std::int32_t FixCoord(const std::int32_t& coord, const std::int32_t& dim) const;
    std::array<std::int32_t, 2> Neighbours() const;
    float Error(const std::vector<float>& triangulation) const;
};

#endif /* QUADTREE_H */
