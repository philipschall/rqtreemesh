#ifndef QUADTREE_H
#define QUADTREE_H

#include<pybind11/stl.h>
#include<pybind11/numpy.h>

struct Heightmap {
    int maxDepth;
    std::size_t vertDim;
    pybind11::array_t<float> array;
    std::vector<float> vertexHeights;
    Heightmap(const pybind11::array_t<float>& array, const int& maxDepth);
};

struct Level {
    std::size_t width;
    int depth;
    const Heightmap* heightmap;;
    Level(const int& depth, const Heightmap* heightmap);
    std::vector<std::size_t> CenterVerts() const;
    std::vector<std::size_t> BoundaryVerts() const;
    std::vector<float> Triangulation(const int& offset) const;
};

struct Vertex {
    std::size_t index;
    std::size_t row;
    std::size_t col;
    const Level* level;
    bool isCenter;
    Vertex(std::size_t index, bool isCenter, const Level* level);
    bool IsHorizontal() const;
    std::size_t ParentQuad(const std::size_t& coord) const;
    std::size_t ToIndex(const std::size_t& row, const std::size_t& col) const;
    std::size_t FixCoord(const std::size_t& coord) const;
    std::array<std::size_t, 2> Neighbours() const;
    float Error(const std::vector<float>& triangulation) const;
};

#endif /* QUADTREE_H */
