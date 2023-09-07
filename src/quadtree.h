#ifndef QUADTREE_H
#define QUADTREE_H

#include<pybind11/stl.h>
#include<pybind11/numpy.h>

struct Heightmap {
    int maxDepth;
    long vertDim;
    pybind11::array_t<float> array;
    std::vector<float> vertexHeights;
    Heightmap(const pybind11::array_t<float>& array, const int& maxDepth);
};

struct Level {
    long width;
    int depth;
    const Heightmap* heightmap;;
    Level(const int& depth, const Heightmap* heightmap);
    std::vector<long> CenterVerts() const;
    std::vector<long> BoundaryVerts() const;
    std::vector<float> Triangulation(const int& offset) const;
};

struct Vertex {
    long index;
    long row;
    long col;
    const Level* level;
    bool isCenter;
    Vertex(long index, bool isCenter, const Level* level);
    bool IsHorizontal() const;
    long ParentQuad(const long& coord) const;
    long ToIndex(const long& row, const long& col) const;
    long FixCoord(const long& coord) const;
    std::array<long, 2> Neighbours() const;
    float Error(const std::vector<float>& triangulation) const;
};

#endif /* QUADTREE_H */
