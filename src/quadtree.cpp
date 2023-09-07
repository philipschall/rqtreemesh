#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "triangulation.h"

inline constexpr long pow2(const int& i)
{
    return long(1) << i;
}

struct Heightmap {
    int maxDepth;
    long vertDim;
    pybind11::array_t<float> array;
    std::vector<float> vertexHeights;
    Heightmap(const pybind11::array_t<float>& array, const int& maxDepth);
};

Heightmap::Heightmap(const pybind11::array_t<float>& array, const int& maxDepth) {
    this->maxDepth = maxDepth;
    vertDim = pow2(maxDepth) + 1;
    vertexHeights = std::vector<float>(vertDim * vertDim);
    auto r = array.unchecked<2>();
    pybind11::ssize_t maxDim = static_cast<pybind11::ssize_t>(vertDim - 2);
    for (pybind11::ssize_t row = 0; row < vertDim; row++) {
        pybind11::ssize_t prevRow = std::max<pybind11::ssize_t>(row - 1, 0);
        pybind11::ssize_t nextRow = std::min<pybind11::ssize_t>(row + 1, maxDim);
        float topLeft = r(prevRow, 0);
        float bottomLeft = r(nextRow, 0);
        for (pybind11::ssize_t col = 0; col < vertDim; col++) {
            pybind11::ssize_t prevCol = std::max<pybind11::ssize_t>(col - 1, 0);
            pybind11::ssize_t nextCol = std::min<pybind11::ssize_t>(col + 1, maxDim);
            float topRight = r(prevRow, nextCol);
            float bottomRight = r(nextRow, nextCol);
            vertexHeights[row * vertDim + col] = 0.25f * (topLeft + topRight + bottomLeft + bottomRight);
            topLeft = topRight;
            bottomLeft = bottomRight;
        }
    }
}

struct Level {
    long width;
    int depth;
    const Heightmap* heightmap;;
    Level(const int& depth, const Heightmap* heightmap);
    std::vector<long> CenterVerts() const;
    std::vector<long> BoundaryVerts() const;
    std::vector<float> Triangulation(const int& offset) const;
};

Level::Level(const int& depth, const Heightmap* heightmap) {
    this->heightmap = heightmap;
    this->depth = depth;
    width = (heightmap->vertDim - 1) / pow2(depth);
}

std::vector<long> Level::CenterVerts() const {
    std::vector<long> vertices;
    vertices.reserve(pow2(2 * (depth - 1)));
    for (long row = width; row < heightmap->vertDim; row += (2 * width)) {
        for (long col = width; col < heightmap->vertDim; col += (2 * width)) {
            vertices.push_back(row * heightmap->vertDim + col);
        }
    }
    return vertices;
}

std::vector<long> Level::BoundaryVerts() const {
    std::vector<long> vertices;
    vertices.reserve(2 * pow2(2 * (depth - 1)) + pow2(depth));
    for (long row = 0; row < heightmap->vertDim; row += width) {
        long offset = width - (row % (2 * width));
        for (long col = offset; col < heightmap->vertDim; col += (2 * width)) {
            vertices.push_back(row * heightmap->vertDim + col);
        }
    }
    return vertices;
}

std::vector<float> Level::Triangulation(const int& offset) const {
    long vertDim = heightmap->vertDim;
    std::vector<float> triangulation(vertDim * vertDim, 0);
    std::vector<bool> marked(vertDim * vertDim, true);
    std::vector<std::array<long, 3>> triangles = CreateMesh(marked, 2 * depth + offset, vertDim);
    for (std::array<long, 3>&triangle : triangles) {
        std::array<float, 3> norm = TriangleNorm(triangle,
            heightmap->vertexHeights[triangle[0]],
            heightmap->vertexHeights[triangle[1]],
            heightmap->vertexHeights[triangle[2]],
            heightmap->vertDim);
        std::set<long> subVertices = { triangle[0], triangle[1], triangle[2] };
        RecursiveCollect(subVertices, triangle, 2 * depth + offset, 2 * (heightmap->maxDepth));
        for (const long& subVertex : subVertices) {
            if (triangulation[subVertex] == 0) {
                triangulation[subVertex] = heightmap->vertexHeights[triangle[0]] -
                    1 / norm[2] * (norm[0] * ((subVertex % heightmap->vertDim) - (triangle[0] % heightmap->vertDim)) +
                        norm[1] * ((subVertex / heightmap->vertDim) - (triangle[0] / heightmap->vertDim)));

            }
        }
    }
    return triangulation;
}

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

Vertex::Vertex(long index, bool isCenter, const Level* level) {
    this->index = index;
    this->isCenter = isCenter;
    row = index / level->heightmap->vertDim;
    col = index % level->heightmap->vertDim;
    this->level = level;
}

bool Vertex::IsHorizontal() const {
    if (!isCenter && (row % (2 * level->width) == 0)) {
        return true;
    }
    return false;
}

long Vertex::ParentQuad(const long& coord) const {
    return coord % (4 * level->width) / (2 * level->width);
}

long Vertex::ToIndex(const long& row, const long& col) const {
    return row * level->heightmap->vertDim + col;
}

long Vertex::FixCoord(const long& coord) const {
    return std::max<long>(0, std::min<long>(level->heightmap->vertDim - 1, coord));
}

std::array<long, 2> Vertex::Neighbours() const {
    if (isCenter) {
        if (ParentQuad(row) == ParentQuad(col)) {
            return { ToIndex(FixCoord(row + level->width), FixCoord(col - level->width)),
                     ToIndex(FixCoord(row - level->width), FixCoord(col + level->width)) };
        }
        else {
            return { ToIndex(FixCoord(row - level->width), FixCoord(col - level->width)),
                     ToIndex(FixCoord(row + level->width), FixCoord(col + level->width)) };
        }
    }
    else if (IsHorizontal()) {
        return { ToIndex(FixCoord(row + level->width), col), ToIndex(FixCoord(row - level->width), col) };
    }
    else {
        return { ToIndex(row, FixCoord(col - level->width)), ToIndex(row, FixCoord(col + level->width)) };
    }
}

float Vertex::Error(const std::vector<float>& triangulation) const {
    std::vector<long> subverts;
    if (isCenter) {
        subverts.reserve((size_t)(4 * level->width * level->width + 4 * level->width - 3));
        for (long rowOffset = row - level->width; rowOffset <= row + level->width; rowOffset++) {
            for (long colOffset = col - level->width; colOffset <= col + level->width; colOffset++) {
                if ((std::abs(row - rowOffset) + std::abs(col - colOffset)) < pow2(level->width)) {
                    subverts.push_back(ToIndex(FixCoord(rowOffset), FixCoord(colOffset)));
                }
            }
        }
    }
    else {
        subverts.reserve((size_t)(2 * level->width * level->width + 2 * level->width - 3));
        for (long rowOffset = row - level->width; rowOffset <= row + level->width; rowOffset++) {
            for (long colOffset = col - level->width + std::abs(row - rowOffset);
                colOffset <= col + level->width - std::abs(row - rowOffset); colOffset++) {
                if ((std::abs(rowOffset - row) < level->width) && (std::abs(colOffset - col) < level->width)) {
                    subverts.push_back(ToIndex(FixCoord(rowOffset), FixCoord(colOffset)));
                }
            }
        }
    }
    float maxError = 0;
    for (long& vertex : subverts) {
        float error = std::abs(level->heightmap->vertexHeights[vertex] - triangulation[vertex]);
        if (error > maxError) {
            maxError = error;
        }
    }
    return maxError;
}