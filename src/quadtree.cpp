#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <algorithm>
#include "triangulation.h"

constexpr float TRIANGULATION_MARKER = -9999;

inline constexpr std::size_t pow2(const int& i)
{
    return std::size_t(1) << i;
}

struct Heightmap {
    int maxDepth;
    std::size_t vertDim;
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
    std::size_t width;
    int depth;
    const Heightmap* heightmap;;
    Level(const int& depth, const Heightmap* heightmap);
    std::vector<std::size_t> CenterVerts() const;
    std::vector<std::size_t> BoundaryVerts() const;
    std::vector<float> Triangulation(const int& offset) const;
};

Level::Level(const int& depth, const Heightmap* heightmap) {
    this->heightmap = heightmap;
    this->depth = depth;
    width = (heightmap->vertDim - 1) / pow2(depth);
}

std::vector<std::size_t> Level::CenterVerts() const {
    std::vector<std::size_t> vertices;
    vertices.reserve(pow2(2 * (depth - 1)));
    for (std::size_t row = width; row < heightmap->vertDim; row += (2 * width)) {
        for (std::size_t col = width; col < heightmap->vertDim; col += (2 * width)) {
            vertices.push_back(row * heightmap->vertDim + col);
        }
    }
    return vertices;
}

std::vector<std::size_t> Level::BoundaryVerts() const {
    std::vector<std::size_t> vertices;
    vertices.reserve(2 * pow2(2 * (depth - 1)) + pow2(depth));
    for (std::size_t row = 0; row < heightmap->vertDim; row += width) {
        std::size_t offset = width - (row % (2 * width));
        for (std::size_t col = offset; col < heightmap->vertDim; col += (2 * width)) {
            vertices.push_back(row * heightmap->vertDim + col);
        }
    }
    return vertices;
}

std::vector<float> Level::Triangulation(const int& offset) const {
    std::size_t vertDim = heightmap->vertDim;
    std::vector<float> triangulation(vertDim * vertDim, TRIANGULATION_MARKER);
    std::vector<bool> marked(vertDim * vertDim, true);
    std::vector<std::array<std::size_t, 3>> triangles = CreateMesh(marked, 2 * depth + offset, vertDim);
    for (std::array<std::size_t, 3>&triangle : triangles) {
        std::array<double, 3> xVerts = { (double)(triangle[0] % vertDim), (double)(triangle[1] % vertDim), (double)(triangle[2] % vertDim) };
        std::array<double, 3> yVerts = { (double)(triangle[0] / vertDim), (double)(triangle[1] / vertDim), (double)(triangle[2] / vertDim) };
        std::array<double, 3> zVerts = { (double)(heightmap->vertexHeights[triangle[0]]), (double)(heightmap->vertexHeights[triangle[1]]), (double)(heightmap->vertexHeights[triangle[2]]) };
        std::size_t minX = (size_t)*std::min_element(std::begin(xVerts), std::end(xVerts));
        std::size_t maxX = (size_t)*std::max_element(std::begin(xVerts), std::end(xVerts));
        std::size_t minY = (size_t)*std::min_element(std::begin(yVerts), std::end(yVerts));
        std::size_t maxY = (size_t)*std::max_element(std::begin(yVerts), std::end(yVerts));
        std::pair<double, double> vectorAB = { xVerts[1] - xVerts[0], yVerts[1] - yVerts[0] };
        std::pair<double, double> vectorAC = { xVerts[2] - xVerts[0], yVerts[2] - yVerts[0] };
        std::pair<double, double> vectorNAC = { yVerts[0] - yVerts[2], xVerts[2] - xVerts[0] };
        std::pair<double, double> vectorNAB = { yVerts[0] - yVerts[1], xVerts[1] - xVerts[0] };
        for (std::size_t row = minY; row <= maxY; row++) {
            for (std::size_t col = minX; col <= maxX; col++) {
                size_t index = row * vertDim + col;
                if (triangulation[index] == TRIANGULATION_MARKER) {
                    std::pair<double, double> vectorAP = { (double)col - xVerts[0], (double)row - yVerts[0]};
                    double beta = Dot(vectorAP, vectorNAC) / Dot(vectorAB, vectorNAC);
                    double gamma = Dot(vectorAP, vectorNAB) / Dot(vectorAC, vectorNAB);
                    double alpha = 1 - beta - gamma;
                    if (alpha >= 0 && beta >= 0 && gamma >= 0) {
                        triangulation[index] = alpha * zVerts[0] + beta * zVerts[1] + gamma * zVerts[2];
                    }
                }
            }
        }
    }
    return triangulation;
}

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

Vertex::Vertex(std::size_t index, bool isCenter, const Level* level) {
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

std::size_t Vertex::ParentQuad(const std::size_t& coord) const {
    return coord % (4 * level->width) / (2 * level->width);
}

std::size_t Vertex::ToIndex(const std::size_t& row, const std::size_t& col) const {
    return row * level->heightmap->vertDim + col;
}

std::size_t Vertex::FixCoord(const std::size_t& coord) const {
    return std::max<std::size_t>(0, std::min<std::size_t>(level->heightmap->vertDim - 1, coord));
}

std::array<std::size_t, 2> Vertex::Neighbours() const {
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
    std::vector<std::size_t> subverts;
    if (isCenter) {
        subverts.reserve(4 * level->width * level->width + 4 * level->width - 3);
        for (std::size_t rowOffset = row - level->width; rowOffset <= row + level->width; rowOffset++) {
            for (std::size_t colOffset = col - level->width; colOffset <= col + level->width; colOffset++) {
                if ((std::abs<std::size_t>(row - rowOffset) + std::abs<std::size_t>(col - colOffset)) < pow2((int)level->width)) {
                    subverts.push_back(ToIndex(FixCoord(rowOffset), FixCoord(colOffset)));
                }
            }
        }
    }
    else {
        subverts.reserve(2 * level->width * level->width + 2 * level->width - 3);
        for (std::size_t rowOffset = row - level->width; rowOffset <= row + level->width; rowOffset++) {
            for (std::size_t colOffset = col - level->width + std::abs<std::size_t>(row - rowOffset);
                colOffset <= col + level->width - std::abs<std::size_t>(row - rowOffset); colOffset++) {
                if ((std::abs<std::size_t>(rowOffset - row) < level->width) && (std::abs<std::size_t>(colOffset - col) < level->width)) {
                    subverts.push_back(ToIndex(FixCoord(rowOffset), FixCoord(colOffset)));
                }
            }
        }
    }
    float maxError = 0;
    for (std::size_t& vertex : subverts) {
        float error = std::abs(level->heightmap->vertexHeights[vertex] - triangulation[vertex]);
        if (error > maxError) {
            maxError = error;
        }
    }
    return maxError;
}