#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <algorithm>
#include <cstdint>

constexpr float TRIANGULATION_MARKER = -9999;

inline std::int32_t Pow2(const std::int32_t& i)
{
    return std::int32_t(1) << i;
}

inline double Dot(const std::array<double, 2>& vecA, const std::array<double, 2>& vecB) {
    return vecA[0] * vecB[0] + vecA[1] * vecB[1];
}

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

Heightmap::Heightmap(const pybind11::array_t<float>& array, const std::int32_t& maxDepth, const std::int32_t& vertDimX, const std::int32_t& vertDimY, const std::int32_t& minVertDim) {
    this->maxDepth = maxDepth;
    this->vertDimX = vertDimX;
    this->vertDimY = vertDimY;
    this->minVertDim = minVertDim;
    this->maxTris = 2 * (std::size_t)(vertDimX - 1) * (vertDimY - 1);
    vertexHeights = std::vector<float>(vertDimX * vertDimY);
    auto r = array.unchecked<2>();
    pybind11::ssize_t maxDimX = static_cast<pybind11::ssize_t>(vertDimX - 2);
    pybind11::ssize_t maxDimY = static_cast<pybind11::ssize_t>(vertDimY - 2);
    for (pybind11::ssize_t row = 0; row < vertDimY; row++) {
        pybind11::ssize_t prevRow = std::max<pybind11::ssize_t>(row - 1, 0);
        pybind11::ssize_t nextRow = std::min<pybind11::ssize_t>(row + 1, maxDimY);
        float topLeft = r(prevRow, 0);
        float bottomLeft = r(nextRow, 0);
        for (pybind11::ssize_t col = 0; col < vertDimX; col++) {
            pybind11::ssize_t prevCol = std::max<pybind11::ssize_t>(col - 1, 0);
            pybind11::ssize_t nextCol = std::min<pybind11::ssize_t>(col + 1, maxDimX);
            float topRight = r(prevRow, nextCol);
            float bottomRight = r(nextRow, nextCol);
            vertexHeights[row * vertDimX + col] = 0.25f * (topLeft + topRight + bottomLeft + bottomRight);
            topLeft = topRight;
            bottomLeft = bottomRight;
        }
    }
    std::vector<std::array<std::int32_t, 3>> initTris;
    std::vector<std::int32_t> initVerts;
    for (std::int32_t topLeftRow = 0; topLeftRow < vertDimY - 1; topLeftRow += minVertDim - 1) {
        for (std::int32_t topLeftCol = 0; topLeftCol < vertDimX - 1; topLeftCol += minVertDim - 1) {
            std::int32_t topLeftIndex = topLeftRow * vertDimX + topLeftCol;
            std::int32_t bottomLeftIndex = (topLeftRow + minVertDim - 1) * vertDimX + topLeftCol;
            std::int32_t bottomRightIndex = (topLeftRow + minVertDim - 1) * vertDimX + topLeftCol + minVertDim - 1;
            std::int32_t topRightIndex = topLeftRow * vertDimX + topLeftCol + minVertDim - 1;
            initTris.push_back({ bottomLeftIndex, bottomRightIndex, topLeftIndex });
            initTris.push_back({ topRightIndex, topLeftIndex, bottomRightIndex });
            initVerts.insert(initVerts.end(), { topLeftIndex, topRightIndex, bottomLeftIndex, bottomRightIndex });
        }
    }
    this->initTris = initTris;
    this->initVerts = initVerts;
}

void Heightmap::RecursiveTriangulate(std::vector<bool>& marked, std::vector<std::array<std::int32_t, 3>>& triangles,
    const std::array<std::int32_t, 3>& triangle, const int& level, const int& maxLevel) {
    if (level == maxLevel) {
        triangles.push_back(triangle);
    }
    else if (!marked[(triangle[1] + triangle[2]) / (std::int32_t)2]) {
        triangles.push_back(triangle);
    }
    else {
        RecursiveTriangulate(marked, triangles, { (triangle[1] + triangle[2]) / 2,
                             triangle[0], triangle[1] }, level + 1, maxLevel);
        RecursiveTriangulate(marked, triangles, { (triangle[1] + triangle[2]) / 2,
                             triangle[2], triangle[0] }, level + 1, maxLevel);
    }
}

std::vector<std::array<std::int32_t, 3>> Heightmap::CreateMesh(std::vector<bool>& marked, const int& maxLevel, const std::vector<std::array<std::int32_t, 3>>& initTris, const std::size_t& maxTris) {
    std::vector<std::array<std::int32_t, 3>> triangles;
    triangles.reserve(maxTris);
    for (const std::array<std::int32_t, 3>&triangle : initTris) {
        RecursiveTriangulate(marked, triangles, triangle, 0, maxLevel);
    }
    triangles.shrink_to_fit();
    return triangles;
}

struct Level {
    std::int32_t width;
    std::int32_t depth;
    const Heightmap* heightmap;;
    Level(const std::int32_t& depth, const Heightmap* heightmap);
    std::vector<std::int32_t> CenterVerts() const;
    std::vector<std::int32_t> BoundaryVerts() const;
    std::vector<float> Triangulation(const std::int32_t& offset) const;
};

Level::Level(const std::int32_t& depth, const Heightmap* heightmap) {
    this->heightmap = heightmap;
    this->depth = depth;
    width = (heightmap->minVertDim - 1) / Pow2(depth);
}

std::vector<std::int32_t> Level::CenterVerts() const {
    std::vector<std::int32_t> vertices;
    vertices.reserve(Pow2(2 * (depth - 1)));
    for (std::int32_t row = width; row < heightmap->vertDimY; row += (2 * width)) {
        for (std::int32_t col = width; col < heightmap->vertDimX; col += (2 * width)) {
            vertices.push_back(row * heightmap->vertDimX + col);
        }
    }
    return vertices;
}

std::vector<std::int32_t> Level::BoundaryVerts() const {
    std::vector<std::int32_t> vertices;
    vertices.reserve(2 * Pow2(2 * (depth - 1)) + Pow2(depth));
    for (std::int32_t row = 0; row < heightmap->vertDimY; row += width) {
        std::int32_t offset = width - (row % (2 * width));
        for (std::int32_t col = offset; col < heightmap->vertDimX; col += (2 * width)) {
            vertices.push_back(row * heightmap->vertDimX + col);
        }
    }
    return vertices;
}

std::vector<float> Level::Triangulation(const std::int32_t& offset) const {
    std::int32_t vertDimX = heightmap->vertDimX;
    std::int32_t vertDimY = heightmap->vertDimY;
    std::vector<float> triangulation(vertDimX * vertDimY, TRIANGULATION_MARKER);
    std::vector<bool> marked(vertDimX * vertDimY, true);
    std::vector<std::array<std::int32_t, 3>> triangles = Heightmap::CreateMesh(marked, 2 * depth + offset, heightmap->initTris, heightmap->maxTris);
    for (std::array<std::int32_t, 3>& triangle : triangles) {
        std::array<double, 3> xVerts = { (double)(triangle[0] % vertDimX), (double)(triangle[1] % vertDimX), (double)(triangle[2] % vertDimX) };
        std::array<double, 3> yVerts = { (double)(triangle[0] / vertDimX), (double)(triangle[1] / vertDimX), (double)(triangle[2] / vertDimX) };
        std::array<double, 3> zVerts = { (double)(heightmap->vertexHeights[triangle[0]]), (double)(heightmap->vertexHeights[triangle[1]]), (double)(heightmap->vertexHeights[triangle[2]]) };
        std::int32_t minX = (std::int32_t)*std::min_element(std::begin(xVerts), std::end(xVerts));
        std::int32_t maxX = (std::int32_t)*std::max_element(std::begin(xVerts), std::end(xVerts));
        std::int32_t minY = (std::int32_t)*std::min_element(std::begin(yVerts), std::end(yVerts));
        std::int32_t maxY = (std::int32_t)*std::max_element(std::begin(yVerts), std::end(yVerts));
        std::array<double, 2> vectorAB = { xVerts[1] - xVerts[0], yVerts[1] - yVerts[0] };
        std::array<double, 2> vectorAC = { xVerts[2] - xVerts[0], yVerts[2] - yVerts[0] };
        std::array<double, 2> vectorNAC = { yVerts[0] - yVerts[2], xVerts[2] - xVerts[0] };
        std::array<double, 2> vectorNAB = { yVerts[0] - yVerts[1], xVerts[1] - xVerts[0] };
        for (std::int32_t row = minY; row <= maxY; row++) {
            for (std::int32_t col = minX; col <= maxX; col++) {
                std::int32_t index = row * vertDimX + col;
                if (triangulation[index] == TRIANGULATION_MARKER) {
                    std::array<double, 2> vectorAP = { (double)col - xVerts[0], (double)row - yVerts[0] };
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

Vertex::Vertex(std::int32_t index, bool isCenter, const Level* level) {
    this->index = index;
    this->isCenter = isCenter;
    row = index / level->heightmap->vertDimX;
    col = index % level->heightmap->vertDimX;
    this->level = level;
}

bool Vertex::IsHorizontal() const {
    if (!isCenter && (row % (2 * level->width) == 0)) {
        return true;
    }
    return false;
}

std::int32_t Vertex::ParentQuad(const std::int32_t& coord) const {
    return coord % (4 * level->width) / (2 * level->width);
}

std::int32_t Vertex::ToIndex(const std::int32_t& row, const std::int32_t& col) const {
    return row * level->heightmap->vertDimX + col;
}

std::int32_t Vertex::FixCoord(const std::int32_t& coord, const std::int32_t& dim) const {
    return std::max<std::int32_t>(0, std::min<std::int32_t>(coord, dim));
}

std::array<std::int32_t, 2> Vertex::Neighbours() const {
    std::int32_t width = level->width;
    if (isCenter) {
        if (ParentQuad(row) == ParentQuad(col)) {
            return { ToIndex(FixCoord(row + width, level->heightmap->vertDimY - 1), FixCoord(col - width, level->heightmap->vertDimX - 1)),
                     ToIndex(FixCoord(row - width, level->heightmap->vertDimY - 1), FixCoord(col + width, level->heightmap->vertDimX - 1)) };
        }
        else {
            return { ToIndex(FixCoord(row - width, level->heightmap->vertDimY - 1), FixCoord(col - width, level->heightmap->vertDimX - 1)),
                     ToIndex(FixCoord(row + width, level->heightmap->vertDimY - 1), FixCoord(col + width, level->heightmap->vertDimX - 1)) };
        }
    }
    else if (IsHorizontal()) {
        return { ToIndex(FixCoord(row + width, level->heightmap->vertDimY - 1), col), ToIndex(FixCoord(row - width, level->heightmap->vertDimY - 1), col) };
    }
    else {
        return { ToIndex(row, FixCoord(col - width, level->heightmap->vertDimX - 1)), ToIndex(row, FixCoord(col + width, level->heightmap->vertDimX - 1)) };
    }
}

float Vertex::Error(const std::vector<float>& triangulation) const {
    std::vector<std::int32_t> subverts;
    if (isCenter) {
        subverts.reserve(4 * level->width * level->width + 4 * level->width - 3);
        for (std::int32_t rowOffset = row - level->width; rowOffset <= row + level->width; rowOffset++) {
            for (std::int32_t colOffset = col - level->width; colOffset <= col + level->width; colOffset++) {
                if ((std::abs<std::int32_t>(row - rowOffset) + std::abs<std::int32_t>(col - colOffset)) < Pow2(level->width)) {
                    subverts.push_back(ToIndex(FixCoord(rowOffset, level->heightmap->vertDimY - 1), FixCoord(colOffset, level->heightmap->vertDimX - 1)));
                }
            }
        }
    }
    else {
        subverts.reserve(2 * level->width * level->width + 2 * level->width - 3);
        for (std::int32_t rowOffset = row - level->width; rowOffset <= row + level->width; rowOffset++) {
            for (std::int32_t colOffset = col - level->width + std::abs<std::int32_t>(row - rowOffset);
                colOffset <= col + level->width - std::abs<std::int32_t>(row - rowOffset); colOffset++) {
                if ((std::abs<std::int32_t>(rowOffset - row) < level->width) && (std::abs<std::int32_t>(colOffset - col) < level->width)) {
                    subverts.push_back(ToIndex(FixCoord(rowOffset, level->heightmap->vertDimY - 1), FixCoord(colOffset, level->heightmap->vertDimX - 1)));
                }
            }
        }
    }
    float maxError = 0;
    for (std::int32_t& vertex : subverts) {
        float error = std::abs(level->heightmap->vertexHeights[vertex] - triangulation[vertex]);
        if (error > maxError) {
            maxError = error;
        }
    }
    return maxError;
}