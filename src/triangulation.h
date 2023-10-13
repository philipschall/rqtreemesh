#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <pybind11/stl.h>

std::array<float, 3> TriangleNorm(const std::array<std::size_t, 3>& triangle, const float& z0, const float& z1, const float& z2, const std::size_t& vertDim);

void RecursiveTriangulate(std::vector<bool>& marked, std::vector<std::array<std::size_t, 3>>& triangles, const std::array<std::size_t, 3>& triangle, const int& level, const int& maxLevel);

void RecursiveCollect(std::vector<std::size_t>& vertices, const std::array<std::size_t, 3>& triangle, const int& level, const int& maxLevel);

std::vector<std::array<std::size_t, 3>> CreateMesh(std::vector<bool>& marked, const int& maxLevel, const std::size_t& vertDim);

#endif /* TRIANGULATION_H */
