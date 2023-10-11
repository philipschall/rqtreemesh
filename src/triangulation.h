#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <pybind11/stl.h>

std::array<float, 3> TriangleNorm(const std::array<long, 3>& triangle, const float& z0, const float& z1, const float& z2, const long& vertDim);

void RecursiveTriangulate(std::vector<bool>& marked, std::vector<std::array<long, 3>>& triangles, const std::array<long, 3>& triangle, const int& level, const int& maxLevel);

void RecursiveCollect(std::vector<long>& vertices, const std::array<long, 3>& triangle, const int& level, const int& maxLevel);

std::vector<std::array<long, 3>> CreateMesh(std::vector<bool>& marked, const int& maxLevel, const long& vertDim);

#endif /* TRIANGULATION_H */
