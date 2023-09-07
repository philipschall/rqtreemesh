#include <pybind11/stl.h>

std::array<float, 3> TriangleNorm(const std::array<long, 3>& triangle, const float& z0,
    const float& z1, const float& z2, const long& vertDim) {
    std::array<float, 3> vector0 = { (float)(triangle[0] % vertDim - triangle[1] % vertDim),
                                     (float)(triangle[0] / vertDim - triangle[1] / vertDim),
                                     z0 - z1 };
    std::array<float, 3> vector1 = { (float)(triangle[2] % vertDim - triangle[1] % vertDim),
                                     (float)(triangle[2] / vertDim - triangle[1] / vertDim),
                                     z2 - z1 };
    return { vector0[1] * vector1[2] - vector0[2] * vector1[1],
             vector0[2] * vector1[0] - vector0[0] * vector1[2],
             vector0[0] * vector1[1] - vector0[1] * vector1[0] };

}

void RecursiveTriangulate(std::vector<bool>& marked, std::vector<std::array<long, 3>>& triangles,
    const std::array<long, 3>& triangle, const int& level, const int& maxLevel) {
    if (level == maxLevel) {
        triangles.push_back(triangle);
    }
    else if (!marked[(size_t)(triangle[1] + triangle[2]) / 2]) {
        triangles.push_back(triangle);
    }
    else {
        RecursiveTriangulate(marked, triangles, { (triangle[1] + triangle[2]) / 2,
                             triangle[0], triangle[1] }, level + 1, maxLevel);
        RecursiveTriangulate(marked, triangles, { (triangle[1] + triangle[2]) / 2,
                             triangle[2], triangle[0] }, level + 1, maxLevel);
    }
}

void RecursiveCollect(std::set<long>& vertices, const std::array<long, 3>& triangle,
    const int& level, const int& maxLevel) {
    if (level == maxLevel) {
        return;
    }
    vertices.insert((triangle[1] + triangle[2]) / 2);
    RecursiveCollect(vertices, { (triangle[1] + triangle[2]) / 2, triangle[0],
        triangle[1] }, level + 1, maxLevel);
    RecursiveCollect(vertices, { (triangle[1] + triangle[2]) / 2, triangle[2],
        triangle[0] }, level + 1, maxLevel);
}

std::vector<std::array<long, 3>> CreateMesh(std::vector<bool>& marked, const int& maxLevel, const long& vertDim) {
    std::vector<std::array<long, 3>> triangles;
    triangles.reserve((size_t)(2 * vertDim * vertDim));
    RecursiveTriangulate(marked, triangles, { vertDim * (vertDim - 1), vertDim * vertDim - 1, 0 }, 0, maxLevel);
    RecursiveTriangulate(marked, triangles, { vertDim - 1, 0, vertDim * vertDim - 1 }, 0, maxLevel);
    triangles.shrink_to_fit();
    return triangles;
}