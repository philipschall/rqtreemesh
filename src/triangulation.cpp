#include <pybind11/stl.h>

std::array<float, 3> TriangleNorm(const std::array<std::size_t, 3>& triangle, const float& z0,
    const float& z1, const float& z2, const std::size_t& vertDim) {
    std::array<float, 3> vector0 = { (float)((long long)triangle[0] % (long long)vertDim - (long long)triangle[1] % (long long)vertDim),
                                     (float)((long long)triangle[0] / (long long)vertDim - (long long)triangle[1] / (long long)vertDim),
                                     z0 - z1 };
    std::array<float, 3> vector1 = { (float)((long long)triangle[2] % (long long)vertDim - (long long)triangle[1] % (long long)vertDim),
                                     (float)((long long)triangle[2] / (long long)vertDim - (long long)triangle[1] / (long long)vertDim),
                                     z2 - z1 };
    return { vector0[1] * vector1[2] - vector0[2] * vector1[1],
             vector0[2] * vector1[0] - vector0[0] * vector1[2],
             vector0[0] * vector1[1] - vector0[1] * vector1[0] };

}

void RecursiveTriangulate(std::vector<bool>& marked, std::vector<std::array<std::size_t, 3>>& triangles,
    const std::array<std::size_t, 3>& triangle, const int& level, const int& maxLevel) {
    if (level == maxLevel) {
        triangles.push_back(triangle);
    }
    else if (!marked[(triangle[1] + triangle[2]) / (std::size_t)2]) {
        triangles.push_back(triangle);
    }
    else {
        RecursiveTriangulate(marked, triangles, { (triangle[1] + triangle[2]) / 2,
                             triangle[0], triangle[1] }, level + 1, maxLevel);
        RecursiveTriangulate(marked, triangles, { (triangle[1] + triangle[2]) / 2,
                             triangle[2], triangle[0] }, level + 1, maxLevel);
    }
}

void RecursiveCollect(std::vector<std::size_t>& vertices, const std::array<std::size_t, 3>& triangle,
    const int& level, const int& maxLevel) {
    if (level == maxLevel) {
        return;
    }
    vertices.push_back((triangle[1] + triangle[2]) / 2);
    RecursiveCollect(vertices, { (triangle[1] + triangle[2]) / 2, triangle[0],
        triangle[1] }, level + 1, maxLevel);
    RecursiveCollect(vertices, { (triangle[1] + triangle[2]) / 2, triangle[2],
        triangle[0] }, level + 1, maxLevel);
}

std::vector<std::array<std::size_t, 3>> CreateMesh(std::vector<bool>& marked, const int& maxLevel, const std::size_t& vertDim) {
    std::vector<std::array<std::size_t, 3>> triangles;
    triangles.reserve((vertDim - 1) * (vertDim - 1) / 2);
    RecursiveTriangulate(marked, triangles, { vertDim * (vertDim - 1), vertDim * vertDim - 1, 0 }, 0, maxLevel);
    RecursiveTriangulate(marked, triangles, { vertDim - 1, 0, vertDim * vertDim - 1 }, 0, maxLevel);
    triangles.shrink_to_fit();
    return triangles;
}