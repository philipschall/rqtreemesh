#include <pybind11/stl.h>
#include<pybind11/numpy.h>
#include "quadtree.h"
#include "triangulation.h"

void MarkVertices(const Level& level, const std::vector<long>& vertices, std::vector<bool>& markedVertices,
    std::vector<bool>& marked, const bool& isCenter, const int& triangulationOffset, const float& maxError) {
    std::vector<float> triangulation;
    bool isTriangulated = false;
    for (const long& index : vertices) {
        Vertex vertex = Vertex(index, isCenter, &level);
        if (!marked[index]) {
            if (!isTriangulated) {
                triangulation = level.Triangulation(triangulationOffset);
                isTriangulated = true;
            }
            if (!(vertex.Error(triangulation) > maxError)) {
                continue;
            }
        }
        markedVertices[index] = true;
        for (long& neighbourIndex : vertex.Neighbours()) {
            marked[neighbourIndex] = true;
        }
        marked[index] = false;
    }
}

std::pair<pybind11::array, pybind11::array> RestrictedQuadTreeTriangulation(pybind11::array_t<float> array,
    int maxDepth, float maxError, double pixelDim, double topLeftX, double topLeftY) {
    Heightmap heightmap = Heightmap(array, maxDepth);
    std::vector<bool> markedVertices(heightmap.vertDim * heightmap.vertDim, false);
    std::vector<bool> marked(heightmap.vertDim * heightmap.vertDim, false);
    for (int depth = heightmap.maxDepth; depth > 0; depth--) {
        Level level = Level(depth, &heightmap);
        MarkVertices(level, level.BoundaryVerts(), markedVertices, marked, false, -1, maxError);
        MarkVertices(level, level.CenterVerts(), markedVertices, marked, true, -2, maxError);
    }
    markedVertices[0] = true;
    markedVertices[heightmap.vertDim - 1] = true;
    markedVertices[heightmap.vertDim * (heightmap.vertDim - 1)] = true;
    markedVertices.back() = true;
    std::vector<std::array<long, 3>> triangles = CreateMesh(markedVertices, 2 * maxDepth, heightmap.vertDim);
    std::vector<std::array<double, 3>> vertexLocations;
    vertexLocations.reserve(markedVertices.size());
    std::vector<std::array<long, 3>> newTriangles(triangles.size());
    std::vector<long> lookup(markedVertices.size(), 0);
    long counter = 0;
    for (size_t i = 0; i < markedVertices.size(); i++) {
        if (markedVertices[i]) {
            size_t row = i / heightmap.vertDim;
            size_t col = i % heightmap.vertDim;
            double x = topLeftX + col * pixelDim;
            double y = topLeftY - row * pixelDim;
            double z = (double)heightmap.vertexHeights[i];
            vertexLocations.push_back({ x, y, z });
            lookup[i] = counter;
            counter++;
        }
    }
    vertexLocations.shrink_to_fit();
    for (size_t i = 0; i < triangles.size(); i++) {
        newTriangles[i] = { lookup[triangles[i][0]], lookup[triangles[i][1]], lookup[triangles[i][2]] };
    }
    return { pybind11::cast(vertexLocations), pybind11::cast(newTriangles) };
}