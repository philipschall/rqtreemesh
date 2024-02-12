#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <iostream>
#include "quadtree.h"

constexpr auto PBSTR = "############################################################";
constexpr int PBWIDTH = 60;

void PrintProgress(int val) {
    int lpad = (int)((double)val / 100 * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\rGenerating Mesh: %3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

void MarkVertices(const Level& level, const std::vector<std::int64_t>& vertices, std::vector<bool>& markedVertices,
    std::vector<bool>& marked, const bool& isCenter, const int& triangulationOffset, const float& maxError,
    std::size_t& count, std::size_t& countdown, const std::size_t& countdownInterval, const bool& progress, const std::size_t& total) {
    std::vector<float> triangulation;
    bool isTriangulated = false;
    for (const std::int64_t& index : vertices) {
        if (progress) {
            if (countdown == 0) {
                PrintProgress((int)(100.0 * count / total));
                countdown = countdownInterval;
            }
            count++;
            countdown--;
        }
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
        for (std::int64_t& neighbourIndex : vertex.Neighbours()) {
            marked[neighbourIndex] = true;
        }
        marked[index] = false;
    }
}

std::pair<pybind11::array, pybind11::array> RestrictedQuadTreeTriangulation(pybind11::array_t<float> array,
    int maxDepth, std::int64_t vertDimX, std::int64_t vertDimY, std::int64_t minVertDim, float maxError, double pixelDim, double topLeftX, double topLeftY, bool progress) {
    Heightmap heightmap = Heightmap(array, maxDepth, vertDimX, vertDimY, minVertDim);
    std::vector<bool> markedVertices(vertDimX * vertDimY, false);
    std::vector<bool> marked(vertDimX * vertDimY, false);
    std::size_t total = vertDimX * vertDimY - 2;
    std::size_t count = 0;
    std::size_t countdown = 0;
    std::size_t countdownInterval = total / 100;
    for (std::int64_t depth = maxDepth; depth > 0; depth--) {
        Level level = Level(depth, &heightmap);
        MarkVertices(level, level.BoundaryVerts(), markedVertices, marked, false, -1, maxError, count, countdown, countdownInterval, progress, total);
        MarkVertices(level, level.CenterVerts(), markedVertices, marked, true, -2, maxError, count, countdown, countdownInterval, progress, total);
    }
    if (progress) {
        PrintProgress(100);
        std::cout << "\n";
    }
    for (const std::int64_t& index : heightmap.initVerts) {
        markedVertices[index] = true;
    }
    std::vector<std::array<std::int64_t, 3>> triangles = Heightmap::CreateMesh(markedVertices, 2 * maxDepth, heightmap.initTris, heightmap.maxTris);
    std::vector<std::array<double, 3>> vertexLocations;
    vertexLocations.reserve(markedVertices.size());
    std::vector<std::array<std::size_t, 3>> newTriangles(triangles.size());
    std::vector<std::size_t> lookup(markedVertices.size(), 0);
    std::size_t counter = 0;
    for (std::size_t i = 0; i < markedVertices.size(); i++) {
        if (markedVertices[i]) {
            std::size_t row = i / vertDimX;
            std::size_t col = i % vertDimX;
            double x = topLeftX + col * pixelDim;
            double y = topLeftY - row * pixelDim;
            double z = (double)heightmap.vertexHeights[i];
            vertexLocations.push_back({ x, y, z });
            lookup[i] = counter;
            counter++;
        }
    }
    vertexLocations.shrink_to_fit();
    for (std::size_t i = 0; i < triangles.size(); i++) {
        newTriangles[i] = { lookup[triangles[i][0]], lookup[triangles[i][1]], lookup[triangles[i][2]] };
    }
    return { pybind11::cast(vertexLocations), pybind11::cast(newTriangles) };
}

PYBIND11_MODULE(_rqtreemesh, m) {
    m.def("generatemesh", &RestrictedQuadTreeTriangulation);
}