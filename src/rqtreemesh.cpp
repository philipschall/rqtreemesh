#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <iostream>
#include "quadtree.h"
#include "triangulation.h"

constexpr auto PBSTR = "############################################################";
constexpr auto PBWIDTH = 60;

void PrintProgress(int val) {
    int lpad = (int)((double)val / 100 * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\rGenerating Mesh: %3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

void MarkVertices(const Level& level, const std::vector<std::size_t>& vertices, std::vector<bool>& markedVertices,
    std::vector<bool>& marked, const bool& isCenter, const int& triangulationOffset, const float& maxError,
    std::size_t& count, std::size_t& countdown, const std::size_t& countdownInterval, const bool& progress, const std::size_t& total) {
    std::vector<float> triangulation;
    bool isTriangulated = false;
    for (const std::size_t& index : vertices) {
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
        for (std::size_t& neighbourIndex : vertex.Neighbours()) {
            marked[neighbourIndex] = true;
        }
        marked[index] = false;
    }
}

std::pair<pybind11::array, pybind11::array> RestrictedQuadTreeTriangulation(pybind11::array_t<float> array,
    int maxDepth, float maxError, double pixelDim, double topLeftX, double topLeftY, bool progress) {
    Heightmap heightmap = Heightmap(array, maxDepth);
    std::size_t vertDim = heightmap.vertDim;
    std::vector<bool> markedVertices(vertDim * vertDim, false);
    std::vector<bool> marked(vertDim * vertDim, false);
    std::size_t total = vertDim * vertDim - 2;
    std::size_t count = 0;
    std::size_t countdown = 0;
    std::size_t countdownInterval = total / 100;
    for (int depth = heightmap.maxDepth; depth > 0; depth--) {
        Level level = Level(depth, &heightmap);
        MarkVertices(level, level.BoundaryVerts(), markedVertices, marked, false, -1, maxError, count, countdown, countdownInterval, progress, total);
        MarkVertices(level, level.CenterVerts(), markedVertices, marked, true, -2, maxError, count, countdown, countdownInterval, progress, total);
    }
    if (progress) {
        PrintProgress(100);
    }
    markedVertices[0] = true;
    markedVertices[vertDim - 1] = true;
    markedVertices[vertDim * (vertDim - 1)] = true;
    markedVertices.back() = true;
    std::vector<std::array<std::size_t, 3>> triangles = CreateMesh(markedVertices, 2 * maxDepth, vertDim);
    std::vector<std::array<double, 3>> vertexLocations;
    vertexLocations.reserve(markedVertices.size());
    std::vector<std::array<std::size_t, 3>> newTriangles(triangles.size());
    std::vector<std::size_t> lookup(markedVertices.size(), 0);
    std::size_t counter = 0;
    for (std::size_t i = 0; i < markedVertices.size(); i++) {
        if (markedVertices[i]) {
            std::size_t row = i / vertDim;
            std::size_t col = i % vertDim;
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