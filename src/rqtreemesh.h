#ifndef RQTREEMESH_H
#define RQTREEMESH_H

#include<pybind11/stl.h>
#include<pybind11/numpy.h>

std::pair<pybind11::array, pybind11::array> RestrictedQuadTreeTriangulation(pybind11::array_t<float> array, int maxDepth, float maxError, double pixelDim, double topLeftX, double topLeftY);

#endif /* RQTREEMESH_H */
