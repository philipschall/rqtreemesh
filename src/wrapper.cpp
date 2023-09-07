#include "rqtreemesh.h"

PYBIND11_MODULE(_rqtreemesh, m) {
    m.def("generatemesh", &RestrictedQuadTreeTriangulation);
}