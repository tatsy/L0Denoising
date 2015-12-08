#ifndef _DENOISING_H_
#define _DENOISING_H_

#define _USE_MATH_DEFINES
#include <cmath>

extern "C" {
#include "mt19937ar.h"
};

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;
typedef OpenMesh::Vec3d Vector3d;

void addNoise(Mesh& mesh);

void denoise(Mesh& mesh);

#endif
