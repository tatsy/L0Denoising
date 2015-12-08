#include <set>
using namespace std;

#include "Denoising.h"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

typedef Eigen::MatrixXd Matrix;
typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> Triplet;

bool operator<(const Vector3d& v1, const Vector3d& v2) {
	if(v1[0] != v2[0]) return v1[0] < v2[0];
	if(v1[1] != v2[1]) return v1[1] < v2[1];
	return v1[2] < v1[2];
}

bool operator>(const Vector3d& v1, const Vector3d& v2) {
	if(v1[0] != v2[0]) return v1[0] > v2[0];
	if(v1[1] != v2[1]) return v1[1] > v2[1];
	return v1[2] > v1[2];
}

double genrand_gauss(double mu, double sigma) {
	double z = sqrt(-2.0 * log(genrand_real2())) * sin(2.0 * M_PI * genrand_real2());
	return mu + sigma * z;
}

void addNoise(Mesh& mesh) {
	init_genrand((unsigned long)time(NULL));
	OpenMesh::VPropHandleT<Mesh::Point> vprop;
	mesh.add_property(vprop);

	Mesh::VertexIter v_it;
	for(v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		double vx = genrand_gauss(0.0, 0.001);
		double vy = genrand_gauss(0.0, 0.001);
		double vz = genrand_gauss(0.0, 0.001);
		mesh.property(vprop, v_it) = mesh.point(v_it) + Mesh::Point(vx, vy, vz);
		mesh.set_point(v_it, mesh.property(vprop, v_it));
	}
	mesh.remove_property(vprop);
}

void denoise(Mesh& mesh) {
	// まず係数行列 R, D を用意する
	Mesh::EdgeIter e_it;
	Mesh::FaceVertexIter fv_it;

	int nVert = mesh.n_vertices();
	int nEdge = mesh.n_edges();

	vector<Triplet> tripR;
	vector<Triplet> tripD;

	int ie = 0;
	for(e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
		Mesh::HalfedgeHandle heh = mesh.halfedge_handle(e_it.handle(), 0);

		// 順番が変だけどこれであっている
		Mesh::VertexHandle vh1 = mesh.to_vertex_handle(heh);
		Mesh::VertexHandle vh3 = mesh.from_vertex_handle(heh);
		Mesh::VertexHandle vh2 = mesh.opposite_vh(heh);
		Mesh::VertexHandle vh4 = mesh.opposite_he_opposite_vh(heh);

		int i1 = vh1.idx();
		int i2 = vh2.idx();
		int i3 = vh3.idx();
		int i4 = vh4.idx();
		if(i1 == -1 || i2 == -1 || i3 == -1 || i4 == -1) {
			continue;
		}

		Mesh::Point p1 = mesh.point(vh1);
		Mesh::Point p2 = mesh.point(vh2);
		Mesh::Point p3 = mesh.point(vh3);
		Mesh::Point p4 = mesh.point(vh4);

		double S123 = 0.5 * OpenMesh::cross(p2 - p1, p2 - p3).norm();
		double S134 = 0.5 * OpenMesh::cross(p4 - p1, p4 - p3).norm();
		double l13  = (p1 - p3).norm();

		double coef1 = (S123 * OpenMesh::dot(p4 - p3, p3 - p1) + S134 * OpenMesh::dot(p1 - p3, p3 - p2)) / (l13 * l13 * (S123 + S134));
		double coef2 = S134 / (S123 + S134);
		double coef3 = (S123 * OpenMesh::dot(p3 - p1, p1 - p4) + S134 * OpenMesh::dot(p2 - p1, p1 - p3)) / (l13 * l13 * (S123 + S134));
		double coef4 = S123 / (S123 + S134);

		tripR.push_back(Triplet(ie, i1, coef1));
		tripR.push_back(Triplet(ie, i2, coef2));
		tripR.push_back(Triplet(ie, i3, coef3));
		tripR.push_back(Triplet(ie, i4, coef4));

		tripD.push_back(Triplet(ie, i1, 1.0));
		tripD.push_back(Triplet(ie, i2, -1.0));
		tripD.push_back(Triplet(ie, i3, 1.0));
		tripD.push_back(Triplet(ie, i4, -1.0));
		ie++;
	}

	SpMat R(nEdge, nVert);
	SpMat D(nEdge, nVert);
	R.setFromTriplets(tripR.begin(), tripR.end());
	D.setFromTriplets(tripD.begin(), tripD.end());

	// 点の座標をベクトルに入れる
	Matrix pInit(nVert, 3);
	Matrix pVec(nVert, 3);
	Mesh::VertexIter v_it;
	int ip = 0;
	for(v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		Mesh::Point p = mesh.point(v_it.handle());
		pInit(ip, 0) = p[0];
		pInit(ip, 1) = p[1];
		pInit(ip, 2) = p[2];

		pVec(ip, 0) = p[0];
		pVec(ip, 1) = p[1];
		pVec(ip, 2) = p[2];
		ip++;
	}

	// 疎な単位行列の用意
	vector<Triplet> tripI(nVert);
	for(int i=0; i<nVert; i++) {
		tripI[i] = Triplet(i, i, 1.0);
	}
	SpMat I(nVert, nVert);
	I.setFromTriplets(tripI.begin(), tripI.end());

	// 最適化問題を解く
	double lambda = 1.0e-2;
	double alpha  = 0.1;
	double beta   = 1.0e-3;
	double bmax   = 1.0;
	double mu     = 2.0;

	// Matrix pVec = pInit;
	Matrix dlt(nEdge, 3);
	while(beta < bmax) {
		// solve sub-problem for delta (shrinkage operator)
		Matrix y = D * pVec;
		for(int i=0; i<nEdge; i++) {
			dlt(i, 0) = 0.0;
			for(int d=0; d<3; d++) {
				dlt(i, 0) += y(i, d) * y(i, d);
			}
			
			if(dlt(i, 0) < lambda / beta) {
				dlt(i, 0) = 0.0;
			}

			dlt(i, 1) = dlt(i, 0);
			dlt(i, 2) = dlt(i, 0);
		}

		// solve sub-problem for points (linear system)
		SpMat  A = I + alpha * R.transpose() * R + beta * D.transpose() * D;
		Matrix b = pInit + beta * D.transpose() * dlt;

		Eigen::ConjugateGradient<SpMat> solver;
		solver.compute(A);
		pVec = solver.solve(b);
		if(solver.info() != Eigen::Success) {
			printf("Solve linear system failed.\n");
		}

		// compute difference
		//double diff = 0.0;
		//for(int i=0; i<nVert; i++) {
		//	for(int d=0; d<3; d++) {
		//		double del = pInit(i, d) - pVec(i, d);
		//		diff += del * del;
		//	}
		//}
		//printf("%f\n", diff);
		
		// update parameters
		printf(".");
		beta  *= mu;
		alpha *= 0.5;
	}
	printf("\n");

	// 点の情報を更新
	OpenMesh::VPropHandleT<Mesh::Point> vprop;
	mesh.add_property(vprop);
	ip = 0;
	for(v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		mesh.property(vprop, v_it) = Mesh::Point(pVec(ip, 0), pVec(ip, 1), pVec(ip, 2));
		mesh.set_point(v_it, mesh.property(vprop, v_it));
		ip++;
	}
	mesh.remove_property(vprop);
}
