// Copyright 2018-2021 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include "gtest/gtest.h"

#include "pmp/algorithms/Normals.h"
#include "pmp/algorithms/Shapes.h"
#include <vector>

using namespace pmp;

TEST(NormalsTest, compute_vertex_normals)
{
    auto mesh = Shapes::icosahedron();
    Normals::compute_vertex_normals(mesh);
    auto vnormals = mesh.get_vertex_property<Normal>("v:normal");
    auto vn0 = vnormals[Vertex(0)];
    EXPECT_GT(norm(vn0), 0);
}

TEST(NormalsTest, compute_face_normals)
{
    auto mesh = Shapes::icosahedron();
    Normals::compute_face_normals(mesh);
    auto fnormals = mesh.get_face_property<Normal>("f:normal");
    auto fn0 = fnormals[Face(0)];
    EXPECT_GT(norm(fn0), 0);
}

TEST(NormalsTest, compute_corner_normal)
{
    auto mesh = Shapes::icosahedron();
    auto h = Halfedge(0);
    auto n = Normals::compute_corner_normal(mesh, h, (Scalar)M_PI / 3.0);
    EXPECT_GT(norm(n), 0);
}

TEST(NormalsTest, polygonal_face_normal)
{
    SurfaceMesh mesh;
    std::vector<Vertex> vertices(5);
    vertices[0] = mesh.add_vertex(Point(0, 0, 0));
    vertices[1] = mesh.add_vertex(Point(1, 0, 0));
    vertices[2] = mesh.add_vertex(Point(1, 1, 0));
    vertices[3] = mesh.add_vertex(Point(0.5, 1, 0));
    vertices[4] = mesh.add_vertex(Point(0, 1, 0));
    auto f0 = mesh.add_face(vertices);
    auto n0 = Normals::compute_face_normal(mesh, f0);
    EXPECT_GT(norm(n0), 0);
}
