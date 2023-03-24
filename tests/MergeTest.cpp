// Copyright 2017-2021 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include "gtest/gtest.h"

#include "pmp/algorithms/Merge.h"
#include "pmp/algorithms/Normals.h"
#include "Helpers.h"

using namespace pmp;

// This function takes a well-formed mesh and splits it up by face to emulate
// a mesh with per-vertex normals that would be typically be encountered where
// there is no support for per-edge or per-face normals.
void prepare_mesh(SurfaceMesh a, SurfaceMesh& b) 
{
    auto a_p = a.get_vertex_property<pmp::Point>("v:point");

    for (auto f : a.faces())
    {
        std::vector<Vertex> b_vertices;
        for (auto v : a.vertices(f))
        {
            b_vertices.push_back(b.add_vertex(a_p[v]));
        }
        b.add_face(b_vertices);
    }

    Normals::compute_vertex_normals(b);
}

void remove_faces(SurfaceMesh& mesh, std::vector<size_t> indices)
{
    auto v = *mesh.vertices_begin();
    std::vector<pmp::Face> faces;
    for (auto f : mesh.faces(v))
    {
        faces.push_back(f);
    }
    for (auto i : indices)
    {
        mesh.delete_face(faces[i]);
    }
}

// This function checks that all primitives (vertices, halfedges, faces) only
// point to valid indices, by collecting valid indices from the enumerators
// and checking any references against these.
void check_references(SurfaceMesh& mesh) 
{
    std::set<pmp::IndexType> vertex_indices;
    for (auto v : mesh.vertices())
    {
        EXPECT_TRUE(v.is_valid());
        vertex_indices.insert(v.idx());
    }

    size_t face_valence_sum = 0;

    std::set<pmp::IndexType> face_indices;
    for (auto f : mesh.faces())
    {
        EXPECT_TRUE(f.is_valid());
        face_indices.insert(f.idx());
        face_valence_sum += mesh.valence(f);
    }

    std::set<pmp::IndexType> edge_indices;
    for (auto e : mesh.halfedges())
    {
        EXPECT_TRUE(e.is_valid());
        edge_indices.insert(e.idx());
    }

    for (auto v : mesh.vertices())
    {
        EXPECT_GT(edge_indices.count(mesh.halfedge(v).idx()), 0);
        for (auto h : mesh.halfedges(v))
        {
            EXPECT_TRUE(h.is_valid());
            EXPECT_GT(edge_indices.count(h.idx()), 0);
        }
        for(auto f : mesh.faces(v))
        {
            if (f.is_valid())
            {
                EXPECT_GT(face_indices.count(f.idx()), 0);
            }
        }
    }

    size_t face_halfedge_references_sum = 0;
    for (auto h : mesh.halfedges())
    {
        EXPECT_TRUE(mesh.from_vertex(h).is_valid());
        EXPECT_TRUE(mesh.to_vertex(h).is_valid());
        EXPECT_GT(vertex_indices.count(mesh.from_vertex(h).idx()), 0);
        EXPECT_GT(vertex_indices.count(mesh.to_vertex(h).idx()), 0);
        if (mesh.face(h).is_valid())
        {
            EXPECT_GT(face_indices.count(mesh.face(h).idx()), 0);
            face_halfedge_references_sum++;
        }
        if (!mesh.is_boundary(h)) // Edges without a face may still have next and prev edges where there is a hole, but all faces must be closed.
        {
            EXPECT_TRUE(mesh.next_halfedge(h).is_valid());
            EXPECT_TRUE(mesh.prev_halfedge(h).is_valid());
            EXPECT_GT(edge_indices.count(mesh.next_halfedge(h).idx()), 0);
            EXPECT_GT(edge_indices.count(mesh.prev_halfedge(h).idx()), 0);
        }
    }

    for (auto f : mesh.faces())
    {
        for(auto h : mesh.halfedges(f))
        {
            EXPECT_TRUE(h.is_valid());
            EXPECT_GT(edge_indices.count(h.idx()), 0);
        }
    }

    EXPECT_EQ(face_valence_sum, face_halfedge_references_sum);
}

// This tests a number of preconditions to the actual tests, such that the
// procedurally generated meshes are as expected.
TEST(MergeTest, pretests) 
{
    auto c = triangle_cube();

    EXPECT_EQ(c.n_vertices(), 8);
    EXPECT_EQ(c.n_edges(), 18);
    EXPECT_EQ(c.n_halfedges(), 36);
    EXPECT_EQ(c.n_faces(), 12);

    auto q = quad_cube();

    EXPECT_EQ(q.n_vertices(), 8);
    EXPECT_EQ(q.n_edges(), 12);
    EXPECT_EQ(q.n_halfedges(), 24);
    EXPECT_EQ(q.n_faces(), 6);
}

// This tests that the merge performs in the typical use case. It is expected
// to reduce the vertex and edge count of the subdivided mesh.
TEST(MergeTest, simple)
{
    // This snippet creates a mesh consisting of a soup of isolated triangles
    SurfaceMesh mesh;
    prepare_mesh(subdivided_icosahedron(), mesh);

    check_references(mesh);

    // This snippet and its counterpart check the number of edges for a face,
    // only vertices and edges should be merged: collapsing edges or combining
    // faces is different operation (decimation), so the faces should remain
    // unchanged.

    auto n_faces = mesh.n_faces();
    std::unordered_map<int, int> valence;
    for (auto f : mesh.faces())
    {
        valence[f.idx()] = mesh.valence(f);
    }

    size_t num_vertices_before = mesh.n_vertices();
    size_t num_edges_before = mesh.n_halfedges();

    Merge merge(mesh);
    merge.merge();

    check_references(mesh);

    mesh.garbage_collection();

    check_references(mesh);

    EXPECT_EQ(mesh.n_faces(), n_faces);
    for (auto f : mesh.faces())
    {
        EXPECT_EQ(mesh.valence(f), valence[f.idx()]);
    }

    // For this test, we expect the count to decrease because the per-vertex normals
    // are within the default range.
    EXPECT_LT(mesh.n_vertices(), num_vertices_before);
    EXPECT_LT(mesh.n_halfedges(), num_edges_before);
}


// This tests the merge against a triangle mesh with sharp edges, only some of
// which should be merged.
TEST(MergeTest, tricube) 
{
    SurfaceMesh mesh;
    prepare_mesh(triangle_cube(), mesh);

    Merge merge(mesh);
    merge.merge();

    mesh.garbage_collection();

    check_references(mesh);

    // During the merge, two vertices of each side can be combined: the ones
    // at the ends of the diagonal edges.
    // This should result in each side (6 in total) having 4 vertices, and 5 
    // edges.

    EXPECT_EQ(mesh.n_vertices(), 24); // 4 vertices per each of six sides
    EXPECT_EQ(mesh.n_edges(), 30); // 5 edges per each of six sides
    EXPECT_EQ(mesh.n_halfedges(), 60); // Double the number of half edges
    EXPECT_EQ(mesh.n_faces(), 12); // We don't remove the diagonal, simply combine the two co-located boundaries, so there are still 12 faces
}

// This tests the merge against a quad mesh with sharp edges. The nature of the
// quad mesh means no edges are viable for merging, and so the mesh should 
// remain unchanged.
TEST(MergeTest, quadcube)
{
    SurfaceMesh mesh;
    prepare_mesh(quad_cube(), mesh);

    Merge merge(mesh);
    merge.merge();

    mesh.garbage_collection();

    check_references(mesh);

    // Merging a quad cube should result in a no-op, as none of the vertices
    // can be combined, and all edges have unique start and end points.

    EXPECT_EQ(mesh.n_vertices(), 24);  // 4 vertices per each of six sides
    EXPECT_EQ(mesh.n_edges(), 24);     // 4 edges per each of six sides
    EXPECT_EQ(mesh.n_halfedges(), 48); // Double the number of half edges
    EXPECT_EQ(mesh.n_faces(), 6); // One quad per side
}

// This tests the merge against a sharp angled quad mesh again, but this time
// with the angle threshold set very high. As a result, the mesh should become
// the simplest possible unit cube.
TEST(MergeTest, high_angle_threshold)
{
    SurfaceMesh mesh;
    prepare_mesh(quad_cube(), mesh);

    Merge merge(mesh);
    merge.merge(180);

    mesh.garbage_collection();

    // Merging a quad cube should result in a no-op, as none of the vertices
    // can be combined, and all edges have unique start and end points.

    EXPECT_EQ(mesh.n_vertices(), 8);  // 4 vertices per each of six sides
    EXPECT_EQ(mesh.n_edges(), 12);     // 4 edges per each of six sides
    EXPECT_EQ(mesh.n_halfedges(), 24); // Double the number of half edges
    EXPECT_EQ(mesh.n_faces(), 6);      // One quad per side
}

// This tests the merge against a mesh with one large hole
TEST(MergeTest, open_face)
{
    auto c = triangle_cube();

    // Pick two incident faces around a single vertex to create a hole with more
    // than three edges.
    remove_faces(c, {0, 1});

    c.garbage_collection();

    SurfaceMesh mesh;
    prepare_mesh(c, mesh);

    // After pre-processing the mesh it should have 12 faces, with each face
    // having three vertices, and three edges.

    Merge merge(mesh);
    merge.merge();

    mesh.garbage_collection();

    check_references(mesh);

    // In the case where the cube is missing one side, we expect 4 vertices and
    // 5 edges for each complete side, and nothing else.

    EXPECT_EQ(mesh.n_vertices(), 20);
    EXPECT_EQ(mesh.n_edges(), 25);
    EXPECT_EQ(mesh.n_halfedges(), 50);
    EXPECT_EQ(mesh.n_faces(), 10);
}

// This tests the merge against a mesh with two holes, both incident to the 
// same vertex.
TEST(MergeTest, open_faces)
{
    auto c = triangle_cube();

    // Pick two faces around a single vertex to remove to create multiple holes.
    remove_faces(c, {0, 3});

    c.garbage_collection();

    SurfaceMesh mesh;
    prepare_mesh(c, mesh);

    // After pre-processing the mesh it should have 12 faces, with each face
    // having three vertices, and three edges.

    Merge merge(mesh);
    merge.merge();

    mesh.garbage_collection();

    check_references(mesh);

    // In the case where the cube is missing two faces and with the angle
    // threshold low, we would expect 22 vertices: 16 for the 4 complete sides,
    // and three for the 2 incomplete sides.

    EXPECT_EQ(mesh.n_vertices(), 22);   
    EXPECT_EQ(mesh.n_edges(), 26);     
    EXPECT_EQ(mesh.n_halfedges(), 52); 
    EXPECT_EQ(mesh.n_faces(), 10);      
}