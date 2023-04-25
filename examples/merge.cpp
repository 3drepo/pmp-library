// Copyright 2011-2019 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/Decimation.h>
#include <pmp/algorithms/Merge.h>
#include <pmp/algorithms/Manifold.h>
#include <pmp/algorithms/Normals.h>
#include <pmp/algorithms/Triangulation.h>
#include <pmp/algorithms/GraphViz.h>
#include <pmp/utilities.h>
#include <imgui.h>

#include <unordered_set>

using namespace pmp;

class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

    void create_mesh() { 
        create_open_cube();
    }

    void create_open_cube() {
        auto v0 = mesh_.add_vertex(pmp::Point(-0.5, -0.5, -0.5));
        auto v1 = mesh_.add_vertex(pmp::Point(0.5, -0.5, -0.5));
        auto v2 = mesh_.add_vertex(pmp::Point(0.5, 0.5, -0.5));
        auto v3 = mesh_.add_vertex(pmp::Point(-0.5, 0.5, -0.5));
        auto v4 = mesh_.add_vertex(pmp::Point(-0.5, 0.5, 0.5));
        auto v5 = mesh_.add_vertex(pmp::Point(0.5, 0.5, 0.5));
        auto v6 = mesh_.add_vertex(pmp::Point(0.5, -0.5, 0.5));
        auto v7 = mesh_.add_vertex(pmp::Point(-0.5, -0.5, 0.5));

        mesh_.add_triangle(v0, v2, v1);
        mesh_.add_triangle(v0, v3, v2);
        mesh_.add_triangle(v2, v3, v4);
        mesh_.add_triangle(v2, v4, v5);
        mesh_.add_triangle(v1, v2, v5);
        mesh_.add_triangle(v1, v5, v6);
        mesh_.add_triangle(v0, v7, v4);
        mesh_.add_triangle(v0, v4, v3);
        mesh_.add_triangle(v5, v4, v7);
        mesh_.add_triangle(v5, v7, v6);
        mesh_.add_triangle(v0, v6, v7);
        mesh_.add_triangle(v0, v1, v6);

        remove_incident_faces();

        prepare_mesh(mesh_);

        update_mesh();
    }

    void highlight_boundary_edges()
    {
        auto colors = mesh_.get_halfedge_property<pmp::Color>("h:color");
        if (!colors)
        {
            colors = mesh_.add_halfedge_property<pmp::Color>("h:color");
        }

        auto features = mesh_.get_edge_property<bool>("e:feature");
        if (!features)
        {
            features = mesh_.add_edge_property<bool>("e:feature");
        }

        for (auto h : mesh_.halfedges())
        {
            if (mesh_.is_boundary(mesh_.edge(h)))
            {
                colors[h] = Color(1, 0, 0);
                features[mesh_.edge(h)] = true;
            }
            else
            {
                colors[h] = Color(0, 0, 0);
                features[mesh_.edge(h)] = false;
            }
        }

        update_mesh();
    }

    void prepare_mesh(SurfaceMesh& b)
    {
        auto a = SurfaceMesh(b);
        b.clear();
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

    void remove_incident_faces()
    {
        auto v = *mesh_.vertices_begin();
        std::vector<pmp::Face> faces;
        for (auto f : mesh_.faces(v))
        {
            faces.push_back(f);
        }
        mesh_.delete_face(faces[0]);
        mesh_.delete_face(faces[1]);
    }

protected:
    void mouse(int button, int action, int mods)
    {
        if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT &&
            shift_pressed())
        {
            double x, y;
            cursor_pos(x, y);

            Vertex v = pick_vertex(x, y);
            if (mesh_.is_valid(v))
            {
                GraphViz viz(mesh_);

                // Find all other vertices
                for (auto vb : mesh_.vertices())
                {
                    if (distance(mesh_.position(vb), mesh_.position(v)) <
                        0.0001)
                    {
                        viz.print_topology(vb);
                    }
                }
            }
        }
        else
        {
            MeshViewer::mouse(button, action, mods);
        }
    }

    void process_imgui() override;
};

Viewer::Viewer(const char* title, int width, int height)
    : MeshViewer(title, width, height)
{
    set_draw_mode("Hidden Line");
    crease_angle_ = 0.0;
}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::Button("Merge"))
    {
        pmp::check_mesh(mesh_);

        pmp::Manifold manifold(mesh_);
        manifold.fix_manifold();

        pmp::check_mesh(mesh_);

        pmp::Merge merge(mesh_);
        merge.merge();

        highlight_boundary_edges();

        pmp::check_mesh(mesh_);

        update_mesh();
    }

    if (ImGui::Button("Manifold"))
    {
        pmp::check_mesh(mesh_);

        Manifold f(mesh_);
        f.fix_manifold();

        pmp::check_mesh(mesh_);

        mesh_.garbage_collection();
        update_mesh();
    }

    if (ImGui::CollapsingHeader("Decimation", ImGuiTreeNodeFlags_DefaultOpen))
    {
        static int target_percentage = 10;
        ImGui::PushItemWidth(100);
        ImGui::SliderInt("Percentage", &target_percentage, 1, 99);
        ImGui::PopItemWidth();

        static int normal_deviation = 30;
        ImGui::PushItemWidth(100);
        ImGui::SliderInt("Normal Deviation", &normal_deviation, 1, 180);
        ImGui::PopItemWidth();

        static int aspect_ratio = 0;
        ImGui::PushItemWidth(100);
        ImGui::SliderInt("Aspect Ratio", &aspect_ratio, 0, 1000);
        ImGui::PopItemWidth();

        if (ImGui::Button("Decimate"))
        {
            try
            {
                Decimation decimater(mesh_);
                decimater.initialize(aspect_ratio, 0.0, 0.0, normal_deviation,
                                     0.0);
                decimater.decimate(mesh_.n_vertices() * 0.01 *
                                   target_percentage);
            }
            catch (const InvalidInputException& e)
            {
                std::cerr << e.what() << std::endl;
                return;
            }
            mesh_.garbage_collection();
            update_mesh();
        }
    }
}

int main(int argc, char** argv)
{
#ifndef __EMSCRIPTEN__
    Viewer window("Merge and Decimate", 800, 600);
    if (argc == 2)
        window.load_mesh(argv[1]);
    else
        window.create_open_cube();

    window.highlight_boundary_edges();

    return window.run();
#else
    Viewer window("Decimation", 800, 600);
    window.load_mesh(argc == 2 ? argv[1] : "input.off");
    return window.run();
#endif
}
