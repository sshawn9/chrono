// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <algorithm>

#include "chrono/utils/ChUtils.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace ch_precice {

ChPreciceAdapterMbs::ChPreciceAdapterMbs(ChSystem& sys, double time_step, bool verbose) : m_sys(&sys), m_time_step(time_step) {
    ChAssertAlways(m_sys->IsInitialized());

    SetVerbose(verbose);

    // Allocate space for checkpoint
    m_sys->Setup();
    auto np = m_sys->GetNumCoordsPosLevel();
    auto nv = m_sys->GetNumCoordsVelLevel();
    m_checkpoint.time = m_sys->GetChTime();
    m_checkpoint.x.setZero(np, m_sys);
    m_checkpoint.v.setZero(nv, m_sys);
}

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
ChPreciceAdapterMbs::ChPreciceAdapterMbs(const std::string& input_filename, bool verbose) {
    SetVerbose(verbose);

    // Create the MBS from the YAML specification file
    parsers::ChParserMbsYAML parser(input_filename, verbose);
    auto sys = parser.CreateSystem();
    parser.Populate(*sys);
    m_sys = sys.get();

    // Extract information from parsed YAML files
    m_vis.render = parser.Render();
    m_vis.render_fps = parser.GetRenderFPS();
    m_vis.camera_vertical = parser.GetCameraVerticalDir();
    m_vis.camera_location = parser.GetCameraLocation();
    m_vis.camera_target = parser.GetCameraTarget();
    m_vis.enable_shadows = parser.EnableShadows();

    // Read in preCICE participant configuration
    ConstructSolver(input_filename);

    // Check consistency between preCICE participant specification and the MBS
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);
    auto config = yaml["precice_adapter_config"];

    // Read names of interface physics items and check that they are defined in the MBS
    if (config["bodies"]) {
        auto bodies = config["bodies"];
        ChAssertAlways(bodies.IsSequence());
        for (int i = 0; i < bodies.size(); i++) {
            ChAssertAlways(bodies[i]["name"]);
            auto body_name = bodies[i]["name"].as<std::string>();
            auto body = parser.FindBodyByName(body_name);
            if (body) {
                m_bodies.push_back(body);
            } else {
                cerr << "No body named '" << body_name << "' was found in the MBS" << endl;
                throw std::runtime_error("Interface body not present in MBS");
            }
        }
    }

    if (config["meshes1d"]) {
        //// TODO
    }

    if (config["meshes2d"]) {
        //// TODO
    }

    if (m_verbose) {
    }
}
#endif

ChPreciceAdapterMbs::~ChPreciceAdapterMbs() {}

// -----------------------------------------------------------------------------

ChPreciceAdapterMbs::VisParams::VisParams()
    : render(false), render_fps(120), camera_vertical(CameraVerticalDir::Z), camera_location({0, -1, 0}), camera_target({0, 0, 0}), enable_shadows(true) {}

bool ChPreciceAdapterMbs::EnableVisualization(double render_fps,
                                              CameraVerticalDir camera_vertical,
                                              const ChVector3d& camera_location,
                                              const ChVector3d& camera_target,
                                              bool enable_shadows) {
#ifdef CHRONO_VSG
    m_vis.render_fps = render_fps;
    m_vis.camera_vertical = camera_vertical;
    m_vis.camera_location = camera_location;
    m_vis.camera_target = camera_target;
    m_vis.enable_shadows = enable_shadows;
    m_vis.render = true;
    return true;
#else
    cerr << "Chrono::VSG not enabled. Disabling run-time visualization" << endl;
    m_vis.render = false;
    return false;
#endif
}

void ChPreciceAdapterMbs::InitializeParticipant() {
    ChPreciceAdapter::InitializeParticipant();

    // Set number of Chrono physics items in the coupling interfaces
    //// TODO - set this from specified FSI data

    // Go through all interface meshes and
    // - check that coupling meshes have dimension 3 (as reported by preCICE)
    // - check that coupling data have dimension 3 (as reported by preCICE)
    // - set mesh vertices (depending on data type)
    // - register mesh with preCICE
    for (const auto& mesh_name : GetMeshNames()) {
        ChAssertAlways(GetMeshDimensions(mesh_name) == 3);

        for (const auto& data_name : GetReadDataNamesOnMesh(mesh_name)) {
            ChAssertAlways(GetDataDimensions(mesh_name, data_name) == 3);
        }

        std::vector<ChVector3d> vertices;
        auto& mesh_info = m_coupling_meshes[mesh_name];
        switch (mesh_info.type) {
            case MeshType::RIGID_BODY_REF_POINTS: {
                for (const auto& body : m_bodies)
                    vertices.push_back(body->GetPos());
                break;
            }
            case MeshType::RIGID_BODY_MESH_POINTS: {
                ////break;
                throw std::runtime_error("MeshType::RIGID_BODY_MESH_POINTS not yet implemented");
            }
            case MeshType::FEA_MESH1D_NODES: {
                ////break;
                throw std::runtime_error("MeshType::FEA_MESH1D_NODES not yet implemented");
            }
            case MeshType::FEA_MESH2D_NODES: {
                ////break;
                throw std::runtime_error("MeshType::FEA_MESH2D_NODES not yet implemented");
            }
        }

        // Register coupling mesh with preCICE
        RegisterMesh(mesh_name, vertices);
    }

#ifdef CHRONO_VSG
    // Enable runtime visualization
    if (m_vis.render) {
        m_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        m_vsg->AttachSystem(m_sys);
        m_vsg->SetWindowTitle("Chrono preCICE MBS participant - " + m_participant_name);
        m_vsg->AddCamera(m_vis.camera_location, m_vis.camera_target);
        m_vsg->SetWindowSize(1280, 800);
        m_vsg->SetWindowPosition(100, 100);
        m_vsg->SetCameraVertical(m_vis.camera_vertical);
        m_vsg->SetCameraAngleDeg(40.0);
        m_vsg->SetLightIntensity(1.0f);
        m_vsg->SetLightDirection(-CH_PI_4, CH_PI_4);
        m_vsg->EnableShadows(m_vis.enable_shadows);
        m_vsg->ToggleAbsFrameVisibility();
        m_vsg->Initialize();
    }
#endif
}

void ChPreciceAdapterMbs::WriteCheckpoint(double time) {
    ChPreciceAdapter::WriteCheckpoint(time);

    double sys_time;
    m_sys->StateGather(m_checkpoint.x, m_checkpoint.v, sys_time);
    assert(time == sys_time);
}

void ChPreciceAdapterMbs::ReadCheckpoint(double time) {
    ChPreciceAdapter::ReadCheckpoint(time);

    ChAssertAlways(m_checkpoint.time == time);
    m_sys->StateScatter(m_checkpoint.x, m_checkpoint.v, m_checkpoint.time, UpdateFlags::UPDATE_ALL);
}

void ChPreciceAdapterMbs::ReadData() {
    ChPreciceAdapter::ReadData();

    for (const auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case MeshType::RIGID_BODY_REF_POINTS:
                ReadBodyRefData(mesh_name, mesh_info);
                break;
            case MeshType::RIGID_BODY_MESH_POINTS:
                //// TODO
                break;
            case MeshType::FEA_MESH1D_NODES:
                //// TODO
                break;
            case MeshType::FEA_MESH2D_NODES:
                //// TODO
                break;
        }
    }
}

double ChPreciceAdapterMbs::GetSolverTimeStep(double max_time_step) const {
    return std::min(m_time_step, max_time_step);
}

void ChPreciceAdapterMbs::AdvanceParticipant(double time, double time_step) {
    ChPreciceAdapter::AdvanceParticipant(time, time_step);

    ChAssertAlways(time == m_sys->GetChTime());

    static int render_frame = 0;
    if (m_vis.render && m_vsg->Run()) {
        if (time >= render_frame / m_vis.render_fps) {
            m_vsg->Render();
            render_frame++;
        }
    }

    m_sys->DoStepDynamics(time_step);
}

void ChPreciceAdapterMbs::WriteData() {
    ChPreciceAdapter::WriteData();

    for (auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        switch (mesh_info.type) {
            case MeshType::RIGID_BODY_REF_POINTS:
                WriteBodyRefData(mesh_name, mesh_info);
                break;
            case MeshType::RIGID_BODY_MESH_POINTS:
                //// TODO
                break;
            case MeshType::FEA_MESH1D_NODES:
                //// TODO
                break;
            case MeshType::FEA_MESH2D_NODES:
                //// TODO
                break;
        }
    }
}

// -----------------------------------------------------------------------------

void ChPreciceAdapterMbs::ReadBodyRefData(const std::string& mesh_name, const MeshInfo& mesh_info) {
    for (const auto& data_name : m_data_read[mesh_name]) {
        const auto& data_info = mesh_info.data.at(data_name);
        auto data_type = data_info.type;
        const auto& data_values = data_info.values;
        switch (data_type) {
            case DataType::FORCES:
                for (int i = 0; i < m_bodies.size(); i++) {
                    ChVector3d force;
                    force.x() = data_values[3 * i + 0];
                    force.x() = data_values[3 * i + 1];
                    force.x() = data_values[3 * i + 2];
                }
                //// TODO -- apply force
                break;
            case DataType::TORQUES:
                for (int i = 0; i < m_bodies.size(); i++) {
                    ChVector3d torque;
                    torque.x() = data_values[3 * i + 0];
                    torque.x() = data_values[3 * i + 1];
                    torque.x() = data_values[3 * i + 2];
                }
                //// TODO -- apply torque
                break;
            default:
                throw std::runtime_error("Invalid read data type for MBS");
        }
    }
}

void ChPreciceAdapterMbs::WriteBodyRefData(const std::string& mesh_name, MeshInfo& mesh_info) {
    for (const auto& data_name : m_data_write[mesh_name]) {
        auto& data_info = mesh_info.data[data_name];
        auto data_type = data_info.type;
        auto& data_values = data_info.values;
        if (m_verbose) {
            cout << m_prefix2 << "'" << data_name << "' (" << GetDataDimensions(mesh_name, data_name) << "," << GetDataTypeAsString(mesh_name, data_name) << ")" << endl;
        }
        switch (data_type) {
            case DataType::POSITIONS:
                for (int i = 0; i < m_bodies.size(); i++) {
                    const auto& pos = m_bodies[i]->GetFrameRefToAbs().GetPos();
                    data_values[3 * i + 0] = pos.x();
                    data_values[3 * i + 1] = pos.y();
                    data_values[3 * i + 2] = pos.z();
                }
                break;
            case DataType::VELOCITIES:
                for (int i = 0; i < m_bodies.size(); i++) {
                    const auto& vel = m_bodies[i]->GetFrameRefToAbs().GetPosDt();
                    data_values[3 * i + 0] = vel.x();
                    data_values[3 * i + 1] = vel.y();
                    data_values[3 * i + 2] = vel.z();
                }
                break;
            default:
                throw std::runtime_error("Invalid write data type for MBS");
        }
    }
}

}  // end namespace ch_precice
}  // namespace chrono
