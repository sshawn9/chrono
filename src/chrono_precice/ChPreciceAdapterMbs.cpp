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
            auto body = m_sys->SearchBody(body_name);
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

void ChPreciceAdapterMbs::InitializeParticipant() {
    ChPreciceAdapter::InitializeParticipant();

    // Set number of Chrono physics items in the coupling interfaces
    //// TODO - set this from specified FSI data

    // Go through all interface meshes and
    // - check that coupling meshes have dimension 3
    // - check that coupling data have dimension 3
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

    //// TODO
}

double ChPreciceAdapterMbs::GetSolverTimeStep(double max_time_step) const {
    return std::min(m_time_step, max_time_step);
}

void ChPreciceAdapterMbs::AdvanceParticipant(double time, double time_step) {
    ChPreciceAdapter::AdvanceParticipant(time, time_step);

    ChAssertAlways(time == m_sys->GetChTime());
    m_sys->DoStepDynamics(time_step);
}

void ChPreciceAdapterMbs::WriteData() {
    ChPreciceAdapter::WriteData();

    //// TODO
}

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

}  // end namespace ch_precice
}  // namespace chrono
