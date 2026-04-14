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

ChPreciceAdapterMbs::ChPreciceAdapterMbs(ChSystem& sys, double time_step) : m_sys(sys), m_time_step(time_step) {
    ChAssertAlways(m_sys.IsInitialized());

    // Allocate space for checkpoint
    m_sys.Setup();
    auto np = m_sys.GetNumCoordsPosLevel();
    auto nv = m_sys.GetNumCoordsVelLevel();
    m_checkpoint.time = m_sys.GetChTime();
    m_checkpoint.x.setZero(np, &m_sys);
    m_checkpoint.v.setZero(nv, &m_sys);
}

ChPreciceAdapterMbs::~ChPreciceAdapterMbs() {}

void ChPreciceAdapterMbs::InitializeSolver() {
    // Go through all interface meshes and check consistency
    for (const auto& mesh_name : GetMeshNames()) {
        // All meshes in Chrono interfaces must have dimension 3
        ChAssertAlways(GetMeshDimensions(mesh_name) == 3);
        // All data in Chrono interfaces must have dimension 3
        for (const auto& data_name : GetReadDataNamesOnMesh(mesh_name)) {
            ChAssertAlways(GetDataDimensions(mesh_name, data_name) == 3);
        }
    }



    struct Interface {
        bool in_defined;
        bool out_defined;

    };

    Interface rigid_body_ref;
    Interface rigid_body_mesh;
    Interface contact_surf_1d;
    Interface contact_surf_2d;

    // Check types of output (write) data
    bool rigid_body_ref_point = false;
    bool rigid_body_mesh_points = false;
    bool contact_mesh1d_nodes = false;
    bool contact_mesh2d_nodes = false;
    for (const auto& [key, value] : m_data_write) {
        if (m_data_write[key].type == DataType::RIGID_BODY_REF_POINT)
            rigid_body_ref_point = true;
        if (m_data_write[key].type == DataType::RIGID_BODY_MESH_POINTS)
            rigid_body_mesh_points = true;
        if (m_data_write[key].type == DataType::CONTACT_MESH1D_NODES)
            contact_mesh1d_nodes = true;
        if (m_data_write[key].type == DataType::CONTACT_MESH2D_NODES)
            contact_mesh2d_nodes = true;
    }

    // Check types of input (read) data
    bool rigid_body_ref_force = false;
    bool rigid_body_mesh_forces = false;
    bool contact_mesh1d_forces = false;
    bool contact_mesh2d_forces = false;
    for (const auto& [key, value] : m_data_read) {
        if (m_data_write[key].type == DataType::RIGID_BODY_REF_FORCE)
            rigid_body_ref_force = true;
        if (m_data_write[key].type == DataType::RIGID_BODY_MESH_FORCES)
            rigid_body_mesh_forces = true;
        if (m_data_write[key].type == DataType::CONTACT_MESH1D_FORCES)
            contact_mesh1d_forces = true;
        if (m_data_write[key].type == DataType::CONTACT_MESH2D_FORCES)
            contact_mesh2d_forces = true;
    }

    // Set number of Chrono physics items in the coupling interface
    //// TODO - set this from specified FSI data
    int num_rigid_bodies = 1;
    int num_contact_mesh1d = 0;
    int num_contact_mesh2d = 0;

    // Check what interfaces are enabled
    m_use_rigid_body_ref_data = (num_rigid_bodies > 0) && rigid_body_ref_point && rigid_body_ref_force;
    m_use_rigid_body_mesh_data = (num_rigid_bodies > 0) && rigid_body_mesh_points && rigid_body_mesh_forces;
    m_use_contact_mesh1d_data = (num_contact_mesh1d > 0) && contact_mesh1d_nodes && contact_mesh1d_forces;
    m_use_contact_mesh2d_data = (num_contact_mesh2d > 0) && contact_mesh2d_nodes && contact_mesh2d_forces;

    // Size the meshes and data appropriately (note that )
    if (m_use_rigid_body_ref_data) {
    }
}

void ChPreciceAdapterMbs::WriteCheckpoint(double time) {
    ChPreciceAdapter::WriteCheckpoint(time);

    double sys_time;
    m_sys.StateGather(m_checkpoint.x, m_checkpoint.v, sys_time);
    assert(time == sys_time);
}

void ChPreciceAdapterMbs::ReadCheckpoint(double time) {
    ChPreciceAdapter::ReadCheckpoint(time);

    ChAssertAlways(m_checkpoint.time == time);
    m_sys.StateScatter(m_checkpoint.x, m_checkpoint.v, m_checkpoint.time, UpdateFlags::UPDATE_ALL);
}

void ChPreciceAdapterMbs::ReadData() {
    ChPreciceAdapter::ReadData();

    //// TODO
}

double ChPreciceAdapterMbs::GetSolverTimeStep(double max_time_step) const {
    return std::min(m_time_step, max_time_step);
}

void ChPreciceAdapterMbs::AdvanceSolver(double time, double time_step) {
    ChPreciceAdapter::AdvanceSolver(time, time_step);

    ChAssertAlways(time == m_sys.GetChTime());
    m_sys.DoStepDynamics(time_step);
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
