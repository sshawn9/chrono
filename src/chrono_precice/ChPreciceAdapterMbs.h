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

#ifndef CH_PRECICE_ADAPTER_MBS_H
#define CH_PRECICE_ADAPTER_MBS_H

#include "chrono_precice/ChPreciceAdapter.h"

#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChJoint.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/utils/ChBodyGeometry.h"

namespace chrono {
namespace ch_precice {

/// @addtogroup precice_module
/// @{

/// preCICE adapter for Chrono MBS simulation.
class ChApiPrecice ChPreciceAdapterMbs : public ChPreciceAdapter {
  public:
    ChPreciceAdapterMbs(ChSystem& sys, double time_step);
    ~ChPreciceAdapterMbs();

    virtual void InitializeSolver() override;
    virtual void WriteCheckpoint(double time) override;
    virtual void ReadCheckpoint(double time) override;
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceSolver(double time, double time_step) override;
    virtual void WriteData() override;

  private:
    struct Checkpoint {
        double time;
        ChState x;
        ChStateDelta v;
    };

    ChSystem& m_sys;
    double m_time_step;
    Checkpoint m_checkpoint;

    bool m_use_rigid_body_ref_data;
    bool m_use_rigid_body_mesh_data;
    bool m_use_contact_mesh1d_data;
    bool m_use_contact_mesh2d_data;
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
