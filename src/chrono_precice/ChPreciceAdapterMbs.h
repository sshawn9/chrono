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

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    #include "chrono_parsers/yaml/ChParserMbsYAML.h"
#endif

namespace chrono {
namespace ch_precice {

/// @addtogroup precice_module
/// @{

/// preCICE adapter for Chrono MBS simulation.
class ChApiPrecice ChPreciceAdapterMbs : public ChPreciceAdapter {
  public:
    ChPreciceAdapterMbs(ChSystem& sys, double time_step, bool verbose = false);

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    ChPreciceAdapterMbs(const std::string& input_filename, bool verbose = false);
#endif

    ~ChPreciceAdapterMbs();

    ChSystem& GetSystem() { return *m_sys; }

    virtual void InitializeParticipant() override;
    virtual void WriteCheckpoint(double time) override;
    virtual void ReadCheckpoint(double time) override;
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void WriteData() override;

  private:
    struct Checkpoint {
        double time;
        ChState x;
        ChStateDelta v;
    };

    void ReadBodyRefData(const std::string& mesh_name, const MeshInfo& mesh_info);
    void WriteBodyRefData(const std::string& mesh_name, MeshInfo& mesh_info);

    ChSystem* m_sys;
    double m_time_step;

    // System checkpoint data
    Checkpoint m_checkpoint;

    // Chrono physics items in coupling interface
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_bodies;

};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
