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
//
// Utility functions for parsing YAML files.
//
// =============================================================================

#ifndef CH_UTILS_YAML_H
#define CH_UTILS_YAML_H

#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColormap.h"
#include "chrono/input_output/ChOutput.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/solver/ChSolver.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

enum class YamlDataPathType { ABS, REL };

/// Check the version specified in the given YAML node against the Chrono version.
/// Throw an exception if the versions are incompatible.
ChApi void CheckVersion(const YAML::Node& a);

/// Read the data path type (absolute or relative).
ChApi YamlDataPathType ReadDataPathType(const YAML::Node& a);

/// Load and return a ChVector3d from the specified node.
ChApi ChVector3d ReadVector(const YAML::Node& a);

/// Load and return a ChQuaternion from the specified node.
ChApi ChQuaterniond ReadQuaternion(const YAML::Node& a);

/// Load a Cardan angle sequence from the specified node and return as a quaternion.
/// The sequence is assumed to be extrinsic rotations X-Y-Z.
ChApi ChQuaterniond ReadCardanAngles(const YAML::Node& a, bool use_degrees);

/// Return a quaternion loaded from the specified node.
/// Data is assumed to provide a quaternion or a Cardan extrinsic X-Y-Z angle set.
ChApi ChQuaterniond ReadRotation(const YAML::Node& a, bool use_degrees);

/// Load and return a coordinate system from the specified node.
ChApi ChCoordsysd ReadCoordinateSystem(const YAML::Node& a, bool use_degrees);

/// Load and return a ChFunction object from the specified node.
ChApi std::shared_ptr<ChFunction> ReadFunction(const YAML::Node& a, bool use_degrees);

/// Load and return a ChColor from the specified node.
ChApi ChColor ReadColor(const YAML::Node& a);

/// Load and return a ChColor from the specified node.
ChApi ChColormap::Type ReadColorMapType(const YAML::Node& a);

/// Load and return a VisualizationType from the specified node.
ChApi VisualizationType ReadVisualizationType(const YAML::Node& a);

/// Load and return the output type from the specified node.
ChApi ChOutput::Type ReadOutputType(const YAML::Node& a);

/// Load and return the output mode from the specified node.
ChApi ChOutput::Mode ReadOutputMode(const YAML::Node& a);

/// Load and return the solver type from the specified node.
ChApi ChSolver::Type ReadSolverType(const YAML::Node& a);

/// Load and return the integrator type from the specified node.
ChApi ChTimestepper::Type ReadIntegratorType(const YAML::Node& a);

/// Print YAML node type.
ChApi void PrintNodeType(const YAML::Node& node);

/// @} chrono_io

}  // namespace chrono

#endif
