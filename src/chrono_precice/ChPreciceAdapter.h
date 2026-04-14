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

#ifndef CH_PRECICE_ADAPTER_H
#define CH_PRECICE_ADAPTER_H

#include <string>
#include <vector>
#include <map>

#include "chrono_precice/ChApiPrecice.h"

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"
#include "chrono/core/ChVector2.h"
#include "chrono/core/ChVector3.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <precice/precice.hpp>

namespace chrono {
namespace ch_precice {

/// @addtogroup precice_module
/// @{

/// Base class for all preCICE adapters.
class ChApiPrecice ChPreciceAdapter {
  public:
    /// Chrono coupling data types.
    enum class DataType {
        GENERIC,                 ///<
        RIGID_BODY_REF_POINT,    ///<
        RIGID_BODY_MESH_POINTS,  ///<
        CONTACT_MESH1D_NODES,    ///<
        CONTACT_MESH2D_NODES,    ///<
        RIGID_BODY_REF_FORCE,    ///<
        RIGID_BODY_MESH_FORCES,  ///<
        CONTACT_MESH1D_FORCES,   ///<
        CONTACT_MESH2D_FORCES    ///<
    };

    ChPreciceAdapter();
    virtual ~ChPreciceAdapter() {}

    ChPreciceAdapter(const ChPreciceAdapter&) = delete;
    void operator=(const ChPreciceAdapter&) = delete;

    void SetVerbose(bool verbose) { m_verbose = verbose; }

    // ---- preCICE participant specification

#ifdef CHRONO_HAS_YAML
    /// Get the participant name from the specified YAML input file.
    static std::string GetParticipantName(const std::string& input_filename);

    /// Register the Chrono solver and its mesh interfaces for use with preCICE, using the specified input file.
    /// The YAML input file must have a group named "precice_adapter_config" with the following parameters:
    /// - participant_name:        name of the participant or solver
    /// - precice_config_filename: path and file name to preCICE configuration file
    /// - interfaces:              list of interfaces, each with the following parameters:
    ///    - mesh_name:  name of the coupling mesh
    ///    - read_data:  data to read from preCICE on this mesh
    ///    - write_data: data to write to preCICE on this mesh
    void RegisterSolver(const std::string& input_filename, int rank, int size);
#endif

    /// Register the Chrono solver for use with preCICE, using the specified configuration file and MPI rank/size.
    void RegisterSolver(const std::string& participant_name, const std::string& precice_config_filename, int rank, int size);

    /// Register the coupling mesh and data names for use with preCICE, using the specified mesh name and lists of data names for writing and reading.
    void RegisterMeshInterface(const std::string& mesh_name, const std::vector<std::string>& data_write_names, const std::vector<std::string>& data_read_names);

    // ---- Mesh specification

    /// Add 2D vertices for solver coupling to the mesh with specified name.
    /// With the mesh size, the data maps inside the adapter initialize the relevant data vector to size of mesh_size*data_dimension.
    void SetMesh(const std::string& mesh_name, const std::vector<ChVector2d>& positions);

    /// Add 3D vertices for solver coupling to the mesh with specified name.
    /// With the mesh size, the data maps inside the adapter initialize the relevant data vector to size of mesh_size*data_dimension.
    void SetMesh(const std::string& mesh_name, const std::vector<ChVector3d>& positions);

    /// Add vertices for solver coupling to the mesh with specified name.
    /// This method can be used for both 2D and 3D meshes by providing the appropriate positions vector.
    /// The positions vector is expected to be in the format:
    /// - (x0, y0, x1, y1, ...) for 2D meshes, and
    /// - (x0, y0, z0, x1, y1, z1, ...) for 3D meshes.
    /// With the mesh size, the data maps inside the adapter initialize the relevant data vector to size of mesh_size*data_dimension.
    /// - scalar data (declared with <data:scalar ...>) has data_dimension = 1; e.g., temperature, pressure, etc.
    /// - vector data (declared with <data:vector ...>) has data_dimension equal to the mesh dimension; e.g., velocity, displacement, etc.
    void SetMesh(const std::string& mesh_name, const std::vector<double>& positions);

    // ---- Accessor functions

    /// Get the number of spatial dimensions for the mesh with the specified name.
    int GetMeshDimensions(const std::string& mesh_name) const;

    /// Get the number of vertices for the mesh with the specified name.
    size_t GetNumVertices(const std::string& mesh_name);

    /// Get the type for the data with the specified name on the mesh with the specified name.
    DataType GetDataType(const std::string& mesh_name, const std::string& data_name) const;

    /// Get the type name for the data with the specified name on the mesh with the specified name.
    std::string GetDataTypeAsString(const std::string& mesh_name, const std::string& data_name) const;

    /// Get the data dimensions for the data with the specified name on the mesh with the specified name.
    int GetDataDimensions(const std::string& mesh_name, const std::string& data_name) const;

    /// Get the maximum time step size from preCICE.
    double GetMaxTimeStepSize() const;

    /// Get the participant name from config
    const std::string& GetParticipantName() const;

    /// Get the coupled mesh names on this participant.
    std::vector<std::string> GetMeshNames() const;

    /// Get the data names for reading on the mesh with specified name.
    std::vector<std::string> GetReadDataNamesOnMesh(const std::string& mesh_name) const;

    /// Get the data names for writing on the mesh with specified name.
    std::vector<std::string> GetWriteDataNamesOnMesh(const std::string& mesh_name) const;

    // ---- Checkpointing

    /// Write the solver state to a checkpoint if required by preCICE.
    /// If requested by preCISE, this function invokes the solver-specific checkpoint writing function.
    /// The return value indicates whether a checkpoint was written.
    bool WriteCheckpointIfRequired(double time);

    /// Read the solver state from a checkpoint if required by preCICE.
    /// If requested by preCISE, this function invokes the solver-specific checkpoint reading function.
    /// The return value indicates whether a checkpoint was read.
    bool ReadCheckpointIfRequired(double time);

    // ---- Data exchange

    // Set the (write) data vector for the specified mesh and data names.
    void SetDataBlock(const std::string& mesh_name, const std::string& data_name, const std::vector<double>& data);

    /// Write (send) a block of data to preCICE.
    void WriteDataBlock(const std::string& mesh_name, const std::string& data_name);

    /// Set and write (send) a block of data to preCICE.
    /// This is a convenience function that combines SetDataBlock and WriteDataBlock into a single call.
    void WriteDataBlock(const std::string& mesh_name, const std::string& data_name, const std::vector<double>& data);

    /// Read (receive) a block of data from preCICE.
    void ReadDataBlock(const std::string& mesh_name, const std::string& data_name, double relative_read_time);

    /// Get the (read) data vector for the specified mesh and data names.
    const std::vector<double>& GetDataBlock(const std::string& mesh_name, const std::string& data_name) const;

    /// Read (receive) a block of data from preCICE and return the data vector.
    /// This is a convenience function that combines ReadDataBlock and GetDataBlock into a single call.
    /// It is assumed that the data is always read at the beginning of the time step (relative_read_time = 0).
    const std::vector<double>& ReadDataBlock(const std::string& mesh_name, const std::string& data_name);

    // ---- Simulation control

    /// Check if the participant is required to provide initial data.
    /// If true, the participant needs to write initial data to defined vertices prior to calling Initialize().
    bool MustWriteInitialData();

    /// Check if coupling is ongoing.
    bool IsCouplingOngoing();

    /// Check if the time window has completed.
    bool IsTimeWindowComplete();

    /// Wrapper function for initializing the coupled simulation for this participant.
    /// The participant initializes the output data (if needed), after which the coupling is initialized.
    void InitializeSimulation();

    /// Wrapper function for performing the simulation loop.
    /// While coupling is on-going, at each iteration, the participant:
    /// - writes a checkpoint if requested
    /// - agrees on the simulation time step size
    /// - reads data
    /// - advances the underlying solver dynamics
    /// - writes data
    /// - advances the preCICE coupling
    /// - reads a checkpoint if requested, otherwise advances time
    void SimulationLoop();

    /// Wrapper function for finalizing the coupled simulation for this participant.
    void FinalizeSimulation();

    // ---- Utility functions

    /// Convert a vector of ChVector2d to a vector of doubles in the format (x0, y0, x1, y1, ...).
    static std::vector<double> SetVerticesToData(const std::vector<ChVector2d>& vertices);

    /// Convert a vector of ChVector3d to a vector of doubles in the format (x0, y0, z0, x1, y1, z1, ...).
    static std::vector<double> SetVerticesToData(const std::vector<ChVector3d>& vertices);

  protected:
    // ---- Solver-specific functions to be implemented by derived classes

    /// Let the derived class perform any necessary operations during the simulation initialization.
    /// This optional function is called before the solver writes initial data (if requested) and before the preCICE coupling is initialized.
    virtual void InitializeSolver();

    /// Let the derived class implement the actual checkpoint writing if required by preCICE.
    virtual void WriteCheckpoint(double time);

    /// Let the derived class implement the actual checkpoint reading if required by preCICE.
    /// The solver from a derived class must restore its state at the values in the last saved checkpoint and, if needed, reset its internal time to the provided value.
    virtual void ReadCheckpoint(double time);

    /// Read data from other solvers.
    /// A derived class must:
    /// - call ReadDataBlock() for all its interfaces,
    /// - obtain the read data with calls to GetDataBlock(), and
    /// - perform any necessary processing on the received data.
    /// A convenience version of ReadDataBlock() that combines the first 2 steps for a given read interface (mesh/data name pair) is available.
    virtual void ReadData();

    /// Let the derived class implement the actual computation of the solver time step based on the maximum time step provided by preCICE.
    /// The default implementation simply returns the maximum time step provided by preCICE, but derived classes can override this to implement custom time-stepping logic.
    virtual double GetSolverTimeStep(double max_time_step) const { return max_time_step; }

    /// Let the derived class implement the actual solver time-stepping by the given time step.
    virtual void AdvanceSolver(double time, double time_step);

    /// Write data for other solvers.
    /// A derived class must:
    /// - prepare the data to be sent,
    /// - set the data for all its interfaces with calls to SetDataBlock(), and
    /// - finally call WriteDataBlock().
    /// A convenience version of WriteDataBlock() that combines the last 2 steps for a given write interface (mesh/data name pair) is available.
    virtual void WriteData();

    /// Let the derived class perform any necessary operations during simulation shutdown.
    /// This function is called before the preCICE coupling is finalized.
    virtual void FinalizeSolver();

    // ---- Member variables

    /// Definition of mesh data.
    struct Data {
        DataType type;
        std::vector<double> values;
    };

    /// Definition of data structure to hold coupling data for a particular mesh/data name pair.
    /// The key is the pair of mesh name and data name, and the value is the data type and a vector of data values for that mesh/data pair.
    /// The dimension of the data vector is determined by the number of vertices on the coupling mesh and the data dimension for that mesh/data pair.
    using MeshData = std::map<std::pair<std::string, std::string>, Data>;

    /// Definition of data structure to hold vertex IDs for a particular mesh.
    /// The key is the mesh name and the value is the vector of preCICE vertex IDs.
    using VertexIDs = std::map<std::string, std::vector<int>>;

    std::unique_ptr<precice::Participant> m_participant;  ///< preCICE instance
    std::string m_precice_config_filename;                ///< preCICE configuration file name and path
    std::string m_participant_name;                       ///< name of the participant (solver)

    MeshData m_data_read;   ///< data read from preCICE (for a particular interface; i.e., mesh/data name pair)
    MeshData m_data_write;  ///< data to write to preCICE (for a particular interface; i.e., mesh/data name pair)

    ////std::vector<int> m_vertex_ids;  ///< preCICE identifiers of the vertices of the coupling mesh
    VertexIDs m_vertex_ids;  ///< preCICE identifiers of the mesh vertices (for a particular mesh name)

    bool m_created;             ///< true if the preCICE participant was created
    bool m_interfaces_created;  ///< true if the data interfaces were created
    bool m_mesh_created;        ///< true if the coupling mesh was created
    bool m_initialized;         ///< true if preCICE participant was initialized

    bool m_verbose;         ///< verbose terminal output
    std::string m_prefix1;  ///< prefix for terminal messages (first line)
    std::string m_prefix2;  ///< prefix for terminal messages (subsequent lines)
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
