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
#ifdef CHRONO_HAS_YAML
    #include "chrono/input_output/ChUtilsYAML.h"
#endif

#include "chrono_precice/ChPreciceAdapter.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace ch_precice {

ChPreciceAdapter::ChPreciceAdapter() : m_participant(nullptr), m_created(false), m_interfaces_created(false), m_mesh_created(false), m_initialized(false), m_verbose(false) {}

#ifdef CHRONO_HAS_YAML

static std::string ToUpper(std::string in) {
    std::transform(in.begin(), in.end(), in.begin(), ::toupper);
    return in;
}

static ChPreciceAdapter::DataType ReadDataType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "GENERIC")
        return ChPreciceAdapter::DataType::GENERIC;
    if (type == "RIGID_BODY_REF_POINT")
        return ChPreciceAdapter::DataType::RIGID_BODY_REF_POINT;
    if (type == "RIGID_BODY_POINTS")
        return ChPreciceAdapter::DataType::RIGID_BODY_POINTS;
    if (type == "CONTACT_MESH1D_NODES")
        return ChPreciceAdapter::DataType::CONTACT_MESH1D_NODES;
    if (type == "CONTACT_MESH2D_NODES")
        return ChPreciceAdapter::DataType::CONTACT_MESH2D_NODES;
    if (type == "RIGID_BODY_REF_FORCE")
        return ChPreciceAdapter::DataType::RIGID_BODY_REF_FORCE;
    if (type == "RIGID_BODY_FORCES")
        return ChPreciceAdapter::DataType::RIGID_BODY_FORCES;
    if (type == "CONTACT_MESH1D_FORCES")
        return ChPreciceAdapter::DataType::CONTACT_MESH1D_FORCES;
    if (type == "CONTACT_MESH2D_FORCES")
        return ChPreciceAdapter::DataType::CONTACT_MESH2D_FORCES;

    cerr << "Unknown data type: " << a.as<std::string>() << endl;
    throw std::runtime_error("Invalid data type");
}

std::string ChPreciceAdapter::GetParticipantName(const std::string& input_filename) {
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);
    auto config = yaml["precice_adapter_config"];
    ChAssertAlways(config["participant_name"]);
    return config["participant_name"].as<std::string>();
}

void ChPreciceAdapter::RegisterSolver(const std::string& input_filename, int rank, int size) {
    // Read YAML input file and search for preCICE adapter configuration
    YAML::Node yaml = YAML::LoadFile(input_filename);
    ChAssertAlways(yaml["precice_adapter_config"]);

    auto config = yaml["precice_adapter_config"];
    ChAssertAlways(config["participant_name"]);
    ChAssertAlways(config["precice_config_filename"]);
    ChAssertAlways(config["interfaces"]);

    // Read data path handler specification from the YAML configuration and set up the file handler accordingly
    ChYamlFileHandler file_handler;
    file_handler.SetReferenceDirectory(input_filename);
    file_handler.Read(config);

    // Read participant name and preCICE configuration file name from the YAML configuration
    auto participant_name = config["participant_name"].as<std::string>();
    auto precice_config_filename = file_handler.GetFilename(config["precice_config_filename"].as<std::string>());

    // Register the solver with preCICE using the specified participant name, preCICE configuration file name, rank, and size
    RegisterSolver(participant_name, precice_config_filename, rank, size);

    // Read mesh interfaces and data names for writing and reading from the YAML configuration, and initialize the data maps for each mesh/data pair
    auto interfaces = config["interfaces"];
    ChAssertAlways(interfaces.IsSequence());

    for (size_t i = 0; i < interfaces.size(); i++) {
        ChAssertAlways(interfaces[i]["mesh_name"]);
        auto mesh_name = interfaces[i]["mesh_name"].as<std::string>();
        if (interfaces[i]["write_data"]) {
            auto write_data = interfaces[i]["write_data"];
            ChAssertAlways(write_data.IsSequence());
            for (size_t j = 0; j < write_data.size(); j++) {
                auto data = write_data[j];
                ChAssertAlways(data["type"]);
                ChAssertAlways(data["name"]);
                auto data_type = ReadDataType(data["type"]);
                auto data_name = data["name"].as<std::string>();
                auto key = std::make_pair(mesh_name, data_name);
                m_data_write[key].type = data_type;
                m_data_write[key].values = std::vector<double>();  // initialize empty vector for this data
            }
        }
        if (interfaces[i]["read_data"]) {
            auto read_data = interfaces[i]["read_data"];
            ChAssertAlways(read_data.IsSequence());
            for (size_t j = 0; j < read_data.size(); j++) {
                auto data = read_data[j];
                ChAssertAlways(data["type"]);
                ChAssertAlways(data["name"]);
                auto data_type = ReadDataType(data["type"]);
                auto data_name = data["name"].as<std::string>();
                auto key = std::make_pair(mesh_name, data_name);
                m_data_read[key].type = data_type;
                m_data_read[key].values = std::vector<double>();  // initialize empty vector for this data
            }
        }
    }

    m_interfaces_created = (interfaces.size() > 0);

    //// TODO : add checks for validity of the configuration (e.g., check that mesh names and data names are consistent with those declared in the preCICE configuration file, etc.)
}

#endif

void ChPreciceAdapter::RegisterSolver(const std::string& participant_name, const std::string& precice_config_filename, int rank, int size) {
    m_participant_name = participant_name;
    m_precice_config_filename = precice_config_filename;

    m_prefix1 = "[" + m_participant_name + "] ";
    m_prefix2 = std::string(m_prefix1.size(), ' ');

    assert(m_participant == nullptr);
    m_participant = std::make_unique<precice::Participant>(m_participant_name, precice_config_filename, rank, size);
    m_created = true;
}

void ChPreciceAdapter::RegisterMeshInterface(const std::string& mesh_name, const std::vector<std::string>& data_write_names, const std::vector<std::string>& data_read_names) {
    assert(m_created);

    for (const auto& data_name : data_write_names) {
        auto key = std::make_pair(mesh_name, data_name);
        m_data_write[key].values = std::vector<double>();  // initialize empty vector for this data
    }
    for (const auto& data_name : data_read_names) {
        auto key = std::make_pair(mesh_name, data_name);
        m_data_read[key].values = std::vector<double>();  // initialize empty vector for this data
    }

    m_interfaces_created = true;
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::SetMesh(const std::string& mesh_name, const std::vector<ChVector2d>& positions) {
    assert(m_created);
    ChAssertAlways(m_participant->getMeshDimensions(mesh_name) == 2);

    std::vector<double> pos_vec;
    for (const auto& pos : positions) {
        pos_vec.push_back(pos.x());
        pos_vec.push_back(pos.y());
    }

    SetMesh(mesh_name, pos_vec);
}

void ChPreciceAdapter::SetMesh(const std::string& mesh_name, const std::vector<ChVector3d>& positions) {
    assert(m_created);
    ChAssertAlways(m_participant->getMeshDimensions(mesh_name) == 3);

    std::vector<double> pos_vec;
    for (const auto& pos : positions) {
        pos_vec.push_back(pos.x());
        pos_vec.push_back(pos.y());
        pos_vec.push_back(pos.z());
    }

    SetMesh(mesh_name, pos_vec);
}

void ChPreciceAdapter::SetMesh(const std::string& mesh_name, const std::vector<double>& positions) {
    assert(m_created);
    assert(m_interfaces_created);

    int mesh_dim = m_participant->getMeshDimensions(mesh_name);
    ChAssertAlways(mesh_dim == 2 || mesh_dim == 3);
    ChAssertAlways(positions.size() % mesh_dim == 0);

    size_t num_vertices = positions.size() / mesh_dim;
    m_vertex_ids[mesh_name].resize(num_vertices);
    m_participant->setMeshVertices(mesh_name, positions, m_vertex_ids[mesh_name]);
    m_mesh_created = true;

    // Compute size of data vectors for coupling data on this mesh
    auto read_data_names = GetReadDataNamesOnMesh(mesh_name);
    auto write_data_names = GetWriteDataNamesOnMesh(mesh_name);

    for (auto& data_name : read_data_names) {
        // Data dimension is the number of values per vertex for this data, which is determined by preCICE based on the configuration file
        // (e.g., scalar data has dimension 1, vector data has dimension equal to mesh dimension, etc.)
        int data_dimension = m_participant->getDataDimensions(mesh_name, data_name);
        m_data_read[std::make_pair(mesh_name, data_name)].values.resize(num_vertices * data_dimension);
    }

    for (auto& data_name : write_data_names) {
        int data_dimension = m_participant->getDataDimensions(mesh_name, data_name);
        m_data_write[std::make_pair(mesh_name, data_name)].values.resize(num_vertices * data_dimension);
    }

    if (m_verbose) {
        cout << m_prefix1 << "Set mesh " << mesh_name << endl;
        cout << m_prefix2 << "dimension:        " << mesh_dim << endl;
        cout << m_prefix2 << "num. vertices:    " << GetNumVertices(mesh_name) << endl;
        cout << m_prefix2 << "read interfaces:  ";
        for (auto& data_name : read_data_names)
            cout << data_name << " (" << GetDataTypeAsString(mesh_name, data_name) << "," << GetDataDimensions(mesh_name, data_name) << ")  ";
        cout << endl;
        cout << m_prefix2 << "write interfaces: ";
        for (auto& data_name : write_data_names)
            cout << data_name << " (" << GetDataTypeAsString(mesh_name, data_name) << "," << GetDataDimensions(mesh_name, data_name) << ")  ";
        cout << endl;
    }
}

ChPreciceAdapter::DataType ChPreciceAdapter::GetDataType(const std::string& mesh_name, const std::string& data_name) const {
    auto key = std::make_pair(mesh_name, data_name);
    {
        auto it = m_data_read.find(key);
        if (it != m_data_read.end())
            return it->second.type;
    }
    {
        auto it = m_data_write.find(key);
        if (it != m_data_write.end())
            return it->second.type;
    }
    assert(false);
    return DataType::GENERIC;
}

std::string ChPreciceAdapter::GetDataTypeAsString(const std::string& mesh_name, const std::string& data_name) const {
    auto type = GetDataType(mesh_name, data_name);
    switch (type) {
        case DataType::GENERIC:
            return "GENERIC";
        case DataType::RIGID_BODY_REF_POINT:
            return "RIGID_BODY_REF_POINT";
        case DataType::RIGID_BODY_POINTS:
            return "RIGID_BODY_POINTS";
        case DataType::CONTACT_MESH1D_NODES:
            return "CONTACT_MESH1D_NODES";
        case DataType::CONTACT_MESH2D_NODES:
            return "CONTACT_MESH2D_NODES";
        case DataType::RIGID_BODY_REF_FORCE:
            return "RIGID_BODY_REF_FORCE";
        case DataType::RIGID_BODY_FORCES:
            return "RIGID_BODY_FORCES";
        case DataType::CONTACT_MESH1D_FORCES:
            return "CONTACT_MESH1D_FORCES";
        case DataType::CONTACT_MESH2D_FORCES:
            return "CONTACT_MESH2D_FORCES";
    }
    return "UNKNOWN";
}

int ChPreciceAdapter::GetMeshDimensions(const std::string& mesh_name) const {
    assert(m_created);
    return m_participant->getMeshDimensions(mesh_name);
}

int ChPreciceAdapter::GetDataDimensions(const std::string& mesh_name, const std::string& data_name) const {
    assert(m_created);
    return m_participant->getDataDimensions(mesh_name, data_name);
}

double ChPreciceAdapter::GetMaxTimeStepSize() const {
    return m_participant->getMaxTimeStepSize();
}

const std::string& ChPreciceAdapter::GetParticipantName() const {
    assert(m_created);
    return m_participant_name;
}

std::vector<std::string> ChPreciceAdapter::GetMeshNames() const {
    assert(m_created);

    std::vector<std::string> mesh_names;

    for (const auto& [key, value] : m_data_read) {
        auto it = std::find(mesh_names.begin(), mesh_names.end(), key.first);
        if (it == mesh_names.end()) {
            mesh_names.push_back(key.first);
        }
    }

    for (const auto& [key, value] : m_data_write) {
        auto it = std::find(mesh_names.begin(), mesh_names.end(), key.first);
        if (it == mesh_names.end()) {
            mesh_names.push_back(key.first);
        }
    }

    return mesh_names;
}

std::vector<std::string> ChPreciceAdapter::GetReadDataNamesOnMesh(const std::string& mesh_name) const {
    assert(m_created);

    std::vector<std::string> read_names;
    for (const auto& [key, value] : m_data_read) {
        if (key.first == mesh_name) {
            read_names.push_back(key.second);
        }
    }

    return read_names;
}

std::vector<std::string> ChPreciceAdapter::GetWriteDataNamesOnMesh(const std::string& mesh_name) const {
    assert(m_created);

    std::vector<std::string> write_names;
    for (const auto& [key, value] : m_data_write) {
        if (key.first == mesh_name) {
            write_names.push_back(key.second);
        }
    }

    return write_names;
}

size_t ChPreciceAdapter::GetNumVertices(const std::string& mesh_name) {
    assert(m_created);
    return m_vertex_ids[mesh_name].size();
}

// -----------------------------------------------------------------------------

bool ChPreciceAdapter::MustWriteInitialData() {
    assert(m_created);
    return m_participant->requiresInitialData();
}

void ChPreciceAdapter::InitializeCoupling() {
    assert(m_created);
    assert(m_mesh_created);
    assert(!m_initialized);

    m_participant->initialize();

    m_initialized = true;
}

void ChPreciceAdapter::FinalizeCoupling() {
    assert(m_created);
    if (m_initialized)
        m_participant->finalize();
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::InitializeSimulation() {
    assert(m_created);
    assert(m_mesh_created);

    if (m_participant->requiresInitialData()) {
        WriteData();
    }

    InitializeCoupling();
}

void ChPreciceAdapter::SimulationLoop() {
    assert(m_created);
    assert(m_interfaces_created);
    assert(m_mesh_created);
    assert(m_initialized);

    double time = 0;
    while (IsCouplingOngoing()) {
        if (m_participant->requiresWritingCheckpoint())
            WriteCheckpoint(time);

        // Agree on time step size
        double max_time_step = GetMaxTimeStepSize();
        double time_step = std::min(max_time_step, GetSolverTimeStep(max_time_step));

        // Compute time step for solver and advance solver and coupling
        ReadData();
        AdvanceSolver(time, time_step);
        WriteData();
        AdvanceCoupling(time_step);

        if (m_participant->requiresReadingCheckpoint())
            ReadCheckpoint(time);
        else
            time += time_step;
    }
}

bool ChPreciceAdapter::IsCouplingOngoing() {
    assert(m_created);
    return m_participant->isCouplingOngoing();
}

bool ChPreciceAdapter::IsTimeWindowComplete() {
    assert(m_created);
    return m_participant->isTimeWindowComplete();
}

void ChPreciceAdapter::AdvanceCoupling(double time_step) {
    assert(m_created);
    m_participant->advance(time_step);
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::WriteData() {
    if (m_verbose)
        cout << m_prefix1 << "Write data" << endl;
}

void ChPreciceAdapter::ReadData() {
    if (m_verbose)
        cout << m_prefix1 << "Read data" << endl;
}

void ChPreciceAdapter::WriteCheckpoint(double time) {
    if (m_verbose)
        cout << m_prefix1 << "Write checkpoint at time = " << time << endl;
}

void ChPreciceAdapter::ReadCheckpoint(double time) {
    if (m_verbose)
        cout << m_prefix1 << "Read checkpoint for time = " << time << endl;
}

void ChPreciceAdapter::AdvanceSolver(double time, double time_step) {
    if (m_verbose)
        cout << m_prefix1 << "Advance from " << time << " by " << time_step << endl;
}

// -----------------------------------------------------------------------------

void ChPreciceAdapter::SetDataBlock(const std::string& mesh_name, const std::string& data_name, const std::vector<double>& data) {
    auto key = std::make_pair(mesh_name, data_name);
    auto it = m_data_write.find(key);
    assert(it != m_data_write.end());
    assert(it->second.values.size() == data.size());
    it->second.values = data;
}

void ChPreciceAdapter::WriteDataBlock(const std::string& mesh_name, const std::string& data_name) {
    if (m_verbose)
        cout << m_prefix2 << data_name << endl;

    auto key = std::make_pair(mesh_name, data_name);
    std::vector<double>& data_vector = m_data_write[key].values;
    m_participant->writeData(mesh_name, data_name, m_vertex_ids[mesh_name], data_vector);
}

void ChPreciceAdapter::WriteDataBlock(const std::string& mesh_name, const std::string& data_name, const std::vector<double>& data) {
    SetDataBlock(mesh_name, data_name, data);
    WriteDataBlock(mesh_name, data_name);
}

void ChPreciceAdapter::ReadDataBlock(const std::string& mesh_name, const std::string& data_name, double relative_read_time) {
    if (m_verbose)
        cout << m_prefix2 << data_name << endl;

    auto key = std::make_pair(mesh_name, data_name);
    std::vector<double>& data_vector = m_data_read[key].values;
    m_participant->readData(mesh_name, data_name, m_vertex_ids[mesh_name], relative_read_time, data_vector);
}

const std::vector<double>& ChPreciceAdapter::GetDataBlock(const std::string& mesh_name, const std::string& data_name) const {
    auto key = std::make_pair(mesh_name, data_name);
    auto it = m_data_read.find(key);
    assert(it != m_data_read.end());
    return it->second.values;
}

const std::vector<double>& ChPreciceAdapter::ReadDataBlock(const std::string& mesh_name, const std::string& data_name) {
    ReadDataBlock(mesh_name, data_name, 0);
    return GetDataBlock(mesh_name, data_name);
}

// -----------------------------------------------------------------------------

bool ChPreciceAdapter::WriteCheckpointIfRequired(double time) {
    assert(m_created);
    if (!m_participant->requiresWritingCheckpoint())
        return false;
    WriteCheckpoint(time);
    return true;
}

bool ChPreciceAdapter::ReadCheckpointIfRequired(double time) {
    assert(m_created);
    if (!m_participant->requiresReadingCheckpoint())
        return false;
    ReadCheckpoint(time);
    return true;
}

// -----------------------------------------------------------------------------

std::vector<double> ChPreciceAdapter::SetVerticesToData(const std::vector<ChVector2d>& vertices) {
    std::vector<double> data;
    data.reserve(vertices.size() * 2);
    for (const auto& v : vertices) {
        data.push_back(v.x());
        data.push_back(v.y());
    }
    return data;
}

std::vector<double> ChPreciceAdapter::SetVerticesToData(const std::vector<ChVector3d>& vertices) {
    std::vector<double> data;
    data.reserve(vertices.size() * 3);
    for (const auto& v : vertices) {
        data.push_back(v.x());
        data.push_back(v.y());
        data.push_back(v.z());
    }
    return data;
}

}  // end namespace ch_precice
}  // namespace chrono
