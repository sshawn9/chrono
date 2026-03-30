set(TESTS
    btest_SOA_pendulums
)

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_VSG)
  list(APPEND LIBS "Chrono_vsg")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS "Chrono_pardisomkl")
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for SOA module")
build_btests("${TESTS}" "${LIBS}")
