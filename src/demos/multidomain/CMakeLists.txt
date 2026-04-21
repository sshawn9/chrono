#--------------------------------------------------------------

# Add demos that do have no other dependencies
set(MULTIDOMAIN_DEMOS
)

# Add demos that require postprocess
if(CH_ENABLE_MODULE_POSTPROCESS)
    set(MULTIDOMAIN_DEMOS ${MULTIDOMAIN_DEMOS}
        demo_MDOM_MPI_intro_master
        demo_MDOM_MPI_intro_lowlevel
        demo_MDOM_MPI_explicit
        demo_MDOM_MPI_fea
        demo_MDOM_MPI_grid
    )
endif()

# Add demos that require irrlicht 
if(CH_ENABLE_MODULE_IRRLICHT)
    set(MULTIDOMAIN_DEMOS ${MULTIDOMAIN_DEMOS}
        demo_MDOM_OMP_intro_master
        demo_MDOM_OMP_intro_lowlevel
        demo_MDOM_OMP_fea
    )
endif()


#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries
list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_multidomain)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
  list(APPEND LIBS "Chrono_postprocess")
endif()

#--------------------------------------------------------------

message(STATUS "Add demo programs for MULTIDOMAIN module")

foreach(PROGRAM ${MULTIDOMAIN_DEMOS})
  # message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
  if(APPLE)
    set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
  endif()
  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBS})  

  install(TARGETS ${PROGRAM} CONFIGURATIONS Release CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()






