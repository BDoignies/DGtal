#------------------------------------------------------------------------------
# DGtal Configuration file for the install target
#------------------------------------------------------------------------------
include(CMakePackageConfigHelpers)
set(DGTAL_INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/DGtal")

install(TARGETS 
    DGtal 
      # Dependancies also built by the project
      DGtal_STB DGTAL_LibBoard DGTAL_BoostAddons
  EXPORT DGtalTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers 
# Note : this also copies a few .cpp and CMakeLists but simplifies the code here
install(DIRECTORY
  "${PROJECT_SOURCE_DIR}/src/Board"
  "${PROJECT_SOURCE_DIR}/src/DGtal"
  "${PROJECT_SOURCE_DIR}/src/BoostAddons"
  "${PROJECT_SOURCE_DIR}/src/stb"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
# COPY Generated files
install(FILES
  ${PROJECT_BINARY_DIR}/src/DGtal/base/Config.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DGtal/base
)

install(FILES
  ${PROJECT_BINARY_DIR}/src/DGtal/topology/tables/NeighborhoodTables.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DGtal/topology/tables/
)


install(EXPORT DGtalTargets
  FILE DGtalTargets.cmake
  NAMESPACE DGtal::
  DESTINATION ${DGTAL_INSTALL_DESTINATION}
)


#------------------------------------------------------------------------------
# DGtalConfig.cmake variables
#------------------------------------------------------------------------------
set(_dependencies_list
  Boost ZLIB
  LibBoard
  GMP  ITK Cairo HDF5 QGLVIEWER Qt5 OpenMP Eigen3::Eigen CGAL FFTW3 OpenMP::OpenMPCXX
  )
set(_find_cmake_files
  "${PROJECT_SOURCE_DIR}/cmake/deps/eigen.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/deps/FindCairo.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/deps/FindFFTW3.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/deps/FindQGLVIEWER.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/deps/FindGMP.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/deps/eigen.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/deps/libigl.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/deps/openmp.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/CPM.cmake"
)
foreach(dep ${_dependencies_list})
  if(DGTAL_CONFIG_HINTS)
    if("${${dep}_DIR}" STREQUAL "" OR
       "${${dep}_DIR}" STREQUAL "${dep}_DIR-NOTFOUND")
      set(${dep}_HINTS "# NO_HINTS (no ${dep}_DIR or ${dep}_DIR-NOTFOUND)")
    else()
      set(${dep}_HINTS "HINTS \"${${dep}_DIR}\"")
    endif()
  else()
    set(${dep}_HINTS "# NO_HINTS (disabled with DGTAL_CONFIG_HINTS=OFF)")
  endif()
endforeach()

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/DGtalConfigVersion.cmake"
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/DGtalConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/DGtalConfig.cmake"
  INSTALL_DESTINATION ${DGTAL_INSTALL_DESTINATION}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/DGtalConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/DGtalConfigVersion.cmake"
           ${_find_cmake_files}
        DESTINATION ${DGTAL_INSTALL_DESTINATION}
)