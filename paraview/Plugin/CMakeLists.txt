paraview_add_plugin(DGtalParaviewPlugin
  REQUIRED_ON_CLIENT
  REQUIRED_ON_SERVER
  VERSION "1.0"
  MODULES 
    ComputeNormals
  MODULE_FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/Filters/ComputeNormals/vtk.module"
  SERVER_MANAGER_XML DGtal.xml)