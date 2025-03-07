set(classes
  vtkComputeNormals)

vtk_module_add_module(ComputeNormals
  CLASSES ${classes})

vtk_module_link(ComputeNormals PRIVATE DGtal::DGtal)