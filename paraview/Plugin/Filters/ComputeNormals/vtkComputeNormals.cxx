#include <vtkObjectFactory.h>
#include <vtkInformation.h>

#include "vtkComputeNormals.h"

vtkStandardNewMacro(vtkComputeNormals);

//----------------------------------------------------------------------------
vtkComputeNormals::vtkComputeNormals() = default;

//----------------------------------------------------------------------------
vtkComputeNormals::~vtkComputeNormals() = default;

//----------------------------------------------------------------------------
int vtkComputeNormals::FillInputPortInformation(int vtkNotUsed(port), vtkInformation* info) 
{
  // now add our info
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkUnstructuredGrid");
  return 1;
}


int vtkComputeNormals::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
	return 1;
}