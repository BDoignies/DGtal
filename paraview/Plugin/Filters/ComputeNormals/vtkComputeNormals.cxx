#include <vtkObjectFactory.h>

#include <vtkInformation.h>
#include <vtkInformationVector.h>

#include "vtkComputeNormals.h"

#include "../utils/DGtalVTKConverter.h"

vtkStandardNewMacro(vtkComputeNormals);

//----------------------------------------------------------------------------
vtkComputeNormals::vtkComputeNormals() 
{
  this->SetNumberOfInputPorts(1);
}

//----------------------------------------------------------------------------
vtkComputeNormals::~vtkComputeNormals() = default;

//----------------------------------------------------------------------------
void vtkComputeNormals::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkComputeNormals::RequestData(vtkInformation *request,
                                   vtkInformationVector **inputVectors,
                                   vtkInformationVector *outputVector)
{
  DGtalImageFromVTK image = GetImageFromVtkInformation(inputVectors[0]->GetInformationObject(0));
  vtkSmartPointer<vtkUnstructuredGrid> grid = GetVtkDataSetFromImage(image);
  outputVector->GetInformationObject(0)->Set(vtkDataObject::DATA_OBJECT(), grid);
  return 1;
}