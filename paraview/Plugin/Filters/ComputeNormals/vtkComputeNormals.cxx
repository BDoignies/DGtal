#include <vtkObjectFactory.h>

#include <vtkInformation.h>
#include <vtkInformationVector.h>

#include "vtkComputeNormals.h"

#include "../utils/DGtalVTKSurface.h"
#include "../utils/VTKToDGtalVTKImage.h"
#include "../utils/DGtalVTKAbstractContainerToVTK.h"


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
  DGtalVTKImage image = GetImageFromVtkInformation(inputVectors[0]->GetInformationObject(0));
  DGtalVTKSurface surface(image); 

  vtkSmartPointer<vtkUnstructuredGrid> grid = GetVtkDataSetFromAbstractContainer(&image);
  outputVector->GetInformationObject(0)->Set(vtkDataObject::DATA_OBJECT(), grid);
  return 1;
}