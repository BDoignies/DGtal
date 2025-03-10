#include <vtkObjectFactory.h>

#include <vtkInformation.h>
#include <vtkInformationVector.h>

#include "vtkComputeCurvature.h"

#include "../utils/DGtalVTKSurface.h"
#include "../utils/VTKToDGtalVTKImage.h"
#include "../utils/DGtalVTKAbstractContainerToVTK.h"


vtkStandardNewMacro(vtkComputeCurvature);

//----------------------------------------------------------------------------
vtkComputeCurvature::vtkComputeCurvature() 
{
  this->SetNumberOfInputPorts(1);
}

//----------------------------------------------------------------------------
vtkComputeCurvature::~vtkComputeCurvature() = default;

//----------------------------------------------------------------------------
void vtkComputeCurvature::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkComputeCurvature::RequestData(vtkInformation *request,
                                   vtkInformationVector **inputVectors,
                                   vtkInformationVector *outputVector)
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters();
  
  DGtalVTKImage image = GetImageFromVtkInformation(inputVectors[0]->GetInformationObject(0));
  DGtalVTKSurface surface(image); 

  std::vector<double> curvature = SHG3::getIIMeanCurvatures( surface.GetImage(), surface.GetSurfelRange(), params );

  vtkSmartPointer<vtkUnstructuredGrid> grid = GetVtkDataSetFromAbstractContainer(&surface, &curvature);
  outputVector->GetInformationObject(0)->Set(vtkDataObject::DATA_OBJECT(), grid);
  return 1;
}