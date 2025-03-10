#ifndef vtkMyElevationFilter_h
#define vtkMyElevationFilter_h

#include <vtkAlgorithm.h>
#include <vtkUnstructuredGridAlgorithm.h>

#include "ComputeCurvatureModule.h" // for export macro

class COMPUTECURVATURE_EXPORT vtkComputeCurvature : public vtkUnstructuredGridAlgorithm
{
public:
  static vtkComputeCurvature* New();
  vtkTypeMacro(vtkComputeCurvature, vtkUnstructuredGridAlgorithm);
  
  int RequestData(vtkInformation *request,
                  vtkInformationVector **inputVectors,
                  vtkInformationVector *outputVector);


  void PrintSelf(ostream& os, vtkIndent indent) override;
protected:
  vtkComputeCurvature();
  ~vtkComputeCurvature();

private:
  vtkComputeCurvature(const vtkComputeCurvature&) = delete;
  void operator=(const vtkComputeCurvature&) = delete;
};

#endif
