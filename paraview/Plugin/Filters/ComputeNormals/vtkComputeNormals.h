#ifndef vtkMyElevationFilter_h
#define vtkMyElevationFilter_h

#include <vtkAlgorithm.h>
#include <vtkPolyDataAlgorithm.h>

#include "ComputeNormalsModule.h" // for export macro

class COMPUTENORMALS_EXPORT vtkComputeNormals : public vtkPolyDataAlgorithm
{
public:
  static vtkComputeNormals* New();
  vtkTypeMacro(vtkComputeNormals, vtkPolyDataAlgorithm);
  
  int FillInputPortInformation(int, vtkInformation* info) override;
  int RequestData(vtkInformation* vtkNotUsed(request), vtkInformationVector** inputVector, vtkInformationVector* outputVector) override;
protected:
  vtkComputeNormals();
  ~vtkComputeNormals();

private:
  vtkComputeNormals(const vtkComputeNormals&) = delete;
  void operator=(const vtkComputeNormals&) = delete;
};

#endif
