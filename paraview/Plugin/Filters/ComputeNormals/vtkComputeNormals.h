#ifndef vtkMyElevationFilter_h
#define vtkMyElevationFilter_h

#include <vtkAlgorithm.h>
#include <vtkUnstructuredGridAlgorithm.h>

#include "ComputeNormalsModule.h" // for export macro

class COMPUTENORMALS_EXPORT vtkComputeNormals : public vtkUnstructuredGridAlgorithm
{
public:
  static vtkComputeNormals* New();
  vtkTypeMacro(vtkComputeNormals, vtkUnstructuredGridAlgorithm);
  
  int RequestData(vtkInformation *request,
                  vtkInformationVector **inputVectors,
                  vtkInformationVector *outputVector);


  void PrintSelf(ostream& os, vtkIndent indent) override;
protected:
  vtkComputeNormals();
  ~vtkComputeNormals();

private:
  vtkComputeNormals(const vtkComputeNormals&) = delete;
  void operator=(const vtkComputeNormals&) = delete;
};

#endif
