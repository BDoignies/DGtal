#pragma once

#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkCellIterator.h>
#include <vtkDataObject.h>
#include <vtkDataSet.h>
#include <vtkLogger.h>

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"

using Space3 = DGtal::Z3i::KSpace;
using SH3 = DGtal::Shortcuts<Space3>;

#define VERIFY_CELL_SIZE

struct DGtalImageFromVTK
{
public:

	static DGtalImageFromVTK Empty()
	{
		DGtalImageFromVTK img;
		img.image = nullptr;
		return img;
	}

	void SetBounds(const double imageBounds[6], const double cellBounds[6])
	{
		imagePos[0] = imageBounds[0];
		imagePos[1] = imageBounds[2];
		imagePos[2] = imageBounds[4];

		imageSize[0] = imageBounds[1] - imageBounds[0];
		imageSize[1] = imageBounds[3] - imageBounds[2];
		imageSize[2] = imageBounds[5] - imageBounds[4]; 

		cellSize[0] = cellBounds[1] - cellBounds[0];
		cellSize[1] = cellBounds[3] - cellBounds[2];
		cellSize[2] = cellBounds[5] - cellBounds[4]; 
	
		cellCount[0] = std::round(imageSize[0] / cellSize[0]) + 1;
		cellCount[1] = std::round(imageSize[1] / cellSize[1]) + 1;
		cellCount[2] = std::round(imageSize[2] / cellSize[2]) + 1;
	}

	SH3::BinaryImage* image;
	
	unsigned int cellCount[3];
	
	double imageSize[3];
	double imagePos[3];
	double cellSize[3];
};

inline DGtalImageFromVTK GetImageFromVtkDataSet(vtkDataSet* dataset)
{
	DGtalImageFromVTK dgtalImage = DGtalImageFromVTK::Empty();

	vtkVLog(vtkLogger::VERBOSITY_INFO, "Getting image from vtkUnstructuredGrid");
	if (!dataset)
	{
		vtkVLog(vtkLogger::VERBOSITY_ERROR, "No grid to convert from !");
		return DGtalImageFromVTK::Empty();
	}

	vtkVLog(vtkLogger::VERBOSITY_INFO, "Number of cells: " + std::to_string(dataset->GetNumberOfCells()));

	// Get grid information
	double imageBounds[6], cellBounds[6];
	dataset->GetBounds(imageBounds);
	dataset->GetCell(0)->GetBounds(cellBounds);
	dgtalImage.SetBounds(imageBounds, cellBounds);

	// Use generic model of cell to be compatible with most models
	int subId = 0; 
	double interpolationWeights[8] = { 0., 0., 0., 0., 0., 0., 0., 0.};
	const double parametricCoords[3] = {0.5, 0.5, 0.5};
	
	// Output coordinates and cell
	double    rawCoordinates[3] = {0., 0., 0.};
	unsigned int coordinates[3] = {0 , 0 , 0 }; 

	auto it = dataset->NewCellIterator();
	vtkGenericCell* cell = vtkGenericCell::New();

	for (it->InitTraversal(); !it->IsDoneWithTraversal(); it->GoToNextCell())
	{
		if (it->GetCellType() != VTK_VOXEL) continue;
		
		// Note : can not cast to vtkVoxel, cell type might be different...
		it->GetCell(cell);
		cell->EvaluateLocation(subId, parametricCoords, rawCoordinates, interpolationWeights);

		coordinates[0] = std::round((rawCoordinates[0] - dgtalImage.imagePos[0]) / dgtalImage.cellSize[0]);
		coordinates[1] = std::round((rawCoordinates[1] - dgtalImage.imagePos[1]) / dgtalImage.cellSize[1]);
		coordinates[2] = std::round((rawCoordinates[2] - dgtalImage.imagePos[2]) / dgtalImage.cellSize[2]);

		// Todo add point to DGtal Image container
	}
	

	return dgtalImage;
}

inline DGtalImageFromVTK GetImageFromVtkInformation(vtkInformation* info)
{
	// No information
	if (info == nullptr) 
	{
		vtkVLog(vtkLogger::VERBOSITY_ERROR, "No information to convert from !");
		return DGtalImageFromVTK::Empty();
	}
	
	// Can not deduce underlying data type
    auto input = vtkDataObject::GetData(info);
	if (!input)
	{
		vtkVLog(vtkLogger::VERBOSITY_ERROR, "Can not get data from input");
		return DGtalImageFromVTK::Empty();
	}

	if (input->IsA("vtkDataSet"))
		return GetImageFromVtkDataSet(vtkDataSet::GetData(info));

	vtkVLog(vtkLogger::VERBOSITY_ERROR, "Unknown type to convert from \"" + std::string(input->GetClassName()) + "\"");
	return DGtalImageFromVTK::Empty();
}