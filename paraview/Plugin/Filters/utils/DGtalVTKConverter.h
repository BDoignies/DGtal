#pragma once

#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkCellIterator.h>
#include <vtkDataObject.h>
#include <vtkDataSet.h>
#include <vtkLogger.h>

#include <vtkTypeInt64Array.h>
#include <vtkDoubleArray.h>

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"

using Space3 = DGtal::Z3i::KSpace;
using SH3 = DGtal::Shortcuts<Space3>;

#if VTK_TYPE_INT64 == VTK_LONG
using VtkInt64 = long int;
#elif VTK_TYPE_INT64 == VTK_LONG_LONG
using VtkInt64 = long long int;
#else
using VtkInt64 = DGtal::int64_t;
#endif

// This is a light-to-copy class
struct DGtalImageFromVTK
{
public:

	static DGtalImageFromVTK Empty()
	{
		DGtalImageFromVTK img;
		return img;
	}

	operator bool() 
	{
		return image && domain;
	}

	void Init(const double imageBounds[6], const double cellBounds[6], unsigned int nbCells)
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

		domain = new SH3::Domain(
			SH3::Point(std::floor(imageBounds[0] / cellSize[0]), 
				       std::floor(imageBounds[2] / cellSize[1]), 
				       std::floor(imageBounds[4] / cellSize[2])), 
			SH3::Point(std::ceil (imageBounds[1] / cellSize[0]), 
				       std::ceil (imageBounds[3] / cellSize[1]), 
				       std::ceil (imageBounds[5] / cellSize[2]))
		);
		image  = new SH3::BinaryImage(*domain);

		this->nbCells = nbCells;
	}

	SH3::Domain* domain = nullptr;
	SH3::BinaryImage* image = nullptr;
	
	unsigned int nbCells = 0;
	unsigned int cellCount[3] = {0, 0, 0};
	
	double imageSize[3] = {0., 0., 0.};
	double imagePos[3]  = {0., 0., 0.};
	double cellSize[3]  = {0., 0., 0.};
};

inline DGtalImageFromVTK GetImageFromVtkDataSet(vtkDataSet* dataset)
{
	DGtalImageFromVTK dgtalImage;
	
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
	dataset->GetCell(0)->GetBounds(cellBounds); // Assume all cells are equivalents

	dgtalImage.Init(imageBounds, cellBounds, dataset->GetNumberOfCells());

	// Use generic model of cell to be compatible with most models
	int subId = 0; /* unused: assume there are no composite cells */
	double interpolationWeights[8] = { 0., 0., 0., 0., 0., 0., 0., 0.}; /* unused */
	const double parametricCoords[3] = {0.5, 0.5, 0.5}; /* center of cell */
	
	// Output coordinates and cell
	double    rawCoordinates[3] = {0., 0., 0.}; 

	auto it = dataset->NewCellIterator();
	vtkGenericCell* cell = vtkGenericCell::New();

	for (it->InitTraversal(); !it->IsDoneWithTraversal(); it->GoToNextCell())
	{
		if (it->GetCellType() != VTK_VOXEL) continue;
		
		it->GetCell(cell);
		cell->EvaluateLocation(subId, parametricCoords, rawCoordinates, interpolationWeights);

		const SH3::Point p(std::round(rawCoordinates[0] / dgtalImage.cellSize[0]),
			               std::round(rawCoordinates[1] / dgtalImage.cellSize[1]),
				           std::round(rawCoordinates[2] / dgtalImage.cellSize[2]));
		dgtalImage.image->setValue(p, true);
	}
	
	return dgtalImage;
}

void FillCube(const double center[3], const double cellSize[3], double* location)
{
	static constexpr double factors[24] = {
		-0.5, -0.5, -0.5, 
		 0.5, -0.5, -0.5, 
		-0.5,  0.5, -0.5, 
		 0.5,  0.5, -0.5, 
		-0.5, -0.5,  0.5, 
		 0.5, -0.5,  0.5, 
		-0.5,  0.5,  0.5, 
		 0.5,  0.5,  0.5
	};

	for (unsigned int i = 0; i < 8; i++)
	{
		for (unsigned int k = 0; k < 3; k++)
		{
			location[i * 3 + k] = center[k] + factors[i * 3 + k] * cellSize[k];
		}
	}
}

inline vtkSmartPointer<vtkUnstructuredGrid> GetVtkDataSetFromImage(const DGtalImageFromVTK& image)
{
	// Method for converting is taken from vtkUnstrucuredGridReader

	const unsigned int dim = 3;
	const unsigned int pointsPerCube = 8;
	const unsigned int nbPoints = pointsPerCube * image.nbCells; 
	
	auto grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
		// Point array
		auto vtkPointsArray = vtkDoubleArray::New();
		vtkPointsArray->SetNumberOfComponents(dim); 
		double* pointsPtr = ((vtkDoubleArray*)vtkPointsArray)->WritePointer(0, nbPoints * dim);
		
		unsigned int ptCount = 0;
		for (auto it = image.domain->begin(); it != image.domain->end(); ++it)
		{
			unsigned int linearized = image.image->linearized(*it);
			if (image.image->at(linearized))
			{
				if (ptCount == nbPoints)
				{
					vtkVLog(vtkLogger::VERBOSITY_ERROR, "More points in the shape than the nb of cells !");
					return nullptr;
				}

				const double cellCenter[3] = {
					(*it)[0] * image.cellSize[0], 
					(*it)[1] * image.cellSize[1], 
					(*it)[2] * image.cellSize[2] 
				};
				FillCube(cellCenter, image.cellSize, pointsPtr + ptCount * pointsPerCube * dim);
				ptCount ++;
			}
		}

		// Set points 
		{
			vtkPoints* pts = vtkPoints::New();
			pts->SetData(vtkPointsArray);
			vtkPointsArray->Delete();
			grid->SetPoints(pts);
			pts->Delete();
		}

		// Cell offsets (cube is 8 points -> 0, 8, 16, ...)
		// +1 so that end is past last point
		auto vtkOffsetArray = vtkTypeInt64Array::New();
		vtkOffsetArray->SetNumberOfComponents(1); 
		VtkInt64* offsetPtr = ((vtkTypeInt64Array*)vtkOffsetArray)->WritePointer(0, image.nbCells + 1);
		for (unsigned int i = 0; i < image.nbCells + 1; i++) offsetPtr[i] = i * pointsPerCube;

		// Cell connectivity (points of same cell are next to each other -> 0, 1, 2, ...)
		auto vtkConnArray = vtkTypeInt64Array::New();
		vtkConnArray->SetNumberOfComponents(1); 
		VtkInt64* connPtr = ((vtkTypeInt64Array*)vtkConnArray)->WritePointer(0, nbPoints);
		for (unsigned int i = 0; i < nbPoints; i++) connPtr[i] = i;

		auto cells = vtkSmartPointer<vtkCellArray>::New();
		cells->SetData(vtkOffsetArray, vtkConnArray); // TODO: check for success ? 

		// Cell type (here always VOXEL_TYPE)
		int* cellsType = new int[image.nbCells];
		for (unsigned int i = 0; i < image.nbCells; i++) cellsType[i] = VTK_VOXEL;

		grid->SetCells(cellsType, cells);

	if (false)
	{
		delete[] cellsType;
		if (cells) cells->Delete();
	}
	return grid;
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

	if (input->IsA("vtkDataSet")) // Lowest supported model
		return GetImageFromVtkDataSet(vtkDataSet::GetData(info));

	vtkVLog(vtkLogger::VERBOSITY_ERROR, "Unknown type to convert from \"" + std::string(input->GetClassName()) + "\"");
	return DGtalImageFromVTK::Empty();
}