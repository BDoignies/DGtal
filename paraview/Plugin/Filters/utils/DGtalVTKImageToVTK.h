#pragma once

#include "DGtalVTKImage.h"

#include <vtkUnstructuredGrid.h>
#include <vtkTypeInt64Array.h>
#include <vtkDoubleArray.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkLogger.h>

#if VTK_TYPE_INT64 == VTK_LONG
using VtkInt64 = long int;
#elif VTK_TYPE_INT64 == VTK_LONG_LONG
using VtkInt64 = long long int;
#else
using VtkInt64 = DGtal::int64_t;
#endif

inline void FillCube(const double* center, const double* cellSize, double* location)
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

inline vtkSmartPointer<vtkPoints> PointsFromDGtalVTKImage(const DGtalVTKImage& image)
{
	const unsigned int dim = 3;
	const unsigned int pointsPerCube = 8;
	const unsigned int nbPoints = pointsPerCube * image.GetCellCount(); 

	vtkDoubleArray* vtkPointsArray = vtkDoubleArray::New();
	vtkPointsArray->SetNumberOfComponents(dim); 
	
	double* pointsPtr = ((vtkDoubleArray*)vtkPointsArray)->WritePointer(0, nbPoints * dim);
	
	unsigned int ptCount = 0;
	for (auto it = image.begin(); it != image.end(); ++it)
	{
		if (ptCount == nbPoints)
		{
			vtkVLog(vtkLogger::VERBOSITY_ERROR, "More points in the shape than the nb of cells !");
			std::cout << ptCount << std::endl;
			return nullptr;
		}

		FillCube(*it, image.GetCellSize(), pointsPtr + ptCount * pointsPerCube * dim);
		ptCount++;
	}

	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
	pts->SetData(vtkPointsArray);

	return pts;
}

inline vtkSmartPointer<vtkCellArray> CellsFromDGtalVTKImage(const DGtalVTKImage& image)
{
	const unsigned int dim = 3;
	const unsigned int pointsPerCube = 8;
	const unsigned int nbPoints = pointsPerCube * image.GetCellCount(); 

	// Cell offsets (cube is 8 points -> 0, 8, 16, ...)
	// +1 so that end is past last point
	vtkSmartPointer<vtkTypeInt64Array> vtkOffsetArray = vtkSmartPointer<vtkTypeInt64Array>::New();
	vtkOffsetArray->SetNumberOfComponents(1); 
	
	VtkInt64* offsetPtr = ((vtkTypeInt64Array*)vtkOffsetArray.Get())->WritePointer(0, image.GetCellCount() + 1);
	for (unsigned int i = 0; i < image.GetCellCount() + 1; i++) 
		offsetPtr[i] = i * pointsPerCube;

	// Cell connectivity (points of same cell are next to each other -> 0, 1, 2, ...)
	vtkSmartPointer<vtkTypeInt64Array> vtkConnArray = vtkSmartPointer<vtkTypeInt64Array>::New();
	vtkConnArray->SetNumberOfComponents(1); 
	
	VtkInt64* connPtr = ((vtkTypeInt64Array*)vtkConnArray.Get())->WritePointer(0, nbPoints);
	for (unsigned int i = 0; i < nbPoints; i++) 
		connPtr[i] = i;

	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	cells->SetData(vtkOffsetArray.Get(), vtkConnArray.Get()); // TODO: check for success ? 
	cells->Register(nullptr);
	return cells;
}

inline std::vector<int> CellTypesFromDGtalVTKImage(const DGtalVTKImage& image)
{
	std::vector<int> types(image.GetCellCount(), VTK_VOXEL);
	return types;
}

inline vtkSmartPointer<vtkUnstructuredGrid> GetVtkDataSetFromImage(const DGtalVTKImage& image)
{
	auto grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
	
	vtkSmartPointer<vtkPoints> pts = PointsFromDGtalVTKImage(image);
	grid->SetPoints(pts.Get());

	vtkSmartPointer<vtkCellArray> cells = CellsFromDGtalVTKImage(image);
	std::vector<int> types = CellTypesFromDGtalVTKImage(image);

	grid->SetCells(types.data(), cells.Get());

	return grid;
}