#pragma once

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"

using Space3 = DGtal::Z3i::KSpace;
using SH3 = DGtal::Shortcuts<Space3>;

#include <iterator>
#include <memory>
#include <array>

/**
 * @brief A container that holds information necessary to represent both VTK and DGtal voxel grids
 */
struct DGtalVTKImage
{
public:
	static DGtalVTKImage Empty()
	{
		DGtalVTKImage img;
		return img;
	}

	static DGtalVTKImage CreateFromBounds(
		const double imageBounds[6], 
		const double cellBounds[6], 
		const unsigned int nbCells
	) 
	{
		DGtalVTKImage image;
		image.imagePos[0] = imageBounds[0];
		image.imagePos[1] = imageBounds[2];
		image.imagePos[2] = imageBounds[4];

		image.imageSize[0] = imageBounds[1] - imageBounds[0];
		image.imageSize[1] = imageBounds[3] - imageBounds[2];
		image.imageSize[2] = imageBounds[5] - imageBounds[4]; 

		image.cellSize[0] = cellBounds[1] - cellBounds[0];
		image.cellSize[1] = cellBounds[3] - cellBounds[2];
		image.cellSize[2] = cellBounds[5] - cellBounds[4]; 
	
		image.cellCount[0] = std::round(image.imageSize[0] / image.cellSize[0]) + 1;
		image.cellCount[1] = std::round(image.imageSize[1] / image.cellSize[1]) + 1;
		image.cellCount[2] = std::round(image.imageSize[2] / image.cellSize[2]) + 1;

		image.nbCells = nbCells;
		image.domain = std::make_shared<SH3::Domain>(
			SH3::Point(std::floor(imageBounds[0] / image.cellSize[0]), 
				       std::floor(imageBounds[2] / image.cellSize[1]), 
				       std::floor(imageBounds[4] / image.cellSize[2])), 
			SH3::Point(std::ceil (imageBounds[1] / image.cellSize[0]), 
				       std::ceil (imageBounds[3] / image.cellSize[1]), 
				       std::ceil (imageBounds[5] / image.cellSize[2]))
		);
		image.image = std::make_shared<SH3::BinaryImage>(*image.domain);

		return image;
	}

	operator bool() { return image && domain; }

	void SetVoxel(const SH3::Point& point)
	{
		image->setValue(point, true);
	}

	void SetVoxel(const double coords[3])
	{
		const SH3::Point p(std::round(coords[0] / cellSize[0]),
			               std::round(coords[1] / cellSize[1]),
				           std::round(coords[2] / cellSize[2]));
		SetVoxel(p);
	}

	unsigned int GetCellCount() const 
	{
		return nbCells;
	}

	const double* GetCellSize() const
	{
		return cellSize;
	}

	// Iterating facilities
public:
	struct DgtalToVtkIterator
	{
	public:
		using BaseIterator = SH3::Domain::ConstIterator;

		using difference_type = std::ptrdiff_t;
		using value_type = const double*;
		using pointer = const double*;
		using reference = const double*;
		using iterator_category = std::forward_iterator_tag;

		DgtalToVtkIterator(
			const DGtalVTKImage* im, 
			const BaseIterator& it
		) : it(it), image(im)
		{ 
			Advance();
		}

		void Advance()
		{
			while(it != image->domain->end())
			{
				const unsigned int idx = image->image->linearized(*it);
				
				if (image->image->at(idx)) break;
				else ++it;
			} 
		}

		DgtalToVtkIterator& operator++() 
		{
			++it;
			Advance();
			return *this;
		}

		DgtalToVtkIterator operator++(int)
		{
			DgtalToVtkIterator it = *this;
			++(*this);
			return it;
		}

		bool operator==(const DgtalToVtkIterator& other) const 
		{
			return it == other.it;
		}

		bool operator!=(const DgtalToVtkIterator& other) const
		{
			return !(*this == other);
		}

		const double* operator*()
		{
			currentCell[0] = (*it)[0] * image->cellSize[0];
			currentCell[1] = (*it)[1] * image->cellSize[1]; 
			currentCell[2] = (*it)[2] * image->cellSize[2];

			return currentCell;
		}
		
	private:
		const DGtalVTKImage* image;
		BaseIterator it;
		
		double currentCell[3]; 
	};

	DgtalToVtkIterator begin() const 
	{
		return DgtalToVtkIterator(this, domain->begin());
	}

	DgtalToVtkIterator end() const 
	{
		return DgtalToVtkIterator(this, domain->end());
	}
private:
	std::shared_ptr<SH3::Domain> domain = nullptr;
	std::shared_ptr<SH3::BinaryImage> image = nullptr;
	
	unsigned int nbCells = 0;
	unsigned int cellCount[3] = {0, 0, 0};
	
	double imageSize[3] = {0., 0., 0.};
	double imagePos[3]  = {0., 0., 0.};
	double cellSize[3]  = {0., 0., 0.};
};
