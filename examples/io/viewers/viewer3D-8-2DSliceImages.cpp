/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file io/viewers/viewer3D-8-2DSliceImages.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/04/29
 *
 * An example file named viewer3D-8-2DSliceImages .
 *
 * This file is part of the DGtal library.
 */

/**
 * \image html  viewer3D-8.png  "Illustration of the 2D image slice visualization."
 * \example io/viewers/viewer3D-8-2DSliceImages.cpp
 *
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/base/Common.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include "ConfigExamples.h"

#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/io/Color.h"

using namespace std;
using namespace DGtal;

int main( int argc, char** argv )
{
  PolyscopeViewer viewer;

  typedef DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain,  unsigned char > Image3D;
  std::string inputFilename = examplesPath + "samples/lobster.vol";
  Image3D imageVol = VolReader<Image3D>::importVol(inputFilename);

  //! [ExampleViewer3D2DImagesExtractImages]
  // Extracting the 2D images from the 3D one and from a given dimension.
  // First image  the teenth Z slice (dim=2)
  typedef DGtal::ConstImageAdapter<Image3D, DGtal::Z2i::Domain, DGtal::functors::Projector< DGtal::Z3i::Space>,
           Image3D::Value,  DGtal::functors::Identity >  MySliceImageAdapter;

  // Define the functor to recover a 2D domain from the 3D one in the Z direction (2):
  DGtal::functors::Projector<DGtal::Z2i::Space>  transTo2DdomainFunctorZ; transTo2DdomainFunctorZ.initRemoveOneDim(2);
  DGtal::Z2i::Domain domain2DZ(transTo2DdomainFunctorZ(imageVol.domain().lowerBound()),
             transTo2DdomainFunctorZ(imageVol.domain().upperBound()));

  // Define the functor to associate 2D coordinates to the 3D one by giving the direction Z (2) and the slide numnber (10):
  DGtal::functors::Projector<DGtal::Z3i::Space> aSliceFunctorZ(10); aSliceFunctorZ.initAddOneDim(2);

  // We can now obtain the slice image (a ConstImageAdapter):
  const auto identityFunctor = DGtal::functors::Identity();
  MySliceImageAdapter aSliceImageZ(imageVol, domain2DZ, aSliceFunctorZ, identityFunctor );

  // Second image  the fiftieth Y slice (dim=1)
  // Define the functor to recover a 2D domain from the 3D one in the Y direction (1):
  DGtal::functors::Projector<DGtal::Z2i::Space>  transTo2DdomainFunctorY; transTo2DdomainFunctorY.initRemoveOneDim(1);
  DGtal::Z2i::Domain domain2DY(transTo2DdomainFunctorY(imageVol.domain().lowerBound()),
             transTo2DdomainFunctorY(imageVol.domain().upperBound()));

  // Define the functor to associate 2D coordinates to the 3D one by giving the direction Y (1) and the slide numnber (50):
  DGtal::functors::Projector<DGtal::Z3i::Space> aSliceFunctorY(50); aSliceFunctorY.initAddOneDim(1);

  // We can now obtain the slice image (a ConstImageAdapter):
  MySliceImageAdapter aSliceImageY(imageVol, domain2DY, aSliceFunctorY, identityFunctor );
  //! [ExampleViewer3D2DImagesExtractImages]

  //! [ExampleViewer3D2DImagesDisplayImages]
  viewer.draw(aSliceImageZ);
  std::string imageY = viewer.draw(aSliceImageY);
  //! [ExampleViewer3D2DImagesDisplayImages]
  //

 
  viewer.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
