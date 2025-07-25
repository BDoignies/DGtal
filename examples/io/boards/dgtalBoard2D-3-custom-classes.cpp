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
 * @file io/boards/dgtalBoard2D-3-custom-classes.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/11/26
 *
 * An example file named dgtalboard-3-custom-classes.
 *
 * This file is part of the DGtal library.
 */

/**
 * \example io/boards/dgtalBoard2D-3-custom-classes.cpp
 *  
 * This example shows you how to modify the style of each drawable
 * elements. You just have to create an instance of CustomColors,
 * CustomPenColor, CustomFillColor or CustomPen and you attach this
 * style to your drawable element type with an instance of CustomStyle
 * outputed in the Board2D stream.
 *   \image html  dgtalboard-3-custom-classes.png  "visualization of resulting export."
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/Color.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z2i;

///////////////////////////////////////////////////////////////////////////////
int main()
{
  trace.beginBlock ( "Example dgtalBoard2D-3-custom-classes" );

  Point p1( -3, -2 );
  Point p2( 7, 3 );
  Point p3( 0, 0 );
  Domain domain( p1, p2 );

  Color red( 255, 0, 0 );
  Color dred( 192, 0, 0 );
  Color dgreen( 0, 192, 0 );
  Color blue( 0, 0, 255 );
  Color dblue( 0, 0, 192 );
  
  Board2D board;
  board << domain 
  << CustomStyle( p1.className(), new CustomColors( red, dred ) )
  << p1
  << CustomStyle( p2.className(), new CustomFillColor( dgreen ) )
  << p2
  << CustomStyle( p3.className(), 
      new CustomPen( blue, dblue, 6.0, 
               Board2D::Shape::SolidStyle,
               Board2D::Shape::RoundCap,
               Board2D::Shape::RoundJoin ) )
  << p3;
  board.saveSVG("dgtalBoard2D-3-custom-classes.svg");
  board.saveEPS("dgtalBoard2D-3-custom-classes.eps");
  board.saveTikZ("dgtalBoard2D-3-custom-classes.tikz");

#ifdef DGTAL_WITH_CAIRO
  board.saveCairo("dgtalBoard2D-3-custom-classes-cairo.pdf", Board2D::CairoPDF);
  board.saveCairo("dgtalBoard2D-3-custom-classes-cairo.png", Board2D::CairoPNG);
  board.saveCairo("dgtalBoard2D-3-custom-classes-cairo.ps", Board2D::CairoPS);
  board.saveCairo("dgtalBoard2D-3-custom-classes-cairo.svg", Board2D::CairoSVG);
#endif
  
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
