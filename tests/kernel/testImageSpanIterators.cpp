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
//LICENSE-END
/**
 * @file test_ImageSpanIterators.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_ImageSpanIterators <p>
 * Aim: simple test of ImageContainerBySTLMap
 */

#include <cstdio>
#include <cmath>
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/images/Image.h"
#include "DGtal/kernel/images/ImageContainerBySTLMap.h"

using namespace DGtal;
using namespace std;





bool testSpanIterators()
{
    typedef SpaceND<int,3> Space3Type;
    typedef Space3Type::Point Point;
    typedef HyperRectDomain<Space3Type> TDomain;
    typedef ImageContainerBySTLVector<Point, double> TContainerV;
    typedef ImageContainerBySTLMap<Point, double> TContainerM;


    const int t[ ] = { 1, 1, 1};
    const int t2[ ] = { 5, 5, 5};
    const int t3[ ] = { 1, 1, 1};
    Point a ( t );
    Point b ( t2 );
    Point c( t3);

    trace.beginBlock("Test of Concepts");
    Image<TDomain,double, TContainerV> myImageV ( a,b );
    Image<TDomain,double, TContainerM> myImageM ( a,b );

    double cpt=0;
    //Image Construction 
    for ( Image<TDomain,double, TContainerV>::Iterator it = myImageV.begin();
            it != myImageV.end();
            ++it)
    {
        myImageV.setValue( it, cpt );
        //myImageM.setValue( (*it), cpt ); //Set value using (point,value) method
        cpt++;
    }

    trace.beginBlock("Builtin iterator on map");
    for ( Image<TDomain,double, TContainerM>::Iterator it = myImageM.begin();
	  it != myImageM.end();
	  ++it)
      trace.info() << myImageM(it)<<" ";
    trace.info()<<endl;
    trace.endBlock();
    
    for ( Image<TDomain,double, TContainerV>::SpanIterator it = myImageV.span_begin(c,1);
            it != myImageV.span_end(c,1);
            ++it)
        trace.info() << myImageV(it)<<" ";
    trace.info() << endl;

    for ( Image<TDomain,double, TContainerM>::SpanIterator it = myImageM.span_begin(c,1);
            it != myImageM.span_end(c,1);
            ++it)
        trace.info() << myImageM(it)<<" ";
    trace.info() << endl;


    trace.endBlock();

    return true;

}



int main()
{

    if ( testSpanIterators())
        return 0;
    else
        return 1;
}

/** @ingroup Tests **/
