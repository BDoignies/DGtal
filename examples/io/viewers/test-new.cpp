#include <iostream>

#include "DGtal/io/NewDisplay3D.h"
#include "DGtal/io/NewDisplay3DFactory.h"
#include "DGtal/io/viewers/NewPolyscopeViewer3D.h"

using namespace std;
using namespace DGtal;
using namespace Z3i;

#include "DGtal/shapes/Shapes.h"
typedef NewPolyscopeViewer3D<> MyViewer;

int main(int argc, char** argv)
{
    MyViewer viewer;

    Point p1( 0, 0, 0 );
    Point p2( 10, 10 , 10 );
    Domain domain( p1, p2 );
    viewer << domain;

    DigitalSet shape_set( domain );
    Shapes<Domain>::addNorm1Ball( shape_set, Point( 5, 5, 5 ), 2 );
    Shapes<Domain>::addNorm2Ball( shape_set, Point( 3, 3, 3 ), 2 );

    shape_set.erase(Point(3,3,3));
    shape_set.erase(Point(6,6,6));

    viewer << shape_set;

    /*
    viewer<< MyViewer::updateDisplay; */
    viewer.show(); 
    return 0;
}
