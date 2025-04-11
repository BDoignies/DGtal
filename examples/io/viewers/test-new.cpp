#include <iostream>

#include "DGtal/io/NewDisplay3D.h"
#include "DGtal/io/NewDisplay3DFactory.h"
#include "DGtal/io/viewers/NewPolyscopeViewer3D.h"

using namespace std;
using namespace DGtal;
using namespace Z3i;

int main(int argc, char** argv)
{
    Point p1( 0, 0, 0 );
    Point p2( 5, 5 ,5 );
    Point p3( 2, 3, 4 );
    Domain domain( p1, p2 );

    typedef NewPolyscopeViewer3D<> MyViewer;
    MyViewer viewer;
    viewer << domain;
    viewer << p1 << p2 << p3;

    /*
    viewer<< MyViewer::updateDisplay;
    viewer.show(); */
    return 0;
}
