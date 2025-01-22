#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

using namespace std;
using namespace DGtal;

template<typename T>
void f(T&& parameter);  // purposefully not defined

///////////////////////////////////////////////////////////////////////////////

int main( int /*argc*/, char** /*argv*/ )
{
  Z3i::Domain::ConstIterator it({}, {}, {});
  std::reverse_iterator<Z3i::Domain::ConstIterator> ri(it);
  
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
