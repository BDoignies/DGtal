#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

using namespace std;
using namespace DGtal;

template<typename T>
void f(T&& parameter);  // purposefully not defined

///////////////////////////////////////////////////////////////////////////////

#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

template<typename T>
void f(T&& v);

int main( int /*argc*/, char** /*argv*/ )
{
  typedef ImageContainerBySTLVector < Z2i::Domain, unsigned char> Image2D;
  typedef ConstImageAdapter<Image2D,  Image2D::Domain, 
                            functors::BasicDomainSubSampler<Image2D::Domain>,  
                            Image2D::Value,
                            functors::Identity > ConstImageAdapterForSubSampling;

  ConstImageAdapterForSubSampling* aa;
  ConstImageAdapterForSubSampling::ConstRange r(aa);
  auto rr = r.rbegin();
  f(r);
  f(rr);
  // Z3i::Domain::ConstIterator it({}, {}, {});
  // std::reverse_iterator<Z3i::Domain::ConstIterator> ri(it);
  
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
