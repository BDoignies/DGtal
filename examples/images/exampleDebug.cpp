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

  // ConstRange        = DefaultConstImageRange<ConstImageAdapterForSubSampling> ConstRange;
  // ConsrRange.rbegin -> ConstReverseIterator
  
  // ConstReverseIterator = std::reverse_iterator<ConstIterator>
  // ConstIterator = ConstIteratorAdapter<Domain::ConstIterator, TImage, Value>
  ConstImageAdapterForSubSampling* a;
  ConstIteratorAdapter<
    typename Z2i::Domain::ConstIterator, // ConstImageAdapterForSubSampling::Domain::ConstIterator
    ConstImageAdapterForSubSampling,     // ConstImageAdapterForSubSampling
    unsigned char                        // ConstImageAdapterForSubSampling::Value
  > adapter(Z2i::Domain{}.begin(), *a);
  std::reverse_iterator<decltype(adapter)> b(adapter);

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
