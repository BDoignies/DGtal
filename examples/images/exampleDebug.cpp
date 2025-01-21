#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

//! [imageBasicSubsamplingHeaders]
#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/images/ConstImageAdapter.h"
//! [imageBasicSubsamplingHeaders]

#include "DGtal/images/ImageContainerBySTLVector.h"

#include "DGtal/io/readers/GenericReader.h"
#include "DGtal/base/BasicFunctors.h"

namespace DGtal { namespace concepts {
namespace Override
{
    template <typename T>
    struct CConstBidirectionalRangeFromPoint
    {
      // ----------------------- Concept checks ------------------------------
    public:
      typedef typename T::Point Point;
      BOOST_CONCEPT_USAGE( CConstBidirectionalRangeFromPoint )
      {
        concepts::ConceptUtils::sameType( myB, myX.rbegin( myPoint ) );
      }

      // ------------------------- Private Datas --------------------------------
    private:
      T myX;
      Point myPoint;
      typename T::ConstReverseIterator myB;

      // I::ConstRange::ConstReverseIterator === I::ConstRange::rebgin()
      // ------------------------- Internals ------------------------------------
    private:

    };

    template <typename I>
    struct CConstImage 
    {
    public:
      typedef typename I::ConstRange ConstRange;
      BOOST_CONCEPT_ASSERT((CConstBidirectionalRangeFromPoint<ConstRange>)); 
    private:
    };

} } }

template <typename TContainer,
          int Tdim =TContainer::Point::dimension,
          typename TValue = typename TContainer::Value,
          typename TFunctor = DGtal::functors::Identity >
struct GenericWriter
{
  BOOST_CONCEPT_ASSERT((  DGtal::concepts::Override::CConstImage<TContainer> )) ;
  /**
   * Export an  image.
   * @param filename the filename of the saved image (with a extension name).
   * @param anImage the image to be saved.
   * @param aFunctor to apply image transformation before saving.
   *
   **/
  static bool exportFile(const std::string &filename,
                         const TContainer &anImage,
                         const TFunctor & aFunctor = TFunctor() ) { return false; }
};
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

template<typename T>
void f(T&& parameter);  // purposefully not defined

///////////////////////////////////////////////////////////////////////////////

int main( int /*argc*/, char** /*argv*/ )
{
  //! [imageBasicSubsamplingType3D]
  typedef ImageContainerBySTLVector < Z3i::Domain, unsigned char> Image3D;
  typedef ConstImageAdapter<Image3D,  Image3D::Domain, 
                            functors::BasicDomainSubSampler<Image3D::Domain>,
                            Image3D::Value, 
                            functors::Identity > ConstImageAdapterForSubSampling3D;


  //! [imageBasicSubsamplingType3D]
  functors::Identity df;
  Image3D image3D = GenericReader<Image3D>::import( "" );
  DGtal::functors::BasicDomainSubSampler<Image3D::Domain> subSampler3D(image3D.domain(), {0, 0, 0}, Z3i::Point(0 ,0, 0));

  ConstImageAdapterForSubSampling3D subsampledImage3D (
    image3D, 
    subSampler3D.getSubSampledDomain(), 
    subSampler3D, 
    df);
  // GenericWriter<ConstImageAdapterForSubSampling3D>::exportFile("", subsampledImage3D );

  using ConstRange = typename ConstImageAdapterForSubSampling3D::ConstRange;
  using Expected   = typename ConstRange::ConstReverseIterator;
  
  auto r  = subsampledImage3D.constRange();
  // auto rr = r.rbegin(Z3i::Point{});
  
  Z3i::Domain::ConstIterator it({}, {}, {});
  std::reverse_iterator<Z3i::Domain::ConstIterator> ri(it);
  // f(r);
  // f(rr);
  f(ri);
  // static_assert(std::is_same_v<Expected, decltype(rr)>);
  
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
