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
    template <typename I>
    struct CTrivialConstImage: concepts::CPointFunctor<I>
    {

    public:

      BOOST_CONCEPT_ASSERT((CLabel<typename I::Value>));
      //Inner types
      typedef typename I::Domain Domain;
      BOOST_CONCEPT_ASSERT((concepts::CDomain<Domain>));


      BOOST_CONCEPT_USAGE(CTrivialConstImage)
      {
        ConceptUtils::sameType(i.domain(), d);
      }

    private:
      I i;
      Domain d;

    };

    template <typename I>
    struct CConstImage // : CTrivialConstImage<I>
    {

    public:
      typedef typename I::Domain Domain;
      BOOST_CONCEPT_ASSERT((concepts::CDomain<Domain>));

      typedef typename I::ConstRange ConstRange;
      BOOST_CONCEPT_ASSERT((CConstBidirectionalRangeFromPoint<ConstRange>));

      BOOST_CONCEPT_USAGE(CConstImage)
      {
        ConceptUtils::sameType(i.domain(), d);
        ConceptUtils::sameType(i.constRange(), r);
      }

    private:
      I i;
      Domain d;
      ConstRange r;
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

///////////////////////////////////////////////////////////////////////////////

int main( int /*argc*/, char** /*argv*/ )
{
  
  //! [imageBasicSubsamplingType2D]
  typedef ImageContainerBySTLVector < Z2i::Domain, unsigned char> Image2D;
  typedef ConstImageAdapter<Image2D,  Image2D::Domain, 
                            functors::BasicDomainSubSampler<Image2D::Domain>,  
                            Image2D::Value,
                            functors::Identity > ConstImageAdapterForSubSampling;

  //! [imageBasicSubsamplingType2D]

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
  GenericWriter<ConstImageAdapterForSubSampling3D>::exportFile("", subsampledImage3D );
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
