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
  std::vector<int> data;

  using Iterator   = typename std::vector<int>::const_iterator;
  using RVIterator = typename std::reverse_iterator<Iterator>;

  RVIterator r1(data.cbegin());
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
