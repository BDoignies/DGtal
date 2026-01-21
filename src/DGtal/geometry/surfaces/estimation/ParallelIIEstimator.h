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

#include <DGtal/geometry/surfaces/estimation/DomainSplitter.h>
#include <DGtal/topology/helpers/Surfaces.h>
#include <omp.h>

namespace DGtal
{
  template<class TEstimator, typename TSplitter>
  class ParallelIIEstimator
  {
  public:
    using Estimator = TEstimator;
    using Splitter = TSplitter;
    using Domain = typename TEstimator::Domain;
    using Scalar = typename TEstimator::Scalar;

    using KSpace = typename TEstimator::KSpace;
    using PointPredicate = typename TEstimator::PointPredicate;
    using Surfel = typename KSpace::Surfel;
    using SurfelSet = typename KSpace::SurfelSet;
    using EstimatorQuantity = typename TEstimator::Quantity;
    
    // Building the surface
    using Boundary = LightImplicitDigitalSurface<KSpace, PointPredicate>;
    using Surface = DigitalSurface<Boundary>;
    using Visitor = DepthFirstVisitor<Surface>;
    using VisitorRange = GraphVisitorRange<Visitor>;

    struct Quantity
    {
      Surfel location;
      EstimatorQuantity value;

      Quantity(EstimatorQuantity q) {
        value = q;
      };
    };
    
    template<typename... Args>
    ParallelIIEstimator(int32_t nbThread, Args&&... args)
    { 
      // Note: We can not use std::vector constructor 
      // because it will rely on copying the estimator but
      // it can have shared state...
      // We rather build a new one in a loop
      
      if (nbThread <= 0)
        nbThread = std::max(1, omp_get_max_threads());
      nbThread = 10;

      myEstimators.reserve(nbThread);
      for (int i = 0; i < nbThread; ++i)
      {
        myEstimators.emplace_back(std::forward<Args>(args)...);
      }
    }
   
    void clear()
    {
      for (auto& estim : myEstimators)
        estim.clear();
    }

    Scalar h() const { return myEstimators[0].h(); }

    void attach(ConstAlias<KSpace> K, 
                ConstAlias<PointPredicate> pp)
    {
      myKSpace = K;
      myPointPredicate = pp;

      for (auto& estim : myEstimators)
	estim.clear();
    }

    void setParams(double dRadius) 
    {
      myRadius = dRadius;
      for (auto& estim : myEstimators)
        estim.setParams(dRadius);
    }

    bool isValid() const
    {
      bool valid = true;
      for (const auto& estim : myEstimators)
        valid = valid && estim.isValid();
      return valid;
    }

    template<typename ItA, typename ItB>
    void init(double h_, ItA, ItB)
    {
      myH = h_;
    }
    
    template<typename It>
    Quantity eval(It it)
    {
      if (!myEstimators[0].isValid())
      {
        auto ite = it; ite++;
        myEstimators[0].init(myH, it, ite);
      }

      return {
        .location = *it, 
        .value = myEstimators[0].eval(it)
      };
    }

    template<typename It, typename Oit>
    Oit eval(It itb, It ite, Oit rslt)
    {
      std::vector<SplitInfo<Domain>> domains = mySplitter(
         Domain(myKSpace->lowerBound(), myKSpace->upperBound()),
         myEstimators.size() 
      );
      std::vector<std::vector<Quantity>> quantities(domains.size());
      
      std::cout << "Main: " << std::endl;
      std::cout << Domain(myKSpace->lowerBound(), myKSpace->upperBound()) << std::endl;
      for (uint32_t i = 0; i < domains.size(); ++i)
      {
        std::cout << domains[i].domain << std::endl;
        quantities[i].reserve(domains[i].hintVoxelCount);

        KSpace localSpace;
        localSpace.init(
            domains[i].domain.lowerBound(), 
            domains[i].domain.upperBound(), 
            true
        );
        
        auto start = Surfaces<KSpace>::findABel(localSpace, *myPointPredicate, 10000000 /* TODO */);
        Boundary boundary(localSpace, *myPointPredicate, SurfelAdjacency<KSpace::dimension>(true), start);
        Surface surface(boundary);

        VisitorRange range(new Visitor(surface, *surface.begin()));

        auto& estim = myEstimators[i];
        auto beg = range.begin();
        auto end = range.end();
        estim.setParams(myRadius);
        estim.attach(localSpace, *myPointPredicate);
        estim.init(myH, beg, end);
        
        estim.eval(beg, end, std::back_inserter(quantities[i]));
        
        // The underlying convolver won't give any access to current
        // location...
        // auto it = range.begin();
        // for (uint32_t j = 0; j != quantities[i].size(); ++j)
        //   quantities[i][j].location = *it++;
      }

      for (uint32_t i = 0; i < quantities.size(); ++i)
        for (uint32_t j = 0; j < quantities[j].size(); ++j)
          *rslt++ = quantities[i][j];
      return rslt;
    }

  private:
    std::vector<Estimator> myEstimators;
    Splitter mySplitter;

    CountedConstPtrOrConstPtr<PointPredicate> myPointPredicate;
    CountedConstPtrOrConstPtr<KSpace> myKSpace;

    double myH;
    double myRadius;
  };
}
