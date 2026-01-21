#pragma once

#include <vector>

namespace DGtal
{
  template<typename Domain>
  struct SplitInfo
  {
      Domain domain;
      uint32_t hintVoxelCount = 0;
  };

  template<typename Domain>
  struct EvenDomainSplitter
  {
    std::vector<SplitInfo<Domain>> operator()(const Domain& d, uint32_t splitHint) const {
      const uint32_t splitCount = std::floor(std::log(splitHint) / std::log(Domain::dimension));
      const uint32_t totalSplits = std::pow(splitCount, Domain::dimension);

      if (splitCount == 0)
        return { SplitInfo{d, 0} };
      
      auto splitSize = (d.upperBound() - d.lowerBound()) / (int32_t)splitCount;
      std::vector<SplitInfo<Domain>> result;
      
      std::cout << d << std::endl;
      std::cout << "Size: " << splitSize << std::endl;
      std::cout << "Hint: " << splitHint << std::endl;
      std::cout << "Count: " << splitCount << std::endl;
      std::cout << "Splits: " << totalSplits << std::endl;
      result.reserve(totalSplits);

      for (uint32_t i = 0; i < totalSplits; ++i)
      {
        auto start = d.lowerBound();
        auto idx = i;
        for (uint32_t j = 0; j < Domain::dimension; ++j)
        {
            auto k = idx % splitCount;
            start[j] += k * splitSize[j];
            idx /= Domain::dimension;
        }
        result.emplace_back(Domain(start, start + splitSize), 0);
        std::cout << result.back().domain << std::endl;
      }


      return result;
    };
  };
}
