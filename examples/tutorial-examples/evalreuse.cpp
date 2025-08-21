/**
 *  this program is free software: you can redistribute it and/or modify
 *  it under the terms of the gnu lesser general public license as
 *  published by the free software foundation, either version 3 of the
 *  license, or  (at your option) any later version.
 *
 *  this program is distributed in the hope that it will be useful,
 *  but without any warranty; without even the implied warranty of
 *  merchantability or fitness for a particular purpose.  see the
 *  gnu general public license for more details.
 *
 *  you should have received a copy of the gnu general public license
 *  along with this program.  if not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file
 * @ingroup examples
 * @author jacques-olivier lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * laboratory of mathematics (cnrs, umr 5127), university of savoie, france
 *
 * @date 2018/06/26
 *
 * an example file named shortcuts.
 *
 * this file is part of the dgtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

#define DEBUG

void ComputeAllSignaturesVoxel(const std::string& path, const int radius) 
{
  typedef Shortcuts<Z3i::KSpace> SH3;
  auto params   = SH3::defaultParameters();
  auto imageptr = SH3::makeBinaryImage( path );

  const auto& im    = *imageptr;
  const auto domain = im.domain();

// std::vector<DGtal::BigInteger> signatures;
  std::set<DGtal::BigInteger> unique_signatures;
  unsigned int vcount = 0;
  for (auto it = domain.begin(); it != domain.end(); ++it) 
  {
    auto pt = *it;
    DGtal::BigInteger signature(0);

    if (im(pt)) 
    {
      vcount ++;
      for (int i = -radius; i <= radius; ++i) 
        for (int j = -radius; j <= radius; ++j) 
          for (int k = -radius; k <= radius; ++k) 
          {
            auto newpt = pt;
            newpt[0] += i;
            newpt[1] += j;
            newpt[2] += k;

            if (domain.isInside(newpt))
              signature += im(newpt);
            signature <<= 1;
          }

      unique_signatures.insert(signature);
    }
//    signatures.push_back(signature);
  }
  double reusePer = 1. - (double) unique_signatures.size() / (double) vcount;
  std::cout << std::fixed <<  std::setprecision(2);
  std::cout << "Model: " << path << " (" << vcount << " voxels) : " << unique_signatures.size() << " unique neighborhood (reuse of " << 100 * reusePer << "%) of radius=" << radius << std::endl;
}

int main(int argc, char** argv)
{
  std::string path = argv[1];
  int radius = std::strtol(argv[2], nullptr, 10);
  std::cout << path << " / " << radius << std::endl;
  ComputeAllSignaturesVoxel(path, radius);
}
