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
#pragma once

#include <map>
#include <set>
#include <iostream>
#include <functional>
#include "polyscope/polyscope.h"

#include "polyscope/curve_network.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/volume_mesh.h"
#include "polyscope/volume_grid.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"


#include "DGtal/io/NewDisplay3D.h"
#include "DGtal/io/NewDisplay3DFactory.h"

/**
 * @file PolyscopeViewer3D.h
 * @author Bastien Doignies
 *
 * @date 2025/03/20
 *
 * This file is part of the DGtal library.
 */

namespace DGtal
{
    template <typename TSpace  = SpaceND<3>,
              typename TKSpace = KhalimskySpaceND<3>>
    class NewPolyscopeViewer3D : public NewDisplay3D<TSpace, TKSpace>
    {

    };
}

// #include "PolyscopeViewer3D.ih"
