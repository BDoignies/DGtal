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
    public:
       /**
        * Default constructor
        *
        * Intializes polyscope
        */
        NewPolyscopeViewer3D();

        /**
         * Constructor from Khalimsky space
         *
         * Initializes polyscope
         *
         * @param emb The khalimsky space
         */
        NewPolyscopeViewer3D(const TKSpace& emb);

        // TODO
        // /**
        //  * Main function to control the viewer
        //  *
        //  * @param key The command
        //  */
        // NewPolyscopeViewer3D<TSpace, TKSpace> & operator<<(
        //     const typename NewPolyscopeViewer3D<TSpace, TKSpace>::StreamKey & key
        // );
        
        /**
         * Show the viewer and start the event loop. Must be called only once. 
         */
        void show() const;

        /**
         * Add an object to draw list
         *
         * @param object The object to draw
         */
        template <typename TDrawableWithViewer3D>
        NewPolyscopeViewer3D<TSpace, TKSpace> & operator<<(const TDrawableWithViewer3D & object);
    protected:
       /** 
         * Create and register polyscope structures
         */
        void createPolyscopeObjects() const;
    };
}

namespace DGtal
{
    template <typename TSpace, typename TKSpace>
    template <typename TDrawableWithViewer3D>
    NewPolyscopeViewer3D<TSpace, TKSpace>& 
    NewPolyscopeViewer3D<TSpace, TKSpace>::operator<<(const TDrawableWithViewer3D & object)
    {
        NewDisplay3DFactory<TSpace, TKSpace>::draw(*this, object);
        return *this;
    }    

    template <typename TSpace, typename TKSpace>
    NewPolyscopeViewer3D<TSpace, TKSpace>::NewPolyscopeViewer3D()
    {
        polyscope::init();  
        // polyscope::state::userCallback = [this](){ this->poyscopeCallback(); };
    }
    
    template <typename TSpace, typename TKSpace>
    NewPolyscopeViewer3D<TSpace, TKSpace>::NewPolyscopeViewer3D(const TKSpace& embd) : NewDisplay3D<TSpace, TKSpace>(embd)
    {
        polyscope::init();  
        // polyscope::state::userCallback = [this](){ this->poyscopeCallback(); };
    }

    template <typename TSpace, typename TKSpace>
    void NewPolyscopeViewer3D<TSpace, TKSpace>::show() const
    {
        createPolyscopeObjects();
        polyscope::show();
    }

    template<typename Data>
    polyscope::Structure* DrawData(const Data& d, const std::string& name)
    {
        const auto& style = d.style;
        polyscope::Structure* structure = nullptr;

        switch(Data::IndicesSize)
        {
        case 1: // Point cloud
            break;
        case 2: // Lines
            break;
        case -1: // Polygonal mesh
        case  3: // Triangles mesh
        case  4: // Quads mesh
            {
                auto smesh = polyscope::registerSurfaceMesh(
                        name, 
                        d.vertices, 
                        d.indices
                );

                structure = smesh;
            }
            break;
        case 8: // Voxel grid
            {
                auto smesh = polyscope::registerVolumeMesh(
                        name, 
                        d.vertices, 
                        d.indices
                );

                structure = smesh;
            }
            break;
        default:
            break;
        };

        return structure;
    }

    template<typename Group>
    void DrawDataGroup(const Group& g, const std::string& name)
    {
        // No structure to draw
        if (g.noGroup.size() + g.groups.size() == 0)
            return;
        
        polyscope::Group* mainGroup = polyscope::createGroup(name);

        if (g.noGroup.size() > 0)
        {
            polyscope::Group* singletons = polyscope::createGroup(name + "/Singletons");

            for (size_t i = 0; i < g.noGroup.size(); i++)
            {
                auto* structure = DrawData(g.noGroup.at(i), name + "_" + std::to_string(i));
                
                if (structure != nullptr)
                    structure->addToGroup(*singletons);
            }

            // Hide infos
            mainGroup->addChildGroup(*singletons);
        }
        
        // Add other informations
        for (const auto& [name, data] : g.groups)
        {
            auto* structure = DrawData(data, name);

            if (structure != nullptr)
                structure->addToGroup(*mainGroup);
        }
    }

    template <typename TSpace, typename TKSpace>
    void NewPolyscopeViewer3D<TSpace, TKSpace>::createPolyscopeObjects() const
    {
        DrawDataGroup(this->voxels, "Voxels");
    }
}

