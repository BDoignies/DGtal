/**
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 **/
#pragma once

// #include <Eigen/Geometry.hpp>

#include <vector>
#include <string>
#include <array>
#include <span>
#include <map>

namespace DGtal { struct Color {}; }
namespace Eigen { struct Affine3d {}; struct AngleAxisd{}; }

namespace DGtal {
    namespace utils {
        template <typename>
        struct is_vector : std::false_type {};
        template <typename T, typename A>
        struct is_vector<std::vector<T, A>> : std::true_type {};
    };

    struct Style {
        DGtal::Color color; // Color of the element
        double size; // Size of the elements. May be width of cube, 
                     // line or radius of ball
        unsigned int ballToQuadThreshold = 4 * 1024; // Threshold to render
                                                     // a quad instead of spheres
        bool drawBackFace = true; // If backfaces should be drawn
    };

    template<typename Space, typename TIndices>
    struct DisplayData {
        using Point = typename Space::RealPoint;
        using Vertex = std::array<Point, Space::Dimension>;
        using Vector = Vertex;
        using T = typename Point::Component;

        inline static const std::string DEFAULT_COLOR_NAME = "Color";
        
        // Style
        Style style;

        // Information to draw geometry
        std::vector<Vertex> vertices;
        std::vector<TIndices> elementIndices;   

        void Append(std::span<const Vertex> vertices, const Color& col) {
            Append(vertices);
            colors[DEFAULT_COLOR_NAME].push_back(col);
        }

        void Append(std::span<const Vertex> v) {
            const std::size_t N = vertices.size();
            vertices.insert(vertices.end(), v.begin(), v.end());
            
            // Assumes elements are separated
            TIndices indices;

            // TIndices may also be a std::vector
            if constexpr (utils::is_vector<TIndices>::value) {
                indices.resize(v.size());
            }

            for (size_t i = 0; i < indices.size(); ++i)
                indices[i] = N + i;
            elementIndices.push_back(std::move(indices));
        }
        
        // Values associated at each face
        std::map<std::string, std::vector<T>> scalarQuantities;
        std::map<std::string, std::vector<Vector>> vectorQuantities;
        std::map<std::string, std::vector<DGtal::Color>> colors;
    };
    
    template<typename Data>
    struct DataGroup {
        DataGroup(const std::string& dName): 
            defaultGroupName(dName), 
            currentGroup(dName), 
            isDefault(true)
        { } 
        
        void Append(std::span<const typename Data::Vertex> data, const Color& col) {
            // Append with current color if default group
            if (isDefault) groups[currentGroup].Append(data, col);
            else           groups[currentGroup].Append(data);
        }

        std::string newGroup(const std::string& name, Style& s) { 
            static const std::string TOKEN = "{i}";

            std::string newGroup = name;
            size_t tokenPos = newGroup.find(TOKEN);  

            if (tokenPos == std::string::npos) {
                size_t i = 0;
                do {
                    newGroup = newGroup.replace(tokenPos, tokenPos + TOKEN.size(), std::to_string(i++));
                } while(groups.find(newGroup) != groups.end());
            }
            else {
                size_t i = 0;
                while (groups.find(newGroup) != groups.end()) {
                    newGroup = name + "_" + std::to_string(i++);
                }
            }

            currentGroup = newGroup;
            groups[currentGroup].style = s;

            isDefault = (currentGroup == defaultGroupName);
        }

        void endGroup() {
            currentGroup = defaultGroupName;
            isDefault = true;
        }

        bool isDefaultSelected() const {
            return isDefault;
        }

        Data& current() { 
            return groups[currentGroup];
        }

        std::size_t size() const {
            return groups.size();
        }

        const std::string defaultGroupName;
        std::map<std::string, Data> groups;
    private:
        // Keep private to ensure isDefault value
        bool isDefault;
        std::string currentGroup;
    };

    /**
     * Information to display an image into the scene
     * 
     * It can handle both 2D and 3D images, but both 
     * are rendered as voxel grids. 
     *
     * @see Display3D
     */
    struct ImageD3D {
        /// Transform
        Eigen::Affine3d transform;

        /// Width of each pixel
        float voxelWidth = 1.f;
        
        /// Data of the image
        std::vector<float> data;
        /// Size of the image
        std::array<size_t, 3> dims;

        ImageD3D(); 
        
        /** 
         * Modify the image data from an arbitrary image and a functor
         *
         * @tparam TImage The image type
         * @tparam TFunctor The functor type
         *
         * @param img The image
         * @param func The functor
         */
        template<typename TImage, typename TFunctor>
        void setData(const TImage& img, const TFunctor& func);
       
        // TODO: As external function
        /**
         *  Convert a direction into a rotation
         *
         *  @param axis The principal direction
         */
        Eigen::AngleAxisd directionToRotation(ImageDirection axis) const; 
        ~ImageD3D(); 
    };
};
