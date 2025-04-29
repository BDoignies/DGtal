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
// namespace DGtal { struct Color {}; }
// namespace Eigen { struct Affine3d {}; struct AngleAxisd{}; }

#include <vector>
#include <string>
#include <array>
#include <span>
#include <map>

namespace DGtal {
    namespace utils {
        template <typename>
        struct is_vector : std::false_type {};
        template <typename T, typename A>
        struct is_vector<std::vector<T, A>> : std::true_type {};

        template<typename>
        struct array_size { inline static constexpr size_t size = 0; };
        template<typename T, size_t N>
        struct array_size<std::array<T, N>> { inline static constexpr size_t size = N; };
    };

    struct Style {
        std::optional<DGtal::Color> color; // Color of the element
        unsigned int ballToQuadThreshold = 4 * 1024; // Threshold to render
                                                     // a quad instead of spheres
        double width = 1.0;
        bool drawBackFace = true; // If backfaces should be drawn
    };

    template<typename Space, typename TIndices>
    struct DisplayData {
        using Point = typename Space::RealPoint;
        using Vertex = Point;
        using Vector = Vertex;
        using Indices = TIndices;
        using T = typename Point::Component;

        inline static constexpr bool VARIABLE_FACE_SIZE = utils::is_vector<TIndices>::value;
        inline static constexpr size_t INDEX_SIZE = utils::array_size<TIndices>::size;

        inline static const std::string DEFAULT_COLOR_NAME = "Color";
        
        // Style
        Style style;

        uint32_t id;
        std::vector<uint32_t> subIndices;

        // Information to draw geometry
        std::vector<Vertex> vertices;
        std::vector<TIndices> elementIndices;

        void Append(std::span<const Vertex> vertices, const Color& col, uint32_t idx) {
            Append(vertices);

            colorQuantities[DEFAULT_COLOR_NAME].push_back(col);
            subIndices.push_back(idx);
        }

        void Append(std::span<const Vertex> v) {
            const std::size_t N = vertices.size();
            vertices.insert(vertices.end(), v.begin(), v.end());
            
            // Assumes elements are separated
            TIndices indices;

            // TIndices may also be a std::vector
            if constexpr (VARIABLE_FACE_SIZE) {
                indices.resize(v.size());
            }

            for (size_t i = 0; i < indices.size(); ++i)
                indices[i] = N + i;
            elementIndices.push_back(std::move(indices));
        }
        
        // Values associated at each face
        std::map<std::string, std::vector<T>> scalarQuantities;
        std::map<std::string, std::vector<Vector>> vectorQuantities;
        std::map<std::string, std::vector<DGtal::Color>> colorQuantities;
    };
    
    template<typename TData>
    struct DataGroup {
        using Data = TData;

        DataGroup(const std::string& dName): 
            defaultGroupName(dName), 
            currentGroup(dName), 
            isDefault(true)
        { } 
        
        void Append(std::span<const typename Data::Vertex> data, const Color& col, uint32_t id) {
            // Append with current color if default group
            if (isDefault) groups[currentGroup].Append(data, col, id);
            else           groups[currentGroup].Append(data);
        }

        std::string newGroup(const Style& s) {
            return newGroup(defaultGroupName, s);
        }

        std::string newGroup(const std::string& name, const Style& s, uint32_t id) { 
            static const std::string TOKEN = "{i}";
            
            std::string newGroup = name;
            if (newGroup.empty())
                newGroup = defaultGroupName + "_{i}";

            size_t tokenPos = newGroup.find(TOKEN);  
            if (tokenPos != std::string::npos) {
                size_t i = 0;
                do {
                    newGroup = newGroup.replace(tokenPos, tokenPos + TOKEN.size(), std::to_string(i++));
                } while(groups.find(newGroup) != groups.end());
            }
            else {
                size_t i = 0;
                while (groups.find(newGroup) != groups.end()) {
                    newGroup = newGroup + "_" + std::to_string(i++);
                }
            }

            currentGroup = newGroup;
            isDefault = (currentGroup == defaultGroupName); 

            if (!isDefault) {
                groups[currentGroup].id = id;
                groups[currentGroup].style = s;
            }

            return currentGroup;
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

        void clear() { 
            groups.clear();
            newGroup(defaultGroupName, Style{});
        }

        const std::string defaultGroupName;
        std::map<std::string, Data> groups;
    private:
        // Keep private to ensure isDefault value
        bool isDefault;
        std::string currentGroup;
    };

    struct ClippingPlaneD3D {
        Style style;
        double a,b,c,d;
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

        ImageD3D() { 
            transform.setIdentity();
        }

        
        /** 
         * Modify the Image data from an arbitrary Image and a functor
         *
         * @tparam Timg The image type
         * @tparam TFunctor The functor type
         *
         * @param img The image
         * @param func The functor
         */
        template<typename Timg, typename TFunctor>
        void setData(const Timg& img, const TFunctor& func) {
            const unsigned int dim = Timg::Domain::Space::dimension; 

            double lowerBound[3];
            for (unsigned int i = 0; i < dim; i++)
            {
                dims[i] = 1 + img.domain().upperBound()[i] - img.domain().lowerBound()[i];
                lowerBound[i] = img.domain().lowerBound()[i] * voxelWidth;
            }
            // For 2D imgs, we also render the back-side
            if (dim == 2)
                dims[2] = 2;
            
            const size_t total = dims[0] * dims[1] * dims[2]; 
            data.resize(total);
            
            using It = typename Timg::Domain::ConstIterator;
            It it = img.domain().begin();
            It itend = img.domain().end();
           
            size_t pos = 0;
            for (; pos < total && it != itend; ++it)
            {
                data[pos] = func(img(*it)); 
                pos++;
            }
            
            transform.translate(Eigen::Vector3d(lowerBound[0], lowerBound[1], lowerBound[2]));

            
            // Also render the back-side of the Image
            // Image with dim of 1 may not be rendered 
            // depending on how the geometry will be computed. 
            // If not needed, this can be stripped easyly
            // by recreating the appropriate dimension
            const size_t start_back = dims[0] * dims[1];
            for (size_t i = start_back; i < total; ++i)
                data[i] = data[i - start_back];
        }

       
        // TODO: As external function
        /**
         *  Convert a direction into a rotation
         *
         *  @param axis The principal direction
         */
        Eigen::AngleAxisd directionToRotation(ImageDirection axis) const {
            switch(axis)
            {
            case DGtal::yDirection:
                return Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
            case DGtal::zDirection:
                return Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
            case DGtal::xDirection:
            case DGtal::undefDirection:
            default:
                return Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
            }
        }
    };
}

