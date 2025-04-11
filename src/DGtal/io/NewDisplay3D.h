#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <span>
#include <map>

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/kernel/CanonicEmbedder.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/images/CImage.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/shapes/Mesh.h"

#include <Eigen/Geometry>

namespace DGtal
{
    template<typename Space, typename KSpace>
    class NewDisplay3D 
    {
    public:    
        typedef typename Space::RealPoint RealPoint;
        /// RealVector type
        typedef typename Space::RealVector RealVector;
        typedef CanonicEmbedder<Space> Embedder;
        typedef CanonicCellEmbedder<KSpace> CellEmbedder;
        typedef CanonicSCellEmbedder<KSpace> SCellEmbedder;


        /**
         * \brief A style to draw an object onto the screen NewD
         */
        struct Style
        {
            /**
             * \brief Drawing mode of the object
             *
             * This enumeration acts as flag to combine multiples draw modes.
             */
            enum class DrawMode 
            {
                DEFAULT = 1
            };

            DGtal::uint64_t mode = static_cast<DGtal::uint64_t>(DrawMode::DEFAULT);
            DGtal::Color color = DGtal::Color::White;
            
            bool backfaceCulling = false;
        };
        
        /**
         * \brief Base structure to display data onto a screen
         *
         * @tparam The types of indices
         *
         * This class is template on the type of indices which 
         * allows to compact data strucutres when the number of 
         * primitive is known in advance.
         */
        template<typename TIndices>
        struct DisplayData
        {
            using Vertices = std::vector<std::array<double, 3>>;
            using Indices = std::vector<TIndices>; 
            
            using ScalarData = std::vector<double>;
            
            /**
             * @brief Append data of an other display data 
             *
             * This function is usefull to build objects 
             * itteratively. 
             * 
             * @param other The data 
             */
            template<typename VType> 
            void Append(std::span<VType> vertices, 
                        std::span<TIndices> indices = {}, 
                        std::span<double> values = {});

            Eigen::Affine3d transform;

            Vertices vertices;
            Indices indices;

            ScalarData scalars;
            Style s;
        };

        struct ClippingPlane
        {
            RealPoint n; /// Normal of the plane
            RealPoint a; /// Point inside the plane

            Style s;
        };

        using IndexType = uint32_t;
        using Points = DisplayData<std::array<IndexType, 1>>;
        using Lines  = DisplayData<std::array<IndexType, 2>>;
        using Triangles = DisplayData<std::array<IndexType, 3>>;
        using Quads = DisplayData<std::array<IndexType, 4>>;
        using Polygonals = DisplayData<std::vector<IndexType>>;
        using Voxels = DisplayData<std::array<IndexType, 8>>;

        using Image = DisplayData<std::array<IndexType, 3>>; 

        /**
         * \brief Groups of data
         *
         * The class manages a group of data, with the ability to register "default group" where object can be pushed indepently
         */
        template<typename TData>
        struct DataGroup 
        {
            const std::string prefix;
            const std::string defaultGroup;

            std::string currentGroup;

            DataGroup(const std::string& prefix, const std::string& defaultGroup);

            void NewGroup(const std::string& newGroup);
            void SetCurrentGroup(const std::string& group);
            void SetDefaultGroup();
            
            /**
             * @brief Retrieves the location to insert new data in
             *
             * It either returns a reference to an element of \ref groups or
             * creates a new element in noGroup.
             *
             * The purpose of this method is to have some unification as to 
             * how data is inserted, regardless of the currentGroup
             *
             * @return A reference to element location
             */
            TData& GetInsertData(); 

            /**
             * @brief Tells if the default group is currently selected
             *
             * The purpose of this method is to tell wether GetInsertData
             * created or not a new element
             *
             * @return True if the default group is selected, false otherwise
             */
            bool IsDefaultGroupSelected() const;

            std::vector<TData> noGroup;
            std::map<std::string, TData> groups;
        };

    public:
        /**
             * @brief Constructor
             *
             * @param KSEmb The khalimsky space for embedding
             */
        NewDisplay3D( const KSpace & KSEmb )
            : myKSpace( KSEmb )
            , myEmbedder( new Embedder() )
            , myCellEmbedder( new CellEmbedder( myKSpace ) )
            , mySCellEmbedder( new SCellEmbedder( myKSpace )  )
            , points("points_", "points")
            , lines("lines_", "lines") 
            , triangles("triangles_", "triangles")
            , cones("cones_", "cones")
            , quads("quads_", "quads")
            , prisms("prism_", "prisms")
            , polygons("polygons_", "polygons")
            , voxels("voxels_", "voxels")
            , images("images_", "images")
            {}
            
            NewDisplay3D() : NewDisplay3D(KSpace()) {}

    public: 
        /// @return the embedder Point -> RealPoint
        const Embedder& embedder() const 
        { return *myEmbedder; }

        /// @return the embedder Cell -> RealPoint
        const CellEmbedder& cellEmbedder() const 
        { return *myCellEmbedder; }

        /// @return the embedder SCell -> RealPoint
        const SCellEmbedder& sCellEmbedder() const 
        { return *mySCellEmbedder; }

        /// @return the cellular grid space.
        const KSpace& space() const 
        { return myKSpace; }

        /**
        * Add a new 3D Clipping plane represented by ax+by+cz+d = 0
        * A maximal of five clipping plane can be added.
        *
        * @param a a
        * @param b b
        * @param c c
        * @param d d plane equation.
        **/
        void addClippingPlane(double a, double b, double c, double d);    
        
        /**
         * @brief Add a new group of a defined geometric type
         *
         * @param of The type of geometric data to create a new list of
         * @param s The new name of the group if empty, use a default new group name
         */
        void createNewGroup(const std::string& of, const std::string& s = "");

        /**
         * @brief Add a new group of a defined geometric type
         *
         * @param of The type of geometric data to create a new list of
         * @param s The new name of the group if empty, use a default new group name
         */
        void endGroup(const std::string& of);

        /**
         * Method to add a specific quad (used by @a addClippingPlane or
         * to represent basic surfels from Khalimsky space). The normal is
         * computed from the vertex order.
         *
         * Note: The quad is added to the current group
         *
         * @param p1 the 1st point
         * @param p2 the 2nd point
         * @param p3 the 3rd point
         * @param p4  the 4th point
        */
        void addQuad(const RealPoint &p1, const RealPoint &p2,
                     const RealPoint &p3, const RealPoint &p4);
        /**
         * Method to add a specific quad. The normal vector is specified
         * by the user. Depending on @a enableReorientation, Quad points
         * can be reordered to make its orientation constistant with the
         * normal direction.
         *
         * @param p1 the 1st point
         * @param p2 the 2nd point
         * @param p3 the 3rd point
         * @param p4  the 4th point
         * @param n the normal vector
         * @param enableReorientation if true,  the quad orientation will
         * match with prescribed normal vector (dot product between the
         * normal and the canonical one is >0).
         * @param enableDoubleFace if true, two quad (with opposite normal
         * vector) will be drawn.
         *
         */
        void addQuadWithNormal(const RealPoint &p1, const RealPoint &p2,
                               const RealPoint &p3, const RealPoint &p4,
                               const RealPoint &n);
        /**
         * Method to add a quad representing a surfel given from its center and its orientation.
         *
         * @param baseQuadCenter the surfel center.
         * @param xSurfel indicates that the sufel is in the x axis direction
         * @param ySurfel indicates that the sufel is in the y axis direction
         * @param zSurfel indicates that the sufel is in the z axis direction
         *
         **/
        void addQuadFromSurfelCenter(const RealPoint &baseQuadCenter, 
                                     bool xSurfel, bool ySurfel, bool zSurfel);
    
    
    
        /**
         * Method to add a quad representing a surfel given from its
         * center and its orientation, and attach a unitary normal vector
         * to it.  Depending on @a enableReorientation, Quad points can be
         * reordered to make its orientation constistant with the normal
         * direction.
         *
         * @param baseQuadCenter the surfel center.
         * @param xSurfel indicates that the sufel is in the x axis direction
         * @param ySurfel indicates that the sufel is in the y axis direction
         * @param zSurfel indicates that the sufel is in the z axis
         * direction
         * @param aNormal a unitary normal vector to attach to the quad.
         * @param enableReorientation if true,  the quad orientation will
         * match with prescribed normal vector (dot product between the
         * normal and the canonical one is >0).
         * @param sign if enableReorientation is true, we use this bool to
         * get the surfel sign
         * @param enableDoubleFace if true, two quad (with opposite normal
         * vector) will be drawn.
         *
         **/
        void addQuadFromSurfelCenterWithNormal(const RealPoint &baseQuadCenter, bool xSurfel, bool ySurfel, bool zSurfel,
                                               const RealVector &aNormal,
                                               const bool enableReorientation,
                                               const bool sign,
                                               const bool enableDoubleFace = false);
    
    
        /**
         * Method to add a specific quad (used by @a addClippingPlane). The normal is computed from the vertex order.
         * @param p1 the 1st point
         * @param p2 the 2nd point
         * @param p3 the 3rd point
         */
        void addTriangle(const RealPoint &p1, const RealPoint &p2, const RealPoint &p3);
    
    
        /**
         * Method to add a specific polygon.
         * @param vertices a vector containing the polygon vertices.
         */
        void addPolygon(const std::vector<RealPoint> &vertices);
    
    
        /**
         * Method to add a line to the current display.
         * x1, y1, z1, x2, y2, z2 the two extremty line points.
         * @param p1 the 1st point
         * @param p2  the 2nd point
         * @param width the line width
         *
         */
    
        void addLine(const RealPoint &p1, const RealPoint &p2, const double width=0.03);
    
    
        /**
         * Method to add specific cube. It includes several modes to
         * display the cube with and without the wire visualisation.
         *
         * @param center cube center
         * @param width the cube width.
         */
        void addCube(const RealPoint &center, double width=1.0);
    
    
        /**
         * Method to add a point to the current display.
         * @param center ball center x
         * @param radius the ball radius (default 0.5)
         * @param resolution ball resolution (default 30)
         *
         */
        void addBall(const RealPoint &center ,
                     const double radius=0.5,
                     const unsigned int resolution = 30);
    
    
    
        /**
         * Specific to display a surfel from Kahlimsky space. The display can
         * take into accounts the sign of the cell.
         * @param baseQuadCenter  base quad center point
         * @param xSurfel true if the surfel has its main face in the direction of the x-axis
         * @param ySurfel true if the surfel has its main face in the direction of the y-axis
         * @param zSurfel true if the surfel has its main face in the direction of the z-axis
         * @param sizeShiftFactor set the distance between the display of the surfel and potential Cube.
         * @param sizeFactor set the difference between the upper face of the prism and the down face
         * @param isSigned to specify if we want to display an signed or unsigned Cell.
         * @param aSign if @a isSigned is true it will be used to apply a different displays
         * according this boolean parameter (if @a aSign=true oriented in the direct axis orientation)
         */
        void addPrism(const RealPoint &baseQuadCenter,
                            bool xSurfel, bool ySurfel, bool zSurfel, double sizeShiftFactor,
                            double sizeFactor=1.0, bool isSigned= false, bool aSign=true);
    
    
    
        /**
         * Specific to display a surfel from Kahlimsky space from a basic way.
         *
         * @param baseQuadCenter  base quad center point
         * @param xSurfel true if the surfel has its main face in the direction of the x-axis
         * @param ySurfel true if the surfel has its main face in the direction of the y-axis
         * @param zSurfel true if the surfel has its main face in the direction of the z-axis
         */
        void addBasicSurfel(const RealPoint &baseQuadCenter,
                            bool xSurfel, bool ySurfel, bool zSurfel);
    
    
        /**
         * Add a signed KSLinel from the Kahlimsky space. Display it as a cone.
         *
         * @param p1  the cone apex
         * @param p2  the cone base
         * @param width the width of the cone (default= 0.08)
         */
        void addCone(const RealPoint &p1, const RealPoint &p2,
                     double width=0.08);
    
    
        /**
         * Add a non signed KSLinel from the Kahlimsky space. Display it as a simple cylinder.
         * @param p1  the 1st point
         * @param p2  the 2nd point
         * @param width the width of the cylinder (default= 0.02)
         */
        void addCylinder(const RealPoint  &p1, const RealPoint &p2,
                         const double width=0.02);
    
        /**
         * Add an image to be rendred
         *
         * @tparam TImage Image type
         * @tparam TFunctor Functor type
         *
         * @param image The image to render
         * @param functor The functor to apply to the image
         */
        template<typename TImage, typename TFunctor>
        void addImage(const TImage& image, const TFunctor& functor);
    
        /*
         * Update the position of an image
         *
         * @param index The index of the image
         * @param pos The new position
         * @param dir The image drawing direction
         */
        void updateImagePosition(size_t index, RealPoint pos,  ImageDirection dir);
        
        /*
         * Update the position of the last image
         *
         * @param pos The new position
         * @param dir The image drawing direction
         */
        void updateLastImagePosition(RealPoint pos, ImageDirection dir);
    
        /**
         * Update the position and width of an image
         * 
         * @param index The index of the image
         * @param lowerLeft The lower left position of the image
         * @param upperLeft The width of the image
         * @param lowerRight The lower
         */
        void update2DImage3DEmbedding(size_t index, RealPoint lowerLeft, RealPoint upperLeft, RealPoint lowerRight);
        /**
         * Used to update the scene bounding box when objects are added.
         *
         * @param point the point to be taken into accounts.
         */
        void updateBoundingBox(const RealPoint &point);
        
        /**
         * Update an image
         *
         * @tparam TImage The type of image
         * @tparam TFunctor The type of functo to apply
         *
         * @param index The index of the image to modify
         * @param newImage The new image data
         * @param functor The functor to apply to the image
         * @param translate The translation to apply
         * @param rotate The new rotation angle
         * @param dir The image main direction
         */
        template<typename TImage, typename TFunctor>
        void updateImage(size_t index, const TImage& newImage, const TFunctor& functor, RealPoint translate, double rotate, ImageDirection dir);
    
        /**
         * Export as Mesh the current displayed elements.
         *
         * @param aMesh : (return) the mesh containing the elements of the display.
         *
         **/
        void exportToMesh(Mesh<RealPoint> & aMesh ) const;
    
    
        /**
         * Draws the drawable [object] in this board. It should satisfy
         * the concept CDrawableWithViewer3D, which requires for instance a
         * method setStyle( Viewer3D & ).
         *
         * @param object any drawable object.
         * @return a reference on 'this'.
         */
        template <typename TDrawableWithDisplay3D>
        NewDisplay3D & operator<<( const TDrawableWithDisplay3D & object );
    
    
    
        /**
         * Writes/Displays the object on an output stream.
         * @param out the output stream where the object is written.
         */
        void selfDisplay ( std::ostream & out ) const;
    
    
        /**
         * Checks the validity/consistency of the object.
         * @return 'true' if the object is valid, 'false' otherwise.
         */
        bool isValid() const;
    
    
        /**
         * Removes all sent data.
         */
        void clear();
    
    
    
        /**
         * Use to embed a DGtal point into space
         * @param dp a DGtal Point
         * @return the point embeded in real space
         */
        RealPoint embed(const typename Space::Point & dp) const ;
    
        /**
         * Use to embed a signed DGtal kahlimsky cell into space
         * @param cell a kahlimsky cell
         * @return the cell embeded in real space
         */
        RealPoint embedKS( const typename KSpace::SCell & cell ) const;
    
    
        /**
         * Use to embed a signed DGtal kahlimsky cell into space
         * @param aTrans a transformed surfel prism
         * @return the cell embeded in real space
         */
        RealPoint embedKS( const DGtal::TransformedPrism& aTrans ) const;
    
    
        /**
         * Use to embed an unsigned DGtal kahlimsky cell into space
         * @param cell kahlimsky cell
         * @return the point embeded in real space
         */
        RealPoint embedK( const typename KSpace::Cell & cell ) const;

    protected:
        /// The Khalimsky space
        KSpace myKSpace;
        /// an embeder from a dgtal space point to a real space point
        Embedder *myEmbedder;
        /// an embeder from a unsigned khalimsky space point to a real space point
        CellEmbedder *myCellEmbedder;
        /// an embeder from a signed khalimsky space point to a real space point
        SCellEmbedder *mySCellEmbedder;

        /// Current style to apply   
        Style currentStyle;    
        
        // Geometrical data  
        DataGroup<Points> points; 
        DataGroup<Lines> lines;   
        DataGroup<Triangles> triangles;
        DataGroup<Triangles> cones;
        DataGroup<Quads> quads;
        DataGroup<Quads> prisms;
        DataGroup<Polygonals> polygons;
        DataGroup<Voxels> voxels; 
        DataGroup<Image> images;

        std::vector<ClippingPlane> planes;
    private:
    };
};

#include "NewDisplay3D.ih"
