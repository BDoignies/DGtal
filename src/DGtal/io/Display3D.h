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

/**
 * @file Display3D.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/08/08
 *
 * Header file for module Display3D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Display3D_RECURSES)
#error Recursive header files inclusion detected in Display3D.h
#else // defined(Display3D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Display3D_RECURSES

#if !defined Display3D_h
/** Prevents repeated inclusion of headers. */
#define Display3D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include "DGtal/base/BasicTypes.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/io/Color.h"
#include "DGtal/images/CImage.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/shapes/Mesh.h"

// For transforms
#include <Eigen/Dense>
#include <Eigen/Geometry>

/// for embedding
#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/kernel/CanonicEmbedder.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/Display3DData.h"
#include "DGtal/kernel/CSpace.h"


//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class Display3D
  /**
   * Description of class 'Display3D' <p>
   * \brief Aim: This semi abstract class defines the stream mechanism to
   display 3d primitive (like BallVector, DigitalSetBySTLSet, Object
   ...). The class Viewer3D and Board3DTo2D implement two different
   ways to display 3D objects. The first one (Viewer3D), permits an
   interactive visualisation (based on @a OpenGL ) and the second one
   (Board3dto2d) provides 3D visualisation from 2D vectorial display
   (based on the CAIRO library)
   @see Viewer3D, Board3DTo2D
   *
   * This class is parametrized by both the Digital and Khalimsky
   * space used to display object. More precisely, embed methods are
   * used to compute the Euclidean coordinate of digital
   * objects/khalimksy cells.
   *
   * @tparam Space any model of Digital 3D Space
   * @tparam KSpace any mode of Khalimksky 3D space
   */
  template < typename Space = Z3i::Space, typename KSpace = Z3i::KSpace>
  class Display3D
  {
  public:

    BOOST_CONCEPT_ASSERT((concepts::CSpace<Space>));
  public:

    typedef Display3D<Space,KSpace> Self;
    /// RealPoint type
    typedef typename Space::RealPoint RealPoint;
    /// RealVector type
    typedef typename Space::RealVector RealVector;
    typedef CanonicEmbedder<Space> Embedder;
    typedef CanonicCellEmbedder<KSpace> CellEmbedder;
    typedef CanonicSCellEmbedder<KSpace> SCellEmbedder;

    /// Select callback function type.
    typedef int (*SelectCallbackFct)( void* viewer, DGtal::int32_t name, void* data );

  protected:

    /// Structure for storing select callback functions. The select
    /// callback function is called whenever the user select a
    /// graphical object with "OpenGL name" in [min,max]. The order
    /// relation is used to find quickly the correct function.
    struct SelectCallbackFctStore {
      SelectCallbackFctStore( SelectCallbackFct _fct, 
                              void* _data,
                              DGtal::int32_t _min, DGtal::int32_t _max )
        : fct( _fct ), data( _data ), min( _min ), max( _max ) {}
      bool operator<( const SelectCallbackFctStore& other ) const
      { 
        return ( min < other.min ); // simple since there is no overlap.
      }
      bool isSelected( DGtal::int32_t name ) const
      { return ( min <= name ) && ( name <= max ); }

      SelectCallbackFct fct;
      void*             data;
      DGtal::int32_t    min;
      DGtal::int32_t    max;
    };
public:
    enum StreamKey {addNewList, updateDisplay, shiftSurfelVisu};

protected:
    /// The Khalimsky space
    KSpace myKSpace;
    /// an embeder from a dgtal space point to a real space point
    Embedder *myEmbedder;
    /// an embeder from a unsigned khalimsky space point to a real space point
    CellEmbedder *myCellEmbedder;
    /// an embeder from a signed khalimsky space point to a real space point
    SCellEmbedder *mySCellEmbedder;
    //----end of private data

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    virtual ~Display3D()
    {
      delete myEmbedder;
      delete mySCellEmbedder;
      delete myCellEmbedder;
    }

    /**
     * Constructor with the Khalimsky Space
     * @param KSEmb the khalimsky space for embedding
     */
    Display3D( const KSpace & KSEmb )
      : myKSpace( KSEmb )
      , myEmbedder( new Embedder() )
      , myCellEmbedder( new CellEmbedder( myKSpace ) )
      , mySCellEmbedder( new SCellEmbedder( myKSpace )  )
      , myBoundingPtEmptyTag( true )
      , myLineSetList("Lines")
      , myBallSetList("Points")
      , myPrismList("Prisms")
      , myQuadsMap("Quads")
      , myTriangleSetList("Triangles")
      , myPolygonSetList("Polygones")
      , myCubesMap("Voxels")
    {
    }
    
    /**
     * Default constructor
     * Display3D
     */
    Display3D()
      : Display3D( KSpace() )
    {
    }

    /// Copy constructor. Deleted.
    Display3D( const Display3D & ) = delete;

    /// Move constructor. Deleted.
    Display3D( Display3D && ) = delete;

    /// Assignment operator. Deleted.
    Display3D & operator= ( const Display3D & ) = delete;

    /// Move operator. Deleted.
    Display3D & operator= ( Display3D && ) = delete;

    // ----------------------- Interface --------------------------------------
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
     * Used to set the current fill color
     * @param aColor the fill color.
     **/
    virtual void setFillColor(DGtal::Color aColor);

    /**
     * Used to set the alpha value of the current fill color.
     * @param alpha the transparency value (from 0 to 255).
     **/
    virtual void setFillTransparency(unsigned char alpha);


    /**
     * Used to set the line fill color
     * @param aColor the line color.
     **/
    virtual void setLineColor(DGtal::Color aColor);


    /**
     * Used to get the fill color
     * @return the current fill color.
     **/

    virtual DGtal::Color getFillColor();

    /**
     * Used to get the line color
     * @return the current line color.
     **/

    virtual DGtal::Color getLineColor();

    /**
     *  Used to change the Khalimsky 3D Space.
     * @param aKSpace the new Khalimsky space.
     **/
    virtual void  setKSpace( const KSpace & aKSpace );


    /**
     * Sets the "OpenGL name" for next graphical directives.
     * @param name the "OpenGL name", an integer identifier or -1 for none.
     */
    void setName3d( DGtal::int32_t name = -1 );

    /**
     * @return the current "OpenGL name", an integer identifier or -1 if none was set.
     */
    DGtal::int32_t name3d() const;

    /**
     * Sets the callback function called when selecting a graphical
     * object with "OpenGL name" between \a min_name and \a max_name.
     * Note that ranges should not overlap. If several functions can
     * be called, behavior is undefined afterwards.
     *
     * @param fct any function.
     * @param data an arbitrary pointer that is given when calling the callback function.
     * @param min_name the first "OpenGL name" for which \a fct should be called.
     * @param max_name the last "OpenGL name" for which \a fct should be called.
     */
    void setSelectCallback3D( SelectCallbackFct fct, void* data,
                              DGtal::int32_t min_name, DGtal::int32_t max_name );

    /**
     * @param[in]  aName the "OpenGL name" that was selected.
     * @param[out] data a pointer that was given setting the callback function.
     * @return the select callback function that match the given \a
     * name, or 0 if none is associated to this name.
     */
    SelectCallbackFct getSelectCallback3D( DGtal::int32_t aName, void*& data ) const;

    // ----------------------- Graphical directives ----------------------------------
  public:


    /**
     * Add a new 3D Clipping plane represented by ax+by+cz+d = 0
     * A maximal of five clipping plane can be added.
     *
     * @param a a
     * @param b b
     * @param c c
     * @param d d plane equation.
     * @param drawPlane true if the plane should be draw
     **/

    virtual void addClippingPlane(double a, double b, double c, double d, bool drawPlane);




    /**
     * @param objectName the name of the object (generally obtained
     * with a 'object.className()').
     *
     * @return the current mode for the given object name or "" if no
     * specific mode has been set.
     */
    std::string getMode( const std::string & objectName ) const;

    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     **/
    std::string createNewLineList(std::string s= "");
 
    /**
     * Used to set the current list to the singletons list
     */
    void setDefaultLineList();

    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     **/
    std::string createNewBallList(std::string s= "");
 
    /**
     * Used to set the current list to the singletons list
     */
    void setDefaultBallList();
    
    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     *
     * @param s Name for the group
     *
     * @return the new key of the map associated to the new list.
     **/
    std::string createNewCubeList(std::string s = "");
    
    /**
     * Used to set the current list to the singletons list
     */
    void setDefaultCubeList();
    
    /**
     * Delete the cube list identified by a its name.
     * @param[in] name the name of the cube list.
     * @return true if the list was found and removed.
     *
     **/
    bool deleteCubeList(std::string name);

     /**
      * Used to create a new list containing new 3D objects
      * (useful to use transparency between different objects).
      * @param s The name of the list
      *
      * @return the new key of the map associated to the new list.
      **/
    std::string createNewQuadList(std::string s = "");

    /**
     * Used to set the current list to the singletons list
     */
    void setDefaultQuadList();

    /**
     * Delete the quad list identified by a its name.
     * @param[in] name the name of the quad list.
     * @return true if the list was found and removed.
     *
     **/
    bool deleteQuadList(std::string name);

    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     **/
    std::string createNewTriangleList(std::string s= "");

    /**
     * Used to set the current list to the singletons list
     */
    void setDefaultTriangleList();

    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     **/
    std::string createNewPolygonList(std::string s= "");

    /**
     * Used to set the current list to the singletons list
     */
    void setDefaultPolygonList();

    /**
     * Method to add a specific quad (used by @a addClippingPlane or
     * to represent basic surfels from Khalimsky space). The normal is
     * computed from the vertex order.
     *
     * @param p1 the 1st point
     * @param p2 the 2nd point
     * @param p3 the 3rd point
     * @param p4  the 4th point
     *
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
                           const RealPoint &n,
                           const bool enableReorientation,
                           const bool enableDoubleFace = false);

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
    Display3D & operator<<( const TDrawableWithDisplay3D & object );



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

    //---end interface

    // ------------------------- Protected Datas ------------------------------
  public:

    /**
     * The associated map type for storing possible modes used for
     * displaying for digital objects.
     */
    typedef std::map< std::string, std::string > ModeMapping;

    /**
     * The associated map type for storing the default styles of
     * digital objects.
     */
    typedef std::map< std::string,CountedPtr<DrawableWithDisplay3D> > StyleMapping;


    ModeMapping myModes;
    /**
     * For instance, may associate a new style object T1 to the class
     * "HyperRectDomain": myStyles[ "HyperRectDomain" ] = T1.
     *
     * One can also store a new style T2 for a specific mode used for
     * drawing a class: myStyles[ "HyperRectDomain/Paving" ] = T2.
     *
     * Modes may only be used in objects implementing the concept
     * CDrawableWithBoard2D.
     */
    StyleMapping myStyles;

    /// True if the bounding box is empty (no objects added)
    bool myBoundingPtEmptyTag;
    ///upper point of the bounding box
    double myBoundingPtUp [3];
    /// lower point of the bouding box
    double myBoundingPtLow [3];

  protected:
    Style myCurrentStyle;
    /// Used to specialized visualisation with KSpace surfels/cubes.
    ///
    double myCurrentfShiftVisuPrisms;

    /// Represents all the images drawn in the Display3D
    std::vector<ImageD3D> myImageSetList;
  
    /// Represent all the clipping planes added to the scene (of maxSize=5).
    ///
    std::vector< ClippingPlaneD3D > myClippingPlaneList;

    using Index = DGtal::int64_t;
    using Points     = DisplayData<Space, std::array<Index, 1>>;
    using Lines      = DisplayData<Space, std::array<Index, 2>>;
    using Triangles  = DisplayData<Space, std::array<Index, 3>>;
    using Quads      = DisplayData<Space, std::array<Index, 4>>;
    using Voxels     = DisplayData<Space, std::array<Index, 8>>;
    using Polygons   = DisplayData<Space, std::vector<Index>>;


    /// Used to represent all the list of line primitive
    DataGroup<Lines> myLineSetList;

    /// Used to represent all the list of point primitive
    DataGroup<Points> myBallSetList;
  
    /// Represent truncated prism object to represent surfels of Khalimsky space (used to display Khalimsky Space Cell)
    DataGroup<Quads> myPrismList;

    /// Represents all the planes drawn in the Display3D or to display
    /// Khalimsky Space Cell.  
    DataGroup<Quads> myQuadsMap;

    /// Represents all the triangles drawn in the Display3D
    DataGroup<Triangles> myTriangleSetList;

    /// Represents all the polygon drawn in the Display3D
    /// Represents all the cubes drawn in the Display3D.
    DataGroup<Polygons> myPolygonSetList;
    DataGroup<Voxels> myCubesMap;


    /// the "OpenGL name", used for instance by QGLViewer for selecting objects.
    DGtal::int32_t myName3d;

    /// Stores the callback functions called when selecting a graphical object.
    ///
    std::set<SelectCallbackFctStore> mySelectCallBackFcts;

    bool myBoundingPtChangedTag = false;

    //----end of protected datas

    // ------------------------- Internals ------------------------------------
  protected:

    /**
     * Calculate the cross product of two 3d vectors and return it.
     * @param dst destination vector.
     * @param srcA source vector A.
     * @param srcB source vector B.
     */
    static void cross (double dst[3], double srcA[3], double srcB[3]);

    /**
     * Normalize the input 3d vector.
     * @param vec source & destination vector.
     */
    static void normalize (double vec[3]);


  }; // end of class Display3D

  /**
   * Overloads 'operator<<' for displaying objects of class 'Display3D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Display3D' to write.
   * @return the output stream after the writing.
   */
  template <typename Space , typename KSpace >
  std::ostream&
  operator<< ( std::ostream & out, const DGtal::Display3D<Space , KSpace > & object );


  template <typename Space , typename KSpace >
  /**
   * Operator ">>" to export a Display3D into a Mesh
   * @param aDisplay3D the Display3D to be exported.
   * @param aMesh (return) the resulting mesh.
   *
   **/
  void
  operator>> ( const Display3D<Space , KSpace > &aDisplay3D,
               DGtal::Mesh< typename Display3D<Space , KSpace >::RealPoint > &aMesh);


  /**
   * Operator ">>" to export a Display3D directly a file
   * @param aDisplay3D the Display3D to be exported.
   * @param aFilename (return) the resulting mesh.
   *
   **/
  template < typename Space , typename KSpace >
  void
  operator>> ( const Display3D< Space , KSpace > &aDisplay3D, std::string aFilename);


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/Display3D.ih"


// //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Display3D_h

#undef Display3D_RECURSES
#endif // else defined(Display3D_RECURSES)
