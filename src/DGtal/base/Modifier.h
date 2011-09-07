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

/**
 * @file Modifier.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/01
 *
 * Header file for module Modifier.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Modifier_RECURSES)
#error Recursive header files inclusion detected in Modifier.h
#else // defined(Modifier_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Modifier_RECURSES

#if !defined Modifier_h
/** Prevents repeated inclusion of headers. */
#define Modifier_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <utility>


#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/RealPointVector.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  
  /////////////////////////////////////////////////////////////////////////////
  // template class Point3dTo2dXY
  /**
   * Description of template class 'Point3dTo2dXY' <p>
   * \brief Aim: transforms a 3d point into a 2d point
   * due to a projection on the xy-plane
   * @tparam Coordinate the type for the coordinates of the points
   * @code
  PointVector<3,int> a3dPoint; 
  PointVector<2,int> a2dPoint; 
  Point3dTo2dXY<int> m; 
  ...
  a2dPoint = m.get(a3dPoint); 
   * @endcode
   * @see ConstIteratorAdapter
   */
  template <typename Coordinate>
  class Point3dTo2dXY
  {
        
    public: 
      
    typedef PointVector<3,Coordinate> Input; 
    typedef PointVector<2,Coordinate> Output; 
    
    public:
      
    /**
     * Get a 2d point from a 3d point
     * @param p a 3d point
     * @return the projected 2d point.
     */
    Output get(const Input& p) const
    {
      Input tmp = p;
      return Output(tmp.at(0),tmp.at(1));
    }
  
  }; // end of class Point3dTo2dXY
  
  /////////////////////////////////////////////////////////////////////////////
  // template class Point3dTo2dXZ
  /**
   * Description of template class 'Point3dTo2dXZ' <p>
   * \brief Aim: transforms a 3d point into a 2d point
   * due to a projection on the xz-plane
   * @tparam Coordinate the type for the coordinates of the points
   * @code
  PointVector<3,int> a3dPoint; 
  PointVector<2,int> a2dPoint; 
  Point3dTo2dXZ<int> m; 
  ...
  a2dPoint = m.get(a3dPoint); 
   * @endcode
   * @see ConstIteratorAdapter
   */
  template <typename Coordinate>
  class Point3dTo2dXZ
  {

    public: 
      
    typedef PointVector<3,Coordinate> Input; 
    typedef PointVector<2,Coordinate> Output; 
    
    public:
      
    /**
     * Get a 2d point from a 3d point
     * @param p a 3d point
     * @return the projected 2d point.
     */
    Output get(const Input& p) const
    {
      Input tmp = p;
      return Output(tmp.at(0),tmp.at(2));
    }
  
  }; // end of class Point3dTo2dXZ

  /////////////////////////////////////////////////////////////////////////////
  // template class Point3dTo2dYZ
  /**
   * Description of template class 'Point3dTo2dYZ' <p>
   * \brief Aim: transforms a 3d point into a 2d point
   * due to a projection on the yz-plane
   * @tparam Coordinate the type for the coordinates of the points
   * @code
  PointVector<3,int> a3dPoint; 
  PointVector<2,int> a2dPoint; 
  Point3dTo2dYZ<int> m; 
  ...
  a2dPoint = m.get(a3dPoint); 
   * @endcode
   * @see ConstIteratorAdapter
   */
  template <typename Coordinate>
  class Point3dTo2dYZ
  {
    
    public: 
      
    typedef PointVector<3,Coordinate> Input; 
    typedef PointVector<2,Coordinate> Output; 
    
    public:
      
    /**
     * Get a 2d point from a 3d point
     * @param p a 3d point
     * @return the projected 2d point.
     */
    Output get(const Input& p) const
    {
      Input tmp = p;
      return Output(tmp.at(1),tmp.at(2));
    }
  
  }; // end of class Point3dTo2dYZ

  /////////////////////////////////////////////////////////////////////////////
  // template class SCellToPoint
  /**
   * Description of template class 'SCellToPoint' <p>
   * \brief Aim: transforms a scell into a point
   * @tparam KSpace, the Khalimsky space 
   * @code 
  KSpace aKSpace;
  KSpace::SCell aSCell; 
  KSpace::Point aPoint; 
  SCellToPoint<KSpace> m(aKSpace);
  ...
  aPoint = m.get(aSCell); 
   * @endcode
   * @see ConstIteratorAdapter KhalimskySpaceND PointVector
   */
  template <typename KSpace>
  class SCellToPoint
  {
    
    private: 
    KSpace myK; 
    
    public: 
      
    typedef typename KSpace::Point Output;
    typedef typename KSpace::SCell Input;
    
    public:
      
     /**
       *  Default constructor.
      */
    SCellToPoint() { }
     /**
       *  Constructor.
       * @param aK a Khalimsky space
      */
    SCellToPoint(KSpace aK) : myK(aK) { }

    /**
     *  Copy constructor.
     * @param other any SCellToPoint modifier
     */
    SCellToPoint(const SCellToPoint& other)
    : myK(other.myK) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SCellToPoint & operator= ( const SCellToPoint & other ) 
    {
      if ( this != &other )
      {
        myK = other.myK; 
      }
      return *this;
    }
    
    /**
     * Get a point (with integer coordinates) from a scell (with khalimsky coordinates)
     * @param s a scell
     * @return the corresponding point.
     */
    Output get(const Input& s) const
    {
      return Output( myK.sCoords(s) );
    }
      
  }; // end of class SCellToPoint

  /////////////////////////////////////////////////////////////////////////////
  // template class SCellToMidPoint
  /**
   * Description of template class 'SCellToMidPoint' <p>
   * \brief Aim: transforms a scell into a real point
   * (the coordinates are divided by 2)
   * @tparam KSpace, the Khalimsky space 
   * @code 
  KSpace aKSpace;
  KSpace::SCell aSCell; 
  RealPointVector<typename KSpace::dimension> aPoint; 
  SCellToMidPoint<KSpace> m(aKSpace); 
  ...
  aPoint = m.get(aSCell); 
   * @endcode
   * @see ConstIteratorAdapter KhalimskySpaceND PointVector RealPointVector
   */
  template <typename KSpace>
  class SCellToMidPoint
  {
    
    public: 
      
    typedef RealPointVector<KSpace::dimension> Output;
    typedef typename KSpace::SCell Input;
    
    private: 
    KSpace myK;   
    
    public:
      
     /**
       *  Default constructor.
      */
    SCellToMidPoint() { }
     /**
       *  Constructor.
       * @param aK a Khalimsky space
      */
    SCellToMidPoint(KSpace aK) : myK(aK) { }

    /**
     *  Copy constructor.
     * @param other any SCellToMidPoint modifier
     */
    SCellToMidPoint(const SCellToMidPoint& other)
    : myK(other.myK) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SCellToMidPoint & operator= ( const SCellToMidPoint & other ) 
    {
      if ( this != &other )
      {
        myK = other.myK; 
      }
      return *this;
    }
    
    /**
     * Get a real point (double coordinates) from a scell (khalimsky coordinates)
     * @param s a scell
     * @return the corresponding point.
     */
    Output get(const Input& s) const
    {
      Output o( myK.sKCoords(s) );
      o /= 2;
      return o;
    } 
      
  }; // end of class SCellToMidPoint

  /////////////////////////////////////////////////////////////////////////////
  // template class SCellToArrow
  /**
   * Description of template class 'SCellToArrow' <p>
   * \brief Aim: transforms a signed cell into an arrow, 
   * ie. a pair point-vector
   * @tparam KSpace, the Khalimsky space 
   * @see SCellToPoint ConstIteratorAdapter KhalimskySpaceND PointVector
   */
  template <typename KSpace>
  class SCellToArrow
  {
    
    public: 
      
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Vector Vector;
    typedef std::pair<Point,Vector> Output;

    typedef typename KSpace::SCell Input;
    
    private: 
    KSpace myK;   
    
    public:
     
     /**
       *  Default constructor.
      */
    SCellToArrow() { }
    
     /**
       *  Constructor.
       * @param aK a Khalimsky space
      */
    SCellToArrow(KSpace aK) : myK(aK) { }

    /**
     *  Copy constructor.
     * @param other any SCellToArrow modifier
     */
    SCellToArrow(const SCellToArrow& other)
    : myK(other.myK) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SCellToArrow & operator= ( const SCellToArrow & other ) 
    {
      if ( this != &other )
      {
        myK = other.myK; 
      }
      return *this;
    }
      
    /**
     * Get an arrow, ie a pair point-vector in integer coordinates
     * from a scell in khalimsky coordinates
     * @param s a scell
     * @return the corresponding point.
     */
    Output get(const Input& s) const
    {
      //starting point of the arrow
      Input pointel( myK.sIndirectIncident( s, *myK.sDirs( s ) ) );
      Point p( myK.sCoords( pointel ) );   //integer coordinates
      //displacement vector
      Vector v( myK.sKCoords( s ) - myK.sKCoords( pointel ) );
      return std::make_pair<Point,Vector>(p,v);
    }
      
  }; // end of class SCellToArrow

  /////////////////////////////////////////////////////////////////////////////
  // template class SCellToInnerPoint
  /**
   * Description of template class 'SCellToInnerPoint' <p>
   * \brief Aim: transforms a signed cell into a point,
   * basically a linel into the indirect incident pixel center
   * @tparam KSpace, the 2d Khalimsky space 
   * @see SCellToPoint SCellToOuterPoint ConstIteratorAdapter KhalimskySpaceND PointVector
   */
  template <typename KSpace>
  class SCellToInnerPoint
  {
    BOOST_STATIC_ASSERT(( KSpace::dimension == 2 ));
    
    public: 
      
    typedef typename KSpace::Point Output;
    typedef typename KSpace::SCell Input;
    
    private: 
    KSpace myK;   
    
    public:
      
     /**
       *  Default constructor.
      */
    SCellToInnerPoint() { }
     /**
       *  Constructor.
       * @param aK a Khalimsky space
      */
    SCellToInnerPoint(KSpace aK) : myK(aK) { }

    /**
     *  Copy constructor.
     * @param other any SCellToInnerPoint modifier
     */
    SCellToInnerPoint(const SCellToInnerPoint& other)
    : myK(other.myK) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SCellToInnerPoint & operator= ( const SCellToInnerPoint & other ) 
    {
      if ( this != &other )
      {
        myK = other.myK; 
      }
      return *this;
    }
      
    /**
     * Get a point (integer coordinates) from a scell (khalimsky coordinates)
     * @param s a linel
     * @return the inner pixel center
     */
    Output get(const Input& s) const
    {
      Input pixel( myK.sIndirectIncident( s, *myK.sOrthDirs( s ) ) );
      return Output( myK.sCoords( pixel ) ); //integer coordinates
    }
      
  }; // end of class SCellToInnerPoint

  /////////////////////////////////////////////////////////////////////////////
  // template class SCellToOuterPoint
  /**
   * Description of template class 'SCellToOuterPoint' <p>
   * \brief Aim: transforms a sigend cell into a point, 
   * basically a linel into the direct incident pixel center
   * @tparam KSpace, the 2d Khalimsky space 
   * @see SCellToPoint SCellToInnerPoint ConstIteratorAdapter KhalimskySpaceND PointVector
   */
  template <typename KSpace>
  class SCellToOuterPoint
  {
    BOOST_STATIC_ASSERT(( KSpace::dimension == 2 ));
    
    public: 
      
    typedef typename KSpace::Point Output;
    typedef typename KSpace::SCell Input;
    
    private: 
    KSpace myK;   
    
    public:
      
     /**
       *  Default constructor.
      */
    SCellToOuterPoint() { }
     /**
       *  Constructor.
       * @param aK a Khalimsky space
      */
    SCellToOuterPoint(KSpace aK) : myK(aK) { }

    /**
     *  Copy constructor.
     * @param other any SCellToOuterPoint modifier
     */
    SCellToOuterPoint(const SCellToOuterPoint& other)
    : myK(other.myK) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SCellToOuterPoint & operator= ( const SCellToOuterPoint & other ) 
    {
      if ( this != &other )
      {
        myK = other.myK; 
      }
      return *this;
    }
      
    /**
     * Get a point (integer coordinates) from a scell (khalimsky coordinates)
     * @param s a linel
     * @return the outer pixel center
     */
    Output get(const Input& s) const
    {
      Input pixel( myK.sDirectIncident( s, *myK.sOrthDirs( s ) ) );
      return Output( myK.sCoords( pixel ) ); //integer coordinates
    }
      
  }; // end of class SCellToOuterPoint

  /////////////////////////////////////////////////////////////////////////////
  // template class SCellToIncidentPoints
  /**
   * Description of template class 'SCellToIncidentPoints' <p>
   * \brief Aim: transforms a linel into a pair of points,  
   * which are the centers of the two incident pixels
   * @tparam KSpace, the 2d Khalimsky space 
   * @see SCellToInnerPoint SCellToOuterPoint ConstIteratorAdapter KhalimskySpaceND PointVector
   */
  template <typename KSpace>
  class SCellToIncidentPoints
  {
    
    BOOST_STATIC_ASSERT(( KSpace::dimension == 2 ));
    
    public: 
      
    typedef typename KSpace::Point Point;
    typedef std::pair<Point,Point> Output;

    typedef typename KSpace::SCell Input;
    
    private: 
    KSpace myK;   
    
    public:
      
     /**
       *  Default constructor.
      */
    SCellToIncidentPoints() { }
     /**
       *  Constructor.
       * @param aK a Khalimsky space
      */
    SCellToIncidentPoints(KSpace aK) : myK(aK) { }

    /**
     *  Copy constructor.
     * @param other any SCellToIncidentPoints modifier
     */
    SCellToIncidentPoints(const SCellToIncidentPoints& other)
    : myK(other.myK) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SCellToIncidentPoints & operator= ( const SCellToIncidentPoints & other ) 
    {
      if ( this != &other )
      {
        myK = other.myK; 
      }
      return *this;
    }
      
    /**
     * Get a pair of point (integer coordinates) from a scell (khalimsky coordinates)
     * @param s a linel
     * @return the inner and outer pixels centers
     */
    Output get(const Input& s) const
    {
      //inner point
      Input innerPixel( myK.sIndirectIncident( s, *myK.sOrthDirs( s ) ) );
      //outer point
      Input outerPixel( myK.sDirectIncident( s, *myK.sOrthDirs( s ) ) );

      return std::make_pair<Point,Point>(myK.sCoords( innerPixel ),myK.sCoords( outerPixel ));
    }
      
  }; // end of class SCellToIncidentPoints
  
  /////////////////////////////////////////////////////////////////////////////
  // template class SCellToCode
  /**
   * Description of template class 'SCellToCode' <p>
   * \brief Aim: transforms a 2d scell, basically a linel, 
    * into a code (0,1,2 or 3), 
   * @tparam KSpace, the 2d Khalimsky space 
   * @see ConstIteratorAdapter KhalimskySpaceND
   */
  template <typename KSpace>
  class SCellToCode
  {
    
    BOOST_STATIC_ASSERT(( KSpace::dimension == 2 ));

    public: 
      
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Vector Vector;
    typedef char Output;

    typedef typename KSpace::SCell Input;
    
    private: 
    KSpace myK;   
    
    public:
      
     /**
       *  Default constructor.
      */
    SCellToCode() { }
     /**
       *  Constructor.
       * @param aK a Khalimsky space
      */
    SCellToCode(KSpace aK) : myK(aK) { }

    /**
     *  Copy constructor.
     * @param other any SCellToCode modifier
     */
    SCellToCode(const SCellToCode& other)
    : myK(other.myK) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SCellToCode & operator= ( const SCellToCode & other ) 
    {
      if ( this != &other )
      {
        myK = other.myK; 
      }
      return *this;
    }
      
    /**
     * Get a code from a linel
     * @param s a linel
     * @return the corresponding code
     */
    Output get(const Input& s) const
    {
      //starting point of the arrow
      Input pointel( myK.sIndirectIncident( s, *myK.sDirs( s ) ) );
      Point p( myK.sCoords( pointel ) );   //integer coordinates
      //displacement vector
      Vector v( myK.sKCoords( s ) - myK.sKCoords( pointel ) );
      if (v == Vector(1,0)) return '0'; 
      else if (v == Vector(0,1)) return '1';
      else if (v == Vector(-1,0)) return '2';
      else if (v == Vector(0,-1)) return '3';
      else return 'e'; //e for error!
    }
      
  }; // end of class SCellToArrow

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/base/Modifier.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Modifier_h

#undef Modifier_RECURSES
#endif // else defined(Modifier_RECURSES)
