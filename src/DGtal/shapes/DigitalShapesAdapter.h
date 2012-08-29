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
 * @file DigitalShapesAdapter.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/08/28
 *
 * Header file for module DigitalShapesAdapter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalShapesAdapter_RECURSES)
#error Recursive header files inclusion detected in DigitalShapesAdapter.h
#else // defined(DigitalShapesAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalShapesAdapter_RECURSES

#if !defined DigitalShapesAdapter_h
/** Prevents repeated inclusion of headers. */
#define DigitalShapesAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/shapes/CDigitalBoundedShape.h"
#include "DGtal/shapes/CDigitalOrientedShape.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalShapesUnion
  /**
   * Description of template class 'DigitalShapesUnion' <p>
   * \brief Aim:
   */
  template <typename ShapeA, typename ShapeB>
  class DigitalShapesUnion
  {
    // ----------------------- Standard services ------------------------------
  public:
    ///@todo BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape<ShapeA> ));
    ///@todo BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape<ShapeA> ));
    typedef typename ShapeA::Space Space;
    typedef typename Space::Point Point;
    typedef typename Space::RealPoint RealPoint;

    DigitalShapesUnion( const ShapeA & a, const ShapeB & b )
      : shapeA(a),
        shapeB(b)
    {}

    bool isInside( const RealPoint & p ) const
    {
      return shapeA.isInside( p ) || shapeB.isInside( p );
    }

    bool isInside( const Point & p ) const
    {
      return shapeA.isInside( p ) || shapeB.isInside( p );
    }

    /**
     * Destructor.
     */
    ~DigitalShapesUnion(){}

    // ----------------------- Interface --------------------------------------
  public:

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

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DigitalShapesUnion();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DigitalShapesUnion ( const DigitalShapesUnion & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DigitalShapesUnion & operator= ( const DigitalShapesUnion & other );

    // ------------------------- Internals ------------------------------------
  private:
    const ShapeA & shapeA;
    const ShapeB & shapeB;

  }; // end of class DigitalShapesUnion

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalShapesIntersection
  /**
   * Description of template class 'DigitalShapesIntersection' <p>
   * \brief Aim:
   */
  template <typename ShapeA, typename ShapeB>
  class DigitalShapesIntersection
  {
    // ----------------------- Standard services ------------------------------
  public:
    ///@todo BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape<ShapeA> ));
    ///@todo BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape<ShapeA> ));
    typedef typename ShapeA::Space Space;
    typedef typename Space::Point Point;
    typedef typename Space::RealPoint RealPoint;

    DigitalShapesIntersection( const ShapeA & a, const ShapeB & b )
      : shapeA(a),
        shapeB(b)
    {}

    bool isInside( const RealPoint & p ) const
    {
      return shapeA.isInside( p ) && shapeB.isInside( p );
    }

    bool isInside( const Point & p ) const
    {
      return shapeA.isInside( p ) && shapeB.isInside( p );
    }

    /**
     * Destructor.
     */
    ~DigitalShapesIntersection(){}

    // ----------------------- Interface --------------------------------------
  public:

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

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DigitalShapesIntersection();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DigitalShapesIntersection ( const DigitalShapesIntersection & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DigitalShapesIntersection & operator= ( const DigitalShapesIntersection & other );

    // ------------------------- Internals ------------------------------------
  private:
    const ShapeA & shapeA;
    const ShapeB & shapeB;

  }; // end of class DigitalShapesIntersection


  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalShapesMinus
  /**
   * Description of template class 'DigitalShapesMinus' <p>
   * \brief Aim:
   */
  template <typename ShapeA, typename ShapeB>
  class DigitalShapesMinus
  {
    // ----------------------- Standard services ------------------------------
  public:
    ///@todo BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape<ShapeA> ));
    ///@todo BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape<ShapeA> ));
    typedef typename ShapeA::Space Space;
    typedef typename Space::Point Point;
    typedef typename Space::RealPoint RealPoint;

    DigitalShapesMinus( const ShapeA & a, const ShapeB & b )
      : shapeA(a),
        shapeB(b)
    {}

    bool isInside( const RealPoint & p ) const
    {
      return shapeA.isInside( p ) && !shapeB.isInside( p );
    }

    bool isInside( const Point & p ) const
    {
      return shapeA.isInside( p ) && !shapeB.isInside( p );
    }

    /**
     * Destructor.
     */
    ~DigitalShapesMinus(){}

    // ----------------------- Interface --------------------------------------
  public:

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

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DigitalShapesMinus();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DigitalShapesMinus ( const DigitalShapesMinus & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DigitalShapesMinus & operator= ( const DigitalShapesMinus & other );

    // ------------------------- Internals ------------------------------------
  private:
    const ShapeA & shapeA;
    const ShapeB & shapeB;

  }; // end of class DigitalShapesMinus


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalShapesAdapter'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalShapesAdapter' to write.
   * @return the output stream after the writing.
   */
  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalShapesUnion< ShapeA, ShapeB > & object );

  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalShapesIntersection< ShapeA, ShapeB > & object );

  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalShapesMinus< ShapeA, ShapeB > & object );

} // namespace DGtal



#endif // !defined DigitalShapesAdapter_h

#undef DigitalShapesAdapter_RECURSES
#endif // else defined(DigitalShapesAdapter_RECURSES)
