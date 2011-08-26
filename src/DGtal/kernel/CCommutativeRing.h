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
 * @file CCommutativeRing.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/08/26
 *
 * Header file for concept CCommutativeRing.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CCommutativeRing_RECURSES)
#error Recursive header files inclusion detected in CCommutativeRing.h
#else // defined(CCommutativeRing_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CCommutativeRing_RECURSES

#if !defined CCommutativeRing_h
/** Prevents repeated inclusion of headers. */
#define CCommutativeRing_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSignedInteger.h"
#include "DGtal/kernel/IntegerTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CCommutativeRing
  /**
   * Description of \b concept '\b CCommutativeRing' <p>
   * @ingroup Concepts
   * Aim: Definses unitary commutative ring.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t Integer : A type that is a model of CCommutativeRing
   * - \t x, \t y	: Object of type Integer
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
   * <table> <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td> 
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr> 
   * <td> Addition </td> <td> x+y </td> <td> x,y of type Integer </td> <td> Integer</td>
   * <td> </td> <td> scalar addition </td> <td> </td> <td> </td>
   * </tr><tr>
  * <td> Opposite </td> <td> -x </td> <td> x of type Integer </td> <td> Integer</td>
   * <td> </td> <td> opposite of a scalar </td> <td> </td> <td> </td>
   * </tr><tr>
   * <td> multiplication </td> <td> x*y </td> <td> x ,yof type Integer </td> <td> Integer</td>
   * <td> </td> <td> multiplication of scalars </td> <td> </td> <td> </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>: DGta::int32_t,DGta::int8_t; mpz_class;
   *
   * <p> Notes <br>
   */
  template <typename Integer>
  struct CCommutativeRing
  {
    // ----------------------- Concept checks ------------------------------
  public:

    BOOST_CONCEPT_ASSERT((CSignedInteger<Integer>));

    BOOST_CONCEPT_USAGE( CCommutativeRing )
    {
      // x( p ) returns bool.
      ConceptUtils::sameType( c, a+b );
      ConceptUtils::sameType( c, -a );
      ConceptUtils::sameType( c, a*b );  
      ConceptUtils::sameType( c, IntegerTraits<Integer>::ONE );  
      ConceptUtils::sameType( c, IntegerTraits<Integer>::ZERO );  
    
    }
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    Integer a,b,c;
  
  }; // end of concept CCommutativeRing
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CCommutativeRing_h

#undef CCommutativeRing_RECURSES
#endif // else defined(CCommutativeRing_RECURSES)
