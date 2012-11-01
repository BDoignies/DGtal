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
 * @file CSeparableMetric.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/11/01
 *
 * Header file for concept CSeparableMetric.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CSeparableMetric_RECURSES)
#error Recursive header files inclusion detected in CSeparableMetric.h
#else // defined(CSeparableMetric_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CSeparableMetric_RECURSES

#if !defined CSeparableMetric_h
/** Prevents repeated inclusion of headers. */
#define CSeparableMetric_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/volumes/distance/CMetric.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CSeparableMetric
/**
Description of \b concept '\b CSeparableMetric' <p>
@ingroup Concepts
@brief Aim: defines the concept of separable metrics. 

Separable metrics are metrics satsifying the monotonicity property. 

### Refinement of CMetric.

### Associated types :

### Notation
 - \e X : A type that is a model of CSeparableMetric
 - \e x, \e y : object of type X

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
|       |            |                   |               |              |           |                |            |

### Invariants

### Models

   A dummy model (for concept checking) is CCSeparableMetricArchetype.

### Notes

@tparam T the type that should be a model of CSeparableMetric.
 */
template <typename T>
struct CSeparableMetric
            // Use derivation for coarser concepts, like
            // : CoarserConcept<T>
            // Think to boost::CopyConstructible<T>, boost::DefaultConstructible<T>, ...
            // http://www.boost.org/doc/libs/1_49_0/libs/concept_check/reference.htm
{
    // ----------------------- Concept checks ------------------------------
public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::InnerType InnerType;
    // possibly check these types so as to satisfy a concept with
    BOOST_CONCEPT_ASSERT(( CConcept< InnerType > ));
    // To test if two types A and Y are equals, use
    BOOST_STATIC_ASSERT(( ConceptUtils::SameType<A,X>::value ));
    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CSeparableMetric )
    {
        // Static members of type A can be tested with
        ConceptUtils::sameType( myA, T::staticMember );
        // non-const method dummy should take parameter myA of type A and return
        // something of type B
        ConceptUtils::sameType( myB, myX.dummy( myA ) );
        // look at CInteger.h for testing tags.
        // check const methods.
        checkConstConstraints();
    }
    void checkConstConstraints() const
    {
        // const method dummyConst should take parameter myA of type A and return
        // something of type B
        ConceptUtils::sameType( myB, myX.dummyConst( myA ) );
    }
    // ------------------------- Private Datas --------------------------------
private:
    T myX; // do not require T to be default constructible.
    A myA;
    B myB;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CSeparableMetric

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSeparableMetric_h

#undef CSeparableMetric_RECURSES
#endif // else defined(CSeparableMetric_RECURSES)
