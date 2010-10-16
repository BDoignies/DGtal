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
//LICENSE-END
#pragma once

/**
 * @file FreemanChain.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2010/07/01
 *
 * Header file for module FreemanChain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(FreemanChain_RECURSES)
#error Recursive header files inclusion detected in FreemanChain.h
#else // defined(FreemanChain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FreemanChain_RECURSES

#if !defined FreemanChain_h
/** Prevents repeated inclusion of headers. */
#define FreemanChain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/kernel/PointVector.h"
#include "DGtal/base/OrderedAlphabet.h"
#include "DGtal/base/BasicTypes.h"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class FreemanChain
  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Description of class 'FreemanChain' <p> Aim: Describes a digital
   * 4-connected contour as a string of '0', '1', '2', and '3' and the
   * coordinate of the first point. When it is a loop, it is the
   * counterclockwise boundary of the shape.
   */


  class FreemanChain
  {

  public :
    typedef PointVector<2,int> PointI2;

    // ------------------------- iterator ------------------------------
  public:
    /**
     * This class represents an iterator on the freeman chain, storing
     * the current coordinate.
     */
    class ConstIterator
    {
      // ------------------------- data -----------------------
    private:
      /**
       * The Freeman chain visited by the iterator.
       */
      const FreemanChain* myFc;

      /**
       * The current position in the word.
       */
      unsigned int myPos;

      /**
       * The current coordinates of the iterator.
       */
      PointI2  myXY;
      


      // ------------------------- Standard services -----------------------
    public:
      /**
       * Default Constructor.
       * The object is not valid.
       */
      ConstIterator();

      /**
       * Constructor.
       * Nb: complexity in O(n).
       *
       * @param aChain a Freeman chain,
       * @param n the position in [chain] (within 0 and chain.size()-1).
       */
      ConstIterator( const FreemanChain & aChain, unsigned int n = 0 );

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
      ConstIterator( const ConstIterator & aOther );
    
      /**
       * Assignment.
       * @param aOther the iterator to copy.
       * @return a reference on 'this'.
       */
      ConstIterator& operator=( const ConstIterator & aOther );
    
      /**
       * Destructor. Does nothing.
       */
      ~ConstIterator();
    
      // ------------------------- iteration services -------------------------
    public:
      
      /**
       * @return the current coordinates.
       */
      PointI2 operator*() const;

      /**
       * @return the current coordinates.
       */
      PointI2 get() const;

      /**
       * Pre-increment.
       * Goes to the next point on the chain.
       */
      ConstIterator& operator++();
      
      /**
       * Goes to the next point on the chain.
       */
      void next();

      /**
       * Goes to the next point on the chain as if on a loop.
       */
      void nextInLoop();

      /**
       * @return the current position (as an index in the Freeman chain).
       */
      unsigned int getPosition() const;

      /**
       * @return the associated Freeman chain.
       */
      const FreemanChain* getChain() const;

      /**
       * @return the current Freeman code (specifies the movement to
       * the next point).
       */
      unsigned int getCode() const;

      /**
       * Pre-decrement.
       * Goes to the previous point on the chain.
       */
      ConstIterator& operator--();
      
      /**
       * Goes to the previous point on the chain if possible.
       */
      void previous();

      /**
       * Goes to the previous point on the chain as if on a loop.
       */
      void previousInLoop();

      /**
       * Equality operator.
       *
       * @param aOther the iterator to compare with (must be defined on
       * the same chain).
       *
       * @return 'true' if their current positions coincide.
       */
      bool operator==( const ConstIterator & aOther ) const;

      /**
       * Inequality operator.
       *
       * @param aOther the iterator to compare with (must be defined on
       * the same chain).
       *
       * @return 'true' if their current positions differs.
       */
      bool operator!=( const ConstIterator & aOther ) const;

      /**
       * Inferior operator.
       *
       * @param aOther the iterator to compare with (must be defined on
       * the same chain).
       *
       * @return 'true' if the current position of 'this' is before
       * the current position of [other].
       */
      bool operator<( const ConstIterator & aOther ) const;
      
    };
      

    // ------------------------- static services ------------------------------
  public:

    /**
     * Outputs the chain [c] to the stream [out].
     * @param out any output stream,
     * @param c a Freeman chain.
     */
    static void write( std::ostream & out, const FreemanChain & c );

    /**
     * Reads a chain from the stream [in] and updates [c].
     * @param in any input stream,
     * @param c (returns) the Freeman chain.
     */
    static void read( std::istream & in, FreemanChain & c );

    /**
     * Creates a Freeman chaincode [chain] and a chain coding the
     * quadrant of each step [qchain], given an iterator on a
     * 4-connected path [it], a number of step [nb], the first step
     * [freeman_code] and the first quadrant [quadrant].
     *
     * @param chain (returns) a string of '0', '1', '2', or '3'
     * (Freeman chaincode)
     *
     * @param qchain (returns) a string of '0', '1', '2', or '3'
     * giving the quadrants.
     *
     * @param it any iterator 
     *
     * @param nb the number of 'next' performed on a copy of [it].
     *
     * @param freeman the first code or step.
     *
     * @param quadrant the first quadrant (equal to freeman_code or one below).
     *
     * @param start_index the starting index in [chain] and [qchain],
     * default is 0.
     */
    
    //static void create( std::string & chain,
    //			std::string & qchain,
    //			const C4CIterator & it, 
    //			unsigned int nb, unsigned int freeman, unsigned int quadrant,
    //			unsigned int start_index = 0 );
  
  
    /**
     * @param aZero (returns) the '0' or 'x' letter for quadrant [quadrant].
     * @param aOne (returns) the '1' or 'y' letter for quadrant [quadrant].
     * @param aQuadrant the quadrant as any of '0', '1', '2', or '3'.
     */
    static void alphabet( char & aZero, char & aOne, char aQuadrant );

    /**
     * Given two consecutive moves on a Freeman chain code, this
     * method returns the type of movement: 0: return move, 1: turning
     * toward the interior, 2: going straight, 3: turning toward
     * exterior. Interior/exterior is specified by [ccw].
     *
     * @param aCode1 the code of the first step as an integer in 0..3.
     * @param aCode2 the code of the second step as an integer in 0..3.
     * @param ccw 'true' if the contour is seen counterclockwise with
     * its inside to the left.
     */
    static unsigned int movement( unsigned int aCode1, unsigned int aCode2, bool ccw = true );

    /**
     * Returns the displacement vector of a Freeman code.
     *
     * @param dx (returns) the x-displacement.
     * @param dy (returns) the y-displacement.
     * @param aCode the code.
     */
    static void displacement( int & dx, int & dy, unsigned int aCode );

    /**
     * @param aCode a Freeman code (between 0-3).
     * Returns the displacement vector of the Freeman code.
     */
    static PointI2 displacement( unsigned int aCode );

    /**
     * @param aCode any Freeman code.
     *
     * @param ccw when 'true' turns counterclockwise (or left),
     * otherwise turns clockwise (right).
     *
     * @return the turned code.
     */
    static unsigned int turnedCode( unsigned int aCode, bool ccw = true );
    
    /**
     * From the Freeman chain [pl_chain] representing a pointel
     * 4-connected contour, constructs the Freeman chain [pix_chain]
     * that represents its inner 4-connected border of pixels. The
     * Freeman chain [pl_chain] has its inside to the left (ie. ccw).
     * 
     * Note that chain codes going back and forth are @b never considered
     * useless: it means that the chain is always supposed to have its
     * interior to the left (ccw) or right (cw) even at configurations
     * "02", "13", "20", "31".
     * 
     * @param aPix_chain (output) the code of the 4-connected inner border. 
     *
     * @param aPl2pix (output) the mapping associating pointels to
     * pixels as indices in their respective Freeman chain.
     *
     * @param aPix2pl (output) the inverse mapping associating pixels to
     * pointels as indices in their respective Freeman chain.
     *
     * @param pl_chain the input code of the 4-connected pointel contour.
     */
    static void pointel2pixel( FreemanChain & aPix_chain,
			       std::vector<unsigned int> & aPl2pix,
			       std::vector<unsigned int> & aPix2pl,
			       const FreemanChain & aPlChain );

    /**
     * From the Freeman chain [outer_chain] representing a 4-connected
     * contour, constructs the Freeman chain [inner_chain] that
     * represents its inner 4-connected contour (which lies in its
     * interpixel space). The boolean [ccw] specifies if the inside is
     * to the left (ccw) or to the right (cw).
     * 
     * Note that chain codes going back and forth are @b never considered
     * useless: it means that the chain is always supposed to have its
     * interior to the left (ccw) or right (cw) even at configurations
     * "02", "13", "20", "31".
     * 
     * @param aInner_chain (output) the code of the 4-connected inner
     * border, with starting coordinates that are floored to the closest
     * integer.
     *
     * @param aOuter2inner (output) the mapping associating outer to
     * inner elements as indices in their respective Freeman chain.
     *
     * @param aInner2outer (output) the mapping associating inner to
     * outer elements as indices in their respective Freeman chain.
     *
     * @param aOuter_chain the input code of the 4-connected contour.
     *
     * @param ccw 'true' if the contour is seen counterclockwise with
     * its inside to the left.
     */
    static void innerContour( FreemanChain & aInner_chain,
			      std::vector<unsigned int> & aOuter2inner,
			      std::vector<unsigned int> & aInner2outer,
			      const FreemanChain & aOuterChain,
			      bool ccw = true );

    /**
     * Reads the 4-connected contour [c] so that meaningless back and
     * forth steps are removed. These operations may create one or
     * several 4-connected contours (stored in [clean_cs]), whether
     * these removals cuts the contour in several loops. Because of
     * that, the mappings are more complex.

     * @param aClean_cs (output) the array of cleaned 4-connected contours.
     *
     * @param aC2clean (output) the mapping associating an element to
     * its clean element as a pair (n,i) where n is the index of the
     * cleaned contour and i the indice of the element in this Freeman
     * chain.
     *
     * @param aClean2c (output) the array of mapping associating a
     * clean element to its non-clean element. clean2c[n][j] gives the
     * index of the non-clean element on c corresponding to the clean
     * element of index j in the n-th contour.
     *
     * @param c the input code of the 4-connected contour.
     *
     * @param ccw 'true' if the contour is seen counterclockwise with
     * its inside to the left.
     *
     * @todo This method is not implemented.
     */
    static void cleanContour( std::vector<FreemanChain> & aCleanCs,
			      std::vector< std::pair<unsigned int,unsigned int> > & aC2clean,
			      std::vector< std::vector<unsigned int> > & aClean2c,
			      const FreemanChain & c,
			      bool ccw = true );
    /**
     * Removes outer spikes along a 4-connected contour, meaning steps
     * "02", "13", "20" or "31", which point outside the shape. The
     * inside is given by parameter [ccw]. Note that 4-connected
     * pointel contours should not have any outer spikes, while
     * 4-connected pixel contours should not have any inner spikes.
     *
     * @param aClean_c (output) the cleaned 4-connected contour.
     *
     * @param aC2clean (output) the mapping associating an element to
     * its clean element.
     *
     * @param aClean2c (output) the inverse mapping associating a
     * clean element to its non-clean element. 
     *
     * @param c the input code of the 4-connected contour (should be a loop !).
     *
     * @param ccw 'true' if the contour is seen counterclockwise with
     * its inside to the left.
     *
     * @return 'true' if the contour add an interior, 'false' otherwise.
     */
    static bool cleanOuterSpikes( FreemanChain & aClean_c,
				  std::vector<unsigned int> & aC2clean,
				  std::vector<unsigned int> & aClean2c,
				  const FreemanChain & c,
				  bool ccw = true );

    /**
     * Given a Freeman chain [c] coding a 4-connected pixel loop, computes
     * its subsampling by the transformation:
     * X = ( x - x0 ) div h, 
     * Y = ( y - y0 ) div v.
     *
     * @param aSubc (output) the subsampled Freeman chain code (may
     * contain spikes)
     * 
     * @param aC2subc (output) the mapping associating an element to
     * its subsampled element.
     *
     * @param aSubc2c (output) the inverse mapping associating a
     * subsampled element to its element. More precisely, subc2c[ j ]
     * is the last pointel to be in j.
     *
     * @param c the input chain code.
     *
     * @param h the subsampling along x
     * @param v the subsampling along y
     * @param x0 the x-origin of the frame (X,Y) in (x,y)
     * @param y0 the y-origin of the frame (X,Y) in (x,y)
     *
     * @return 'false' if initial contour was empty or if [subc] is empty,
     * 'true' otherwise.
     */
    static bool subsample( FreemanChain & aSubc,
			   std::vector<unsigned int> & aC2subc,
			   std::vector<unsigned int> & aSubc2c,
			   const FreemanChain & c,
			   unsigned int h, unsigned int v,
			   int x0, int y0 );


  
    /**
     * Return a vector containing all the interger points of the freemanchain.
     *
     * @param fc the FreemanChain
     * @param aVContour (returns) the vector containing all the integer contour points.
     */
    static void getContourPoints(const FreemanChain & fc, std::vector<PointI2> & aVContour); 
    
  
  
  
  
    static void movePointFromFC(PointI2 & aPoint, unsigned int aCode );



    // ----------------------- Standard services ------------------------------
  public:

  

  


    /**
     * Destructor.
     */
    ~FreemanChain();
    
    /**
     * Constructor.
     * @param s the chain code.
     * @param x the x-coordinate of the first point.
     * @param y the y-coordinate of the first point.
     */
    FreemanChain( const std::string & s = "", int x = 0, int y = 0 );


    /**
     * Constructor.
     * @param in any input stream,
     */
    FreemanChain(std::istream & in );


    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    FreemanChain( const FreemanChain & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    FreemanChain & operator=( const FreemanChain & other );


    /**
     * Iterator service.
     * @return an iterator pointing on the first point of the chain.
     */
    FreemanChain::ConstIterator begin() const;

    /**
     * Iterator service.
     * @return an iterator pointing after the last point of the chain.
     */
    FreemanChain::ConstIterator end() const;

    /**
     * @param pos a position in the chain code.
     * @return the code at position [pos].
     */ 
    unsigned int code( unsigned int pos ) const;

    /**
     * @param pos a position in the chain code.
     * @return the next position.
     */ 
    unsigned int next( unsigned int pos ) const;

    /**
     * @param pos a position in the chain code.
     * @return the previous position.
     */ 
    unsigned int previous( unsigned int pos ) const;

    /**
     * @return the length of the Freeman chain code.
     */
    unsigned int size() const;

    /**
     * Computes a bounding box for the Freeman chain code.
     *
     * @param min_x (returns) the minimal x-coordinate.
     * @param min_y (returns) the minimal y-coordinate.
     * @param max_x (returns) the maximal x-coordinate.
     * @param max_y (returns) the maximal y-coordinate.
     */
    void computeBoundingBox( int & min_x, int & min_y, 
			     int & max_x, int & max_y ) const;

    /**
     * Finds a quadrant change in 'this' Freeman chain and returns the
     * position as an iterator. A quadrant change is some
     <code>
     abb..bc
     |
     iterator
     <endcode>
     *
     * The alphabet is possibly re-ordered so that a > b > c.
     *
     * @param A (possibly updated) a Freeman chain alphabet, possibly
     * re-ordered so that a > b > c.
     *
     * @return an iterator on 'this' that points on the first letter b.
     */
    
  
    //BK
    FreemanChain::ConstIterator
    findQuadrantChange( OrderedAlphabet & A ) const;
  
    /**
     * Finds a quadrant change in 'this' Freeman chain and returns the
     * position as an iterator. A quadrant change is some
     <code>
     (abc)*bc...cd
     |
     iterator
     <endcode>
     *
     * This quadrant change also guarantees that is not a place where a
     * convexity change occurs in the combinatorial MLP algorithm.
     *
     * The alphabet is possibly re-ordered so that b > c > d > a.
     *
     * @param A (possibly updated) a Freeman chain alphabet, possibly
     * re-ordered so that b > c > d > a.
     *
     * @return an iterator on 'this' that points on the first letter c.
     */
  
    //BK  
    FreemanChain::ConstIterator
    findQuadrantChange4( OrderedAlphabet & A ) const;

    /**
     * This method takes O(n) operations and works only for Freeman
     * chains whose successive codes are between +1/-1. It determines
     * if the FreemanChain corresponds to a closed contour, and if
     * this is the case, determines how many counterclockwise loops the
     * contour has done. Of course, it the contour has done
     * clockwise loops, then the given number is accordingly
     * negative.
     *
     * @return the number of counterclockwise loops, or '0' is the contour
     * is open or invalid.
     */
    int isClosed() const;
  

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

     public:



    /*
      Draw the object on a LibBoard board
      @param board the output board where the object is drawn.
      @tparam Functor a Functor to specialize the Board style
    */
    template<typename Functor>
    void selfDraw(LibBoard::Board & board ) const;
    


    /*
      Draw the object on a LibBoard board
      @param board the output board where the object is drawn.
    */
    void selfDraw(LibBoard::Board & board ) const
    {
      selfDraw<SelfDrawStyle>(board);
    }
    

    

    // ------------------------- Public Datas ------------------------------

  public:
    /**
     * The chain code.
     */
    std::string chain;

    /**
     * the x-coordinate of the first point.
     */
    int x0;

    /**
     * the y-coordinate of the first point.
     */
    int y0;

    

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    //    FreemanChain();

  private:


    // ------------------------- Internals ------------------------------------
  private:

   /** 
     * Default Style Functor for selfDraw methods
     * 
     * @param aBoard 
     */

    struct SelfDrawStyle
    {
      SelfDrawStyle(LibBoard::Board & aBoard) 
      {
				aBoard.setFillColor(LibBoard::Color::None);
				aBoard.setPenColor(LibBoard::Color::Red);
      }
    };
    

  }; // end of class FreemanChain




  /**
   * Overloads 'operator<<' for displaying objects of class 'FreemanChain'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FreemanChain' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<< ( std::ostream & out, const FreemanChain & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/geometry/2d/FreemanChain.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FreemanChain_h

#undef FreemanChain_RECURSES
#endif // else defined(FreemanChain_RECURSES)
