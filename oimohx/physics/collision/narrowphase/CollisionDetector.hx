/* Copyright (c) 2012-2013 EL-EMENT saharan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation  * files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy,  * modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package oimohx.physics.collision.narrowphase;

import oimohx.physics.collision.shape.Shape;
import oimohx.physics.constraint.contact.ContactManifold;
/**
	 * A collision detector detects collisions between two shapes.
	 * @author saharan
	 */
class CollisionDetector
{
    /**
		 * Whether the collision detector flips two shapes in detectCollision or not.
		 */
    public var flip : Bool;
    
    public function new()
    {
        
    }
    
    /**
		 * Detect collision between two shapes.
		 * @param	shape1
		 * @param	shape2
		 * @param	manifold
		 */
    public function detectCollision(shape1 : Shape, shape2 : Shape, manifold : ContactManifold) : Void{
        //throw new Error("Inheritance error.");
    }
}

