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
package oimohx.math;

import oimohx.ds.Float32Array;
import haxe.ds.Vector;
import oimohx.math.Quat;

/**
 * A 4x4 matrix. This supports three-dimentional transformations perfectly.
 * @author saharan
 */
class Mat44 {
	
	public var elements:Float32Array = new Float32Array(16);

    /**
	 * Constructor.
	 * If the parameters are empty, the matrix will be set to the identity matrix.
	 * @param	e00
	 * @param	e01
	 * @param	e02
	 * @param	e03
	 * @param	e10
	 * @param	e11
	 * @param	e12
	 * @param	e13
	 * @param	e20
	 * @param	e21
	 * @param	e22
	 * @param	e23
	 * @param	e30
	 * @param	e31
	 * @param	e32
	 * @param	e33
	 */
    public function new(
            e00:Float = 1, e01:Float = 0, e02:Float = 0, e03:Float = 0,
            e10:Float = 0, e11:Float = 1, e12:Float = 0, e13:Float = 0,
            e20:Float = 0, e21:Float = 0, e22:Float = 1, e23:Float = 0,
            e30:Float = 0, e31:Float = 0, e32:Float = 0, e33:Float = 1) {
        var te = this.elements;
		
        te[0] = e00; te[4] = e01; te[8] = e02; te[12] = e03;
        te[1] = e10; te[5] = e11; te[9] = e12; te[13] = e13;
        te[2] = e20; te[6] = e21; te[10] = e22; te[14] = e23;
        te[3] = e30; te[7] = e31; te[11] = e32; te[15] = e33;
    }
    
    /**
	 * Initialize the matrix.
	 * If the parameters are empty, the matrix will be set to the identity matrix.
	 * @param	e00
	 * @param	e01
	 * @param	e02
	 * @param	e03
	 * @param	e10
	 * @param	e11
	 * @param	e12
	 * @param	e13
	 * @param	e20
	 * @param	e21
	 * @param	e22
	 * @param	e23
	 * @param	e30
	 * @param	e31
	 * @param	e32
	 * @param	e33
	 * @return
	 */
    public function init(
            e00:Float = 1, e01:Float = 0, e02:Float = 0, e03:Float = 0,
            e10:Float = 0, e11:Float = 1, e12:Float = 0, e13:Float = 0,
            e20:Float = 0, e21:Float = 0, e22:Float = 1, e23:Float = 0,
            e30:Float = 0, e31:Float = 0, e32:Float = 0, e33:Float = 1):Mat44 {
        var te = this.elements;
		
        te[0] = e00; te[4] = e01; te[8] = e02; te[12] = e03;
        te[1] = e10; te[5] = e11; te[9] = e12; te[13] = e13;
        te[2] = e20; te[6] = e21; te[10] = e22; te[14] = e23;
        te[3] = e30; te[7] = e31; te[11] = e32; te[15] = e33;
		
        return this;
    }
	
}
