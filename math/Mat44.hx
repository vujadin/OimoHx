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

import oimohx.math.Quat;

/**
 * A 4x4 matrix. This supports three-dimentional transformations perfectly.
 * @author saharan
 */
class Mat44 {
	
    public var e00 : Float;
    public var e01 : Float;
    public var e02 : Float;
    public var e03 : Float;
    public var e10 : Float;
    public var e11 : Float;
    public var e12 : Float;
    public var e13 : Float;
    public var e20 : Float;
    public var e21 : Float;
    public var e22 : Float;
    public var e23 : Float;
    public var e30 : Float;
    public var e31 : Float;
    public var e32 : Float;
    public var e33 : Float;
    
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
            e00 : Float = 1, e01 : Float = 0, e02 : Float = 0, e03 : Float = 0,
            e10 : Float = 0, e11 : Float = 1, e12 : Float = 0, e13 : Float = 0,
            e20 : Float = 0, e21 : Float = 0, e22 : Float = 1, e23 : Float = 0,
            e30 : Float = 0, e31 : Float = 0, e32 : Float = 0, e33 : Float = 1) {
        this.e00 = e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e03 = e03;
        this.e10 = e10;
        this.e11 = e11;
        this.e12 = e12;
        this.e13 = e13;
        this.e20 = e20;
        this.e21 = e21;
        this.e22 = e22;
        this.e23 = e23;
        this.e30 = e30;
        this.e31 = e31;
        this.e32 = e32;
        this.e33 = e33;
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
            e00 : Float = 1, e01 : Float = 0, e02 : Float = 0, e03 : Float = 0,
            e10 : Float = 0, e11 : Float = 1, e12 : Float = 0, e13 : Float = 0,
            e20 : Float = 0, e21 : Float = 0, e22 : Float = 1, e23 : Float = 0,
            e30 : Float = 0, e31 : Float = 0, e32 : Float = 0, e33 : Float = 1):Mat44 {
        this.e00 = e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e03 = e03;
        this.e10 = e10;
        this.e11 = e11;
        this.e12 = e12;
        this.e13 = e13;
        this.e20 = e20;
        this.e21 = e21;
        this.e22 = e22;
        this.e23 = e23;
        this.e30 = e30;
        this.e31 = e31;
        this.e32 = e32;
        this.e33 = e33;
        return this;
    }
    
    /**
	 * this = m1 + m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function add(m1:Mat44, m2:Mat44):Mat44 {
        e00 = m1.e00 + m2.e00;
        e01 = m1.e01 + m2.e01;
        e02 = m1.e02 + m2.e02;
        e03 = m1.e03 + m2.e03;
        e10 = m1.e10 + m2.e10;
        e11 = m1.e11 + m2.e11;
        e12 = m1.e12 + m2.e12;
        e13 = m1.e13 + m2.e13;
        e20 = m1.e20 + m2.e20;
        e21 = m1.e21 + m2.e21;
        e22 = m1.e22 + m2.e22;
        e23 = m1.e23 + m2.e23;
        e30 = m1.e30 + m2.e30;
        e31 = m1.e31 + m2.e31;
        e32 = m1.e32 + m2.e32;
        e33 = m1.e33 + m2.e33;
        return this;
    }
    
    /**
	 * this = m1 - m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function sub(m1:Mat44, m2:Mat44):Mat44 {
        e00 = m1.e00 - m2.e00;
        e01 = m1.e01 - m2.e01;
        e02 = m1.e02 - m2.e02;
        e03 = m1.e03 - m2.e03;
        e10 = m1.e10 - m2.e10;
        e11 = m1.e11 - m2.e11;
        e12 = m1.e12 - m2.e12;
        e13 = m1.e13 - m2.e13;
        e20 = m1.e20 - m2.e20;
        e21 = m1.e21 - m2.e21;
        e22 = m1.e22 - m2.e22;
        e23 = m1.e23 - m2.e23;
        e30 = m1.e30 - m2.e30;
        e31 = m1.e31 - m2.e31;
        e32 = m1.e32 - m2.e32;
        e33 = m1.e33 - m2.e33;
        return this;
    }
    
    /**
	 * this = m * s
	 * @param	m
	 * @param	s
	 * @return
	 */
    inline public function scale(m:Mat44, s:Float):Mat44 {
        e00 = m.e00 * s;
        e01 = m.e01 * s;
        e02 = m.e02 * s;
        e03 = m.e03 * s;
        e10 = m.e10 * s;
        e11 = m.e11 * s;
        e12 = m.e12 * s;
        e13 = m.e13 * s;
        e20 = m.e20 * s;
        e21 = m.e21 * s;
        e22 = m.e22 * s;
        e23 = m.e23 * s;
        e30 = m.e30 * s;
        e31 = m.e31 * s;
        e32 = m.e32 * s;
        e33 = m.e33 * s;
        return this;
    }
    
    /**
	 * this = m1 * m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function mul(m1:Mat44, m2:Mat44):Mat44 {
        this.e00 = m1.e00 * m2.e00 + m1.e01 * m2.e10 + m1.e02 * m2.e20 + m1.e03 * m2.e30;
        this.e01 = m1.e00 * m2.e01 + m1.e01 * m2.e11 + m1.e02 * m2.e21 + m1.e03 * m2.e31;
        this.e02 = m1.e00 * m2.e02 + m1.e01 * m2.e12 + m1.e02 * m2.e22 + m1.e03 * m2.e32;
        this.e03 = m1.e00 * m2.e03 + m1.e01 * m2.e13 + m1.e02 * m2.e23 + m1.e03 * m2.e33;
        this.e10 = m1.e10 * m2.e00 + m1.e11 * m2.e10 + m1.e12 * m2.e20 + m1.e13 * m2.e30;
        this.e11 = m1.e10 * m2.e01 + m1.e11 * m2.e11 + m1.e12 * m2.e21 + m1.e13 * m2.e31;
        this.e12 = m1.e10 * m2.e02 + m1.e11 * m2.e12 + m1.e12 * m2.e22 + m1.e13 * m2.e32;
        this.e13 = m1.e10 * m2.e03 + m1.e11 * m2.e13 + m1.e12 * m2.e23 + m1.e13 * m2.e33;
        this.e20 = m1.e20 * m2.e00 + m1.e21 * m2.e10 + m1.e22 * m2.e20 + m1.e23 * m2.e30;
        this.e21 = m1.e20 * m2.e01 + m1.e21 * m2.e11 + m1.e22 * m2.e21 + m1.e23 * m2.e31;
        this.e22 = m1.e20 * m2.e02 + m1.e21 * m2.e12 + m1.e22 * m2.e22 + m1.e23 * m2.e32;
        this.e23 = m1.e20 * m2.e03 + m1.e21 * m2.e13 + m1.e22 * m2.e23 + m1.e23 * m2.e33;
        this.e30 = m1.e30 * m2.e00 + m1.e31 * m2.e10 + m1.e32 * m2.e20 + m1.e33 * m2.e30;
        this.e31 = m1.e30 * m2.e01 + m1.e31 * m2.e11 + m1.e32 * m2.e21 + m1.e33 * m2.e31;
        this.e32 = m1.e30 * m2.e02 + m1.e31 * m2.e12 + m1.e32 * m2.e22 + m1.e33 * m2.e32;
        this.e33 = m1.e30 * m2.e03 + m1.e31 * m2.e13 + m1.e32 * m2.e23 + m1.e33 * m2.e33;
        return this;
    }
    
    /**
		 * Set this matrix to the multiplication of m and scaling matrix.
		 * this = [scaling matrix] * m (prepend == true)
		 * this = m * [scaling matrix] (prepend == false)
		 * @param	m
		 * @param	sx 
		 * @param	sy
		 * @param	sz
		 * @param	prepend
		 * @return
		 */
    inline public function mulScale(m:Mat44, sx:Float, sy:Float, sz:Float, prepend:Bool = false):Mat44 {
        var e00 : Float;
        var e01 : Float;
        var e02 : Float;
        var e03 : Float;
        var e10 : Float;
        var e11 : Float;
        var e12 : Float;
        var e13 : Float;
        var e20 : Float;
        var e21 : Float;
        var e22 : Float;
        var e23 : Float;
        var e30 : Float;
        var e31 : Float;
        var e32 : Float;
        var e33 : Float;
        if (prepend) {
            e00 = sx * m.e00;
            e01 = sx * m.e01;
            e02 = sx * m.e02;
            e03 = sx * m.e03;
            e10 = sy * m.e10;
            e11 = sy * m.e11;
            e12 = sy * m.e12;
            e13 = sy * m.e13;
            e20 = sz * m.e20;
            e21 = sz * m.e21;
            e22 = sz * m.e22;
            e23 = sz * m.e23;
            e30 = m.e30;
            e31 = m.e31;
            e32 = m.e32;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        else {
            e00 = m.e00 * sx;
            e01 = m.e01 * sy;
            e02 = m.e02 * sz;
            e03 = m.e03;
            e10 = m.e10 * sx;
            e11 = m.e11 * sy;
            e12 = m.e12 * sz;
            e13 = m.e13;
            e20 = m.e20 * sx;
            e21 = m.e21 * sy;
            e22 = m.e22 * sz;
            e23 = m.e23;
            e30 = m.e30 * sx;
            e31 = m.e31 * sy;
            e32 = m.e32 * sz;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        return this;
    }
    
    /**
	 * Set this matrix to the multiplication of m and rotation matrix.
	 * this = [rotation matrix] * m (prepend == true)
	 * this = m * [rotation matrix] (prepend == false)
	 * @param	m
	 * @param	rad
	 * @param	ax
	 * @param	ay
	 * @param	az
	 * @param	prepend
	 * @return
	 */
    inline public function mulRotate(m:Mat44, rad:Float, ax:Float, ay:Float, az:Float, prepend:Bool = false):Mat44 {
        var s : Float = Math.sin(rad);
        var c : Float = Math.cos(rad);
        var c1 : Float = 1 - c;
        var r00 : Float = ax * ax * c1 + c;
        var r01 : Float = ax * ay * c1 - az * s;
        var r02 : Float = ax * az * c1 + ay * s;
        var r10 : Float = ay * ax * c1 + az * s;
        var r11 : Float = ay * ay * c1 + c;
        var r12 : Float = ay * az * c1 - ax * s;
        var r20 : Float = az * ax * c1 - ay * s;
        var r21 : Float = az * ay * c1 + ax * s;
        var r22 : Float = az * az * c1 + c;
        var e00 : Float;
        var e01 : Float;
        var e02 : Float;
        var e03 : Float;
        var e10 : Float;
        var e11 : Float;
        var e12 : Float;
        var e13 : Float;
        var e20 : Float;
        var e21 : Float;
        var e22 : Float;
        var e23 : Float;
        var e30 : Float;
        var e31 : Float;
        var e32 : Float;
        var e33 : Float;
        if (prepend) {
            e00 = r00 * m.e00 + r01 * m.e10 + r02 * m.e20;
            e01 = r00 * m.e01 + r01 * m.e11 + r02 * m.e21;
            e02 = r00 * m.e02 + r01 * m.e12 + r02 * m.e22;
            e03 = r00 * m.e03 + r01 * m.e13 + r02 * m.e23;
            e10 = r10 * m.e00 + r11 * m.e10 + r12 * m.e20;
            e11 = r10 * m.e01 + r11 * m.e11 + r12 * m.e21;
            e12 = r10 * m.e02 + r11 * m.e12 + r12 * m.e22;
            e13 = r10 * m.e03 + r11 * m.e13 + r12 * m.e23;
            e20 = r20 * m.e00 + r21 * m.e10 + r22 * m.e20;
            e21 = r20 * m.e01 + r21 * m.e11 + r22 * m.e21;
            e22 = r20 * m.e02 + r21 * m.e12 + r22 * m.e22;
            e23 = r20 * m.e03 + r21 * m.e13 + r22 * m.e23;
            e30 = m.e30;
            e31 = m.e31;
            e32 = m.e32;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        else {
            e00 = m.e00 * r00 + m.e01 * r10 + m.e02 * r20;
            e01 = m.e00 * r01 + m.e01 * r11 + m.e02 * r21;
            e02 = m.e00 * r02 + m.e01 * r12 + m.e02 * r22;
            e03 = m.e03;
            e10 = m.e10 * r00 + m.e11 * r10 + m.e12 * r20;
            e11 = m.e10 * r01 + m.e11 * r11 + m.e12 * r21;
            e12 = m.e10 * r02 + m.e11 * r12 + m.e12 * r22;
            e13 = m.e13;
            e20 = m.e20 * r00 + m.e21 * r10 + m.e22 * r20;
            e21 = m.e20 * r01 + m.e21 * r11 + m.e22 * r21;
            e22 = m.e20 * r02 + m.e21 * r12 + m.e22 * r22;
            e23 = m.e23;
            e30 = m.e30 * r00 + m.e31 * r10 + m.e32 * r20;
            e31 = m.e30 * r01 + m.e31 * r11 + m.e32 * r21;
            e32 = m.e30 * r02 + m.e31 * r12 + m.e32 * r22;
            e33 = m.e33;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e03 = e03;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e13 = e13;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
            this.e23 = e23;
            this.e30 = e30;
            this.e31 = e31;
            this.e32 = e32;
            this.e33 = e33;
        }
        return this;
    }
    
    /**
		 * Set this matrix to the multiplication of m and translation matrix.
		 * this = [translation matrix] * m (prepend == true)
		 * this = m * [translation matrix] (prepend == false)
		 * @param	m
		 * @param	tx
		 * @param	ty
		 * @param	tz
		 * @param	prepend
		 * @return
		 */
    inline public function mulTranslate(m:Mat44, tx:Float, ty:Float, tz:Float, prepend:Bool = false):Mat44 {
        if (prepend) {
            this.e00 = m.e00 + tx * m.e30;
            this.e01 = m.e01 + tx * m.e31;
            this.e02 = m.e02 + tx * m.e32;
            this.e03 = m.e03 + tx * m.e33;
            this.e10 = m.e10 + ty * m.e30;
            this.e11 = m.e11 + ty * m.e31;
            this.e12 = m.e12 + ty * m.e32;
            this.e13 = m.e13 + ty * m.e33;
            this.e20 = m.e20 + tz * m.e30;
            this.e21 = m.e21 + tz * m.e31;
            this.e22 = m.e22 + tz * m.e32;
            this.e23 = m.e23 + tz * m.e33;
            this.e30 = m.e30;
            this.e31 = m.e31;
            this.e32 = m.e32;
            this.e33 = m.e33;
        }
        else {
            this.e00 = m.e00;
            this.e01 = m.e01;
            this.e02 = m.e02;
            this.e03 = m.e00 * tx + m.e01 * ty + m.e02 * tz + m.e03;
            this.e10 = m.e10;
            this.e11 = m.e11;
            this.e12 = m.e12;
            this.e13 = m.e10 * tx + m.e11 * ty + m.e12 * tz + m.e13;
            this.e20 = m.e20;
            this.e21 = m.e21;
            this.e22 = m.e22;
            this.e23 = m.e20 * tx + m.e21 * ty + m.e22 * tz + m.e23;
            this.e30 = m.e30;
            this.e31 = m.e31;
            this.e32 = m.e32;
            this.e33 = m.e30 * tx + m.e31 * ty + m.e32 * tz + m.e33;
        }
        return this;
    }
    
    /**
	 * Set this matrix to the transposed matrix of m.
	 * @param	m
	 * @return
	 */
    inline public function transpose(m:Mat44):Mat44 {
        var e01 : Float = m.e10;
        var e02 : Float = m.e20;
        var e03 : Float = m.e30;
        var e10 : Float = m.e01;
        var e12 : Float = m.e21;
        var e13 : Float = m.e31;
        var e20 : Float = m.e02;
        var e21 : Float = m.e12;
        var e23 : Float = m.e32;
        var e30 : Float = m.e03;
        var e31 : Float = m.e13;
        var e32 : Float = m.e23;
        e00 = m.e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e03 = e03;
        this.e10 = e10;
        e11 = m.e11;
        this.e12 = e12;
        this.e13 = e13;
        this.e20 = e20;
        this.e21 = e21;
        e22 = m.e22;
        this.e23 = e23;
        this.e30 = e30;
        this.e31 = e31;
        this.e32 = e32;
        e33 = m.e33;
        return this;
    }
    
    /**
	 * Set this matrix to the rotation matrix of q.
	 * @param	q
	 * @return
	 */
    inline public function setQuat(q:Quat):Mat44 {
        var x2 : Float = 2 * q.x;
        var y2 : Float = 2 * q.y;
        var z2 : Float = 2 * q.z;
        var xx : Float = q.x * x2;
        var yy : Float = q.y * y2;
        var zz : Float = q.z * z2;
        var xy : Float = q.x * y2;
        var yz : Float = q.y * z2;
        var xz : Float = q.x * z2;
        var sx : Float = q.s * x2;
        var sy : Float = q.s * y2;
        var sz : Float = q.s * z2;
        e00 = 1 - yy - zz;
        e01 = xy - sz;
        e02 = xz + sy;
        e03 = 0;
        e10 = xy + sz;
        e11 = 1 - xx - zz;
        e12 = yz - sx;
        e13 = 0;
        e20 = xz - sy;
        e21 = yz + sx;
        e22 = 1 - xx - yy;
        e23 = 0;
        e30 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 1;
        return this;
    }
    
    /**
	 * this = m ^ -1
	 * @param	m
	 * @return
	 */
    inline public function invert(m:Mat44):Mat44 {
        var e1021_1120 : Float = m.e10 * m.e21 - m.e11 * m.e20;
        var e1022_1220 : Float = m.e10 * m.e22 - m.e12 * m.e20;
        var e1023_1320 : Float = m.e10 * m.e23 - m.e13 * m.e20;
        var e1031_1130 : Float = m.e10 * m.e31 - m.e11 * m.e30;
        var e1032_1230 : Float = m.e10 * m.e32 - m.e12 * m.e30;
        var e1033_1330 : Float = m.e10 * m.e33 - m.e13 * m.e30;
        var e1122_1221 : Float = m.e11 * m.e22 - m.e12 * m.e21;
        var e1123_1321 : Float = m.e11 * m.e23 - m.e13 * m.e21;
        var e1132_1231 : Float = m.e11 * m.e32 - m.e12 * m.e31;
        var e1133_1331 : Float = m.e11 * m.e33 - m.e13 * m.e31;
        var e1220_2022 : Float = m.e12 * m.e20 - m.e20 * m.e22;
        var e1223_1322 : Float = m.e12 * m.e23 - m.e13 * m.e22;
        var e1223_2223 : Float = m.e12 * m.e33 - m.e22 * m.e23;
        var e1233_1332 : Float = m.e12 * m.e33 - m.e13 * m.e32;
        var e2031_2130 : Float = m.e20 * m.e31 - m.e21 * m.e30;
        var e2032_2033 : Float = m.e20 * m.e32 - m.e20 * m.e33;
        var e2032_2230 : Float = m.e20 * m.e32 - m.e22 * m.e30;
        var e2033_2330 : Float = m.e20 * m.e33 - m.e23 * m.e30;
        var e2132_2231 : Float = m.e21 * m.e32 - m.e22 * m.e31;
        var e2133_2331 : Float = m.e21 * m.e33 - m.e23 * m.e31;
        var e2230_2330 : Float = m.e22 * m.e30 - m.e23 * m.e30;
        var e2233_2332 : Float = m.e22 * m.e33 - m.e23 * m.e32;
        var det : Float = 
        m.e00 * (m.e11 * e2233_2332 - m.e12 * e2133_2331 + m.e13 * e2132_2231) +
        m.e01 * (-m.e10 * e2233_2332 - m.e12 * e2032_2033 + m.e13 * e2230_2330) +
        m.e02 * (m.e10 * e2133_2331 - m.e11 * e2033_2330 + m.e13 * e2031_2130) +
        m.e03 * (-m.e10 * e2132_2231 + m.e11 * e2032_2230 - m.e12 * e2031_2130);
        if (det != 0)             det = 1 / det;
        var t00 : Float = m.e11 * e2233_2332 - m.e12 * e2133_2331 + m.e13 * e2132_2231;
        var t01 : Float = -m.e01 * e2233_2332 + m.e02 * e2133_2331 - m.e03 * e2132_2231;
        var t02 : Float = m.e01 * e1233_1332 - m.e02 * e1133_1331 + m.e03 * e1132_1231;
        var t03 : Float = -m.e01 * e1223_2223 + m.e02 * e1123_1321 - m.e03 * e1122_1221;
        var t10 : Float = -m.e10 * e2233_2332 + m.e12 * e2033_2330 - m.e13 * e2032_2230;
        var t11 : Float = m.e00 * e2233_2332 - m.e02 * e2033_2330 + m.e03 * e2032_2230;
        var t12 : Float = -m.e00 * e1233_1332 + m.e02 * e1033_1330 - m.e03 * e1032_1230;
        var t13 : Float = m.e00 * e1223_1322 - m.e02 * e1023_1320 - m.e03 * e1220_2022;
        var t20 : Float = m.e10 * e2133_2331 - m.e11 * e2033_2330 + m.e13 * e2031_2130;
        var t21 : Float = -m.e00 * e2133_2331 + m.e01 * e2033_2330 - m.e03 * e2031_2130;
        var t22 : Float = m.e00 * e1133_1331 - m.e01 * e1033_1330 + m.e03 * e1031_1130;
        var t23 : Float = -m.e00 * e1123_1321 + m.e01 * e1023_1320 - m.e03 * e1021_1120;
        var t30 : Float = -m.e10 * e2132_2231 + m.e11 * e2032_2230 - m.e12 * e2031_2130;
        var t31 : Float = m.e00 * e2132_2231 - m.e01 * e2032_2230 + m.e02 * e2031_2130;
        var t32 : Float = -m.e00 * e1132_1231 + m.e01 * e1032_1230 - m.e02 * e1031_1130;
        var t33 : Float = m.e00 * e1122_1221 - m.e01 * e1022_1220 + m.e02 * e1021_1120;
        e00 = det * t00;
        e01 = det * t01;
        e02 = det * t02;
        e03 = det * t03;
        e10 = det * t10;
        e11 = det * t11;
        e12 = det * t12;
        e13 = det * t13;
        e20 = det * t20;
        e21 = det * t21;
        e22 = det * t22;
        e23 = det * t23;
        e30 = det * t30;
        e31 = det * t31;
        e32 = det * t32;
        e33 = det * t33;
        return this;
    }
    
    /**
	 * Set the matrix to right-handed view matrix.
	 * @param	eyeX
	 * @param	eyeY
	 * @param	eyeZ
	 * @param	atX 
	 * @param	atY
	 * @param	atZ
	 * @param	upX
	 * @param	upY
	 * @param	upZ
	 * @return
	 */
    inline public function lookAt(
            eyeX:Float, eyeY:Float, eyeZ:Float,
            atX:Float, atY:Float, atZ:Float,
            upX:Float, upY:Float, upZ:Float):Mat44{
        var zx : Float = eyeX - atX;
        var zy : Float = eyeY - atY;
        var zz : Float = eyeZ - atZ;
        var tmp : Float = 1 / Math.sqrt(zx * zx + zy * zy + zz * zz);
        zx *= tmp;
        zy *= tmp;
        zz *= tmp;
        var xx : Float = upY * zz - upZ * zy;
        var xy : Float = upZ * zx - upX * zz;
        var xz : Float = upX * zy - upY * zx;
        tmp = 1 / Math.sqrt(xx * xx + xy * xy + xz * xz);
        xx *= tmp;
        xy *= tmp;
        xz *= tmp;
        var yx : Float = zy * xz - zz * xy;
        var yy : Float = zz * xx - zx * xz;
        var yz : Float = zx * xy - zy * xx;
        e00 = xx;
        e01 = xy;
        e02 = xz;
        e03 = -(xx * eyeX + xy * eyeY + xz * eyeZ);
        e10 = yx;
        e11 = yy;
        e12 = yz;
        e13 = -(yx * eyeX + yy * eyeY + yz * eyeZ);
        e20 = zx;
        e21 = zy;
        e22 = zz;
        e23 = -(zx * eyeX + zy * eyeY + zz * eyeZ);
        e30 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 1;
        return this;
    }
    
    /**
	 * Set the matrix to the right-handed perspective projection matrix.
	 * @param	fovY
	 * @param	aspect
	 * @param	near
	 * @param	far
	 * @return
	 */
    inline public function perspective(fovY:Float, aspect:Float, near:Float, far:Float):Mat44 {
        var h : Float = 1 / Math.tan(fovY * 0.5);
        var fnf : Float = far / (near - far);
        e00 = h / aspect;
        e01 = 0;
        e02 = 0;
        e03 = 0;
        e10 = 0;
        e11 = h;
        e12 = 0;
        e13 = 0;
        e20 = 0;
        e21 = 0;
        e22 = fnf;
        e23 = near * fnf;
        e30 = 0;
        e31 = 0;
        e32 = -1;
        e33 = 0;
        return this;
    }
    
    /**
	 * Set the matrix to the right-handed orthogonal projection matrix.
	 * @param	width
	 * @param	height
	 * @param	near
	 * @param	far
	 * @return
	 */
    inline public function ortho(width:Float, height:Float, near:Float, far:Float):Mat44 {
        var nf : Float = 1 / (near - far);
        e00 = 2 / width;
        e01 = 0;
        e02 = 0;
        e03 = 0;
        e10 = 0;
        e11 = 2 / height;
        e12 = 0;
        e13 = 0;
        e20 = 0;
        e21 = 0;
        e22 = nf;
        e23 = near * nf;
        e30 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 0;
        return this;
    }
    
    /**
	 * this = m
	 * @param	m
	 * @return
	 */
    inline public function copy(m:Mat44):Mat44 {
        e00 = m.e00;
        e01 = m.e01;
        e02 = m.e02;
        e03 = m.e03;
        e10 = m.e10;
        e11 = m.e11;
        e12 = m.e12;
        e13 = m.e13;
        e20 = m.e20;
        e21 = m.e21;
        e22 = m.e22;
        e23 = m.e23;
        e30 = m.e30;
        e31 = m.e31;
        e32 = m.e32;
        e33 = m.e33;
        return this;
    }
    
    /**
	 * this = m
	 * @param	m
	 * @return
	 */
    inline public function copyMat33(m:Mat33):Mat44 {
        e00 = m.e00;
        e01 = m.e01;
        e02 = m.e02;
        e03 = 0;
        e10 = m.e10;
        e11 = m.e11;
        e12 = m.e12;
        e13 = 0;
        e20 = m.e20;
        e21 = m.e21;
        e22 = m.e22;
        e23 = 0;
        e30 = 0;
        e31 = 0;
        e32 = 0;
        e33 = 1;
        return this;
    }
    
    /**
	 * Get the clone of the matrix.
	 * @return
	 */
    public function clone():Mat44 {
        return new Mat44(
        e00, e01, e02, e03, 
        e10, e11, e12, e13, 
        e20, e21, e22, e23, 
        e30, e31, e32, e33
        );
    }
    
    /**
	 * Get the string of the matrix.
	 * @return
	 */
    public function toString():String {
        var text : String = 
        "Mat44|" + e00 + ", " + e01 + ", " + e02 + ", " + e03 + "|\n" +
        "     |" + e10 + ", " + e11 + ", " + e12 + ", " + e13 + "|\n" +
        "     |" + e20 + ", " + e21 + ", " + e22 + ", " + e23 + "|\n" +
        "     |" + e30 + ", " + e31 + ", " + e32 + ", " + e33 + "|\n";
        return text;
    }
	
}
