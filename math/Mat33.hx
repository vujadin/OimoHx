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

import oimohx.math.Mat44;
import oimohx.math.Quat;

/**
 * A 3x3 matrix. This supports rotation, skewing, and scaling transformations.
 * @author saharan
 */
class Mat33 {
	
    public var e00:Float;
    public var e01:Float;
    public var e02:Float;
    public var e10:Float;
    public var e11:Float;
    public var e12:Float;
    public var e20:Float;
    public var e21:Float;
    public var e22:Float;
    
    /**
	 * Constructor.
	 * If the parameters are empty, the matrix will be set to the itentity matrix.
	 * @param	e00
	 * @param	e01
	 * @param	e02
	 * @param	e10
	 * @param	e11
	 * @param	e12
	 * @param	e20
	 * @param	e21
	 * @param	e22
	 */
    public function new(
            e00:Float = 1, e01:Float = 0, e02:Float = 0,
            e10:Float = 0, e11:Float = 1, e12:Float = 0,
            e20:Float = 0, e21:Float = 0, e22:Float = 1) {
        this.e00 = e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e10 = e10;
        this.e11 = e11;
        this.e12 = e12;
        this.e20 = e20;
        this.e21 = e21;
        this.e22 = e22;
    }
    
    /**
	 * Initialize the matrix.
	 * If the parameters are empty, the matrix will be set to the itentity matrix.
	 * @param	e00
	 * @param	e01
	 * @param	e02
	 * @param	e10
	 * @param	e11
	 * @param	e12
	 * @param	e20
	 * @param	e21
	 * @param	e22
	 * @return
	 */
    public function init(
            e00:Float = 1, e01:Float = 0, e02:Float = 0,
            e10:Float = 0, e11:Float = 1, e12:Float = 0,
            e20:Float = 0, e21:Float = 0, e22:Float = 1):Mat33 {
        this.e00 = e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e10 = e10;
        this.e11 = e11;
        this.e12 = e12;
        this.e20 = e20;
        this.e21 = e21;
        this.e22 = e22;
        return this;
    }
    
    /**
	 * this = m1 + m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function add(m1:Mat33, m2:Mat33):Mat33 {
        e00 = m1.e00 + m2.e00;
        e01 = m1.e01 + m2.e01;
        e02 = m1.e02 + m2.e02;
        e10 = m1.e10 + m2.e10;
        e11 = m1.e11 + m2.e11;
        e12 = m1.e12 + m2.e12;
        e20 = m1.e20 + m2.e20;
        e21 = m1.e21 + m2.e21;
        e22 = m1.e22 + m2.e22;
        return this;
    }
    
    /**
	 * this = this + m
	 * @param	m
	 * @return
	 */
    inline public function addEqual(m:Mat33):Mat33 {
        e00 += m.e00;
        e01 += m.e01;
        e02 += m.e02;
        e10 += m.e10;
        e11 += m.e11;
        e12 += m.e12;
        e20 += m.e20;
        e21 += m.e21;
        e22 += m.e22;
        return this;
    }
    
    /**
	 * this = m1 - m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function sub(m1:Mat33, m2:Mat33):Mat33 {
        e00 = m1.e00 - m2.e00;
        e01 = m1.e01 - m2.e01;
        e02 = m1.e02 - m2.e02;
        e10 = m1.e10 - m2.e10;
        e11 = m1.e11 - m2.e11;
        e12 = m1.e12 - m2.e12;
        e20 = m1.e20 - m2.e20;
        e21 = m1.e21 - m2.e21;
        e22 = m1.e22 - m2.e22;
        return this;
    }
    
    /**
	 * this = this - m
	 * @param	m
	 * @return
	 */
    inline public function subEqual(m:Mat33):Mat33 {
        e00 -= m.e00;
        e01 -= m.e01;
        e02 -= m.e02;
        e10 -= m.e10;
        e11 -= m.e11;
        e12 -= m.e12;
        e20 -= m.e20;
        e21 -= m.e21;
        e22 -= m.e22;
        return this;
    }
    
    /**
	 * this = m * s
	 * @param	m
	 * @param	s
	 * @return
	 */
    inline public function scale(m:Mat33, s:Float):Mat33 {
        e00 = m.e00 * s;
        e01 = m.e01 * s;
        e02 = m.e02 * s;
        e10 = m.e10 * s;
        e11 = m.e11 * s;
        e12 = m.e12 * s;
        e20 = m.e20 * s;
        e21 = m.e21 * s;
        e22 = m.e22 * s;
        return this;
    }
    
    
    /**
	 * this = this * s
	 * @param	s
	 * @return
	 */
    inline public function scaleEqual(s:Float):Mat33 {
        e00 *= s;
        e01 *= s;
        e02 *= s;
        e10 *= s;
        e11 *= s;
        e12 *= s;
        e20 *= s;
        e21 *= s;
        e22 *= s;
        return this;
    }
    
    /**
	 * this = m1 * m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function mul(m1:Mat33, m2:Mat33):Mat33 {
        this.e00 = m1.e00 * m2.e00 + m1.e01 * m2.e10 + m1.e02 * m2.e20;
        this.e01 = m1.e00 * m2.e01 + m1.e01 * m2.e11 + m1.e02 * m2.e21;
        this.e02 = m1.e00 * m2.e02 + m1.e01 * m2.e12 + m1.e02 * m2.e22;
        this.e10 = m1.e10 * m2.e00 + m1.e11 * m2.e10 + m1.e12 * m2.e20;
        this.e11 = m1.e10 * m2.e01 + m1.e11 * m2.e11 + m1.e12 * m2.e21;
        this.e12 = m1.e10 * m2.e02 + m1.e11 * m2.e12 + m1.e12 * m2.e22;
        this.e20 = m1.e20 * m2.e00 + m1.e21 * m2.e10 + m1.e22 * m2.e20;
        this.e21 = m1.e20 * m2.e01 + m1.e21 * m2.e11 + m1.e22 * m2.e21;
        this.e22 = m1.e20 * m2.e02 + m1.e21 * m2.e12 + m1.e22 * m2.e22;
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
    inline public function mulScale(m:Mat33, sx:Float, sy:Float, sz:Float, prepend:Bool = false):Mat33 {
        var e00:Float;
        var e01:Float;
        var e02:Float;
        var e10:Float;
        var e11:Float;
        var e12:Float;
        var e20:Float;
        var e21:Float;
        var e22:Float;
        if (prepend) {
            e00 = sx * m.e00;
            e01 = sx * m.e01;
            e02 = sx * m.e02;
            e10 = sy * m.e10;
            e11 = sy * m.e11;
            e12 = sy * m.e12;
            e20 = sz * m.e20;
            e21 = sz * m.e21;
            e22 = sz * m.e22;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
        }
        else {
            e00 = m.e00 * sx;
            e01 = m.e01 * sy;
            e02 = m.e02 * sz;
            e10 = m.e10 * sx;
            e11 = m.e11 * sy;
            e12 = m.e12 * sz;
            e20 = m.e20 * sx;
            e21 = m.e21 * sy;
            e22 = m.e22 * sz;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
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
    inline public function mulRotate(m:Mat33, rad:Float, ax:Float, ay:Float, az:Float, prepend:Bool = false):Mat33 {
        var s:Float = Math.sin(rad);
        var c:Float = Math.cos(rad);
        var c1:Float = 1 - c;
        var r00:Float = ax * ax * c1 + c;
        var r01:Float = ax * ay * c1 - az * s;
        var r02:Float = ax * az * c1 + ay * s;
        var r10:Float = ay * ax * c1 + az * s;
        var r11:Float = ay * ay * c1 + c;
        var r12:Float = ay * az * c1 - ax * s;
        var r20:Float = az * ax * c1 - ay * s;
        var r21:Float = az * ay * c1 + ax * s;
        var r22:Float = az * az * c1 + c;
        var e00:Float;
        var e01:Float;
        var e02:Float;
        var e10:Float;
        var e11:Float;
        var e12:Float;
        var e20:Float;
        var e21:Float;
        var e22:Float;
        if (prepend) {
            e00 = r00 * m.e00 + r01 * m.e10 + r02 * m.e20;
            e01 = r00 * m.e01 + r01 * m.e11 + r02 * m.e21;
            e02 = r00 * m.e02 + r01 * m.e12 + r02 * m.e22;
            e10 = r10 * m.e00 + r11 * m.e10 + r12 * m.e20;
            e11 = r10 * m.e01 + r11 * m.e11 + r12 * m.e21;
            e12 = r10 * m.e02 + r11 * m.e12 + r12 * m.e22;
            e20 = r20 * m.e00 + r21 * m.e10 + r22 * m.e20;
            e21 = r20 * m.e01 + r21 * m.e11 + r22 * m.e21;
            e22 = r20 * m.e02 + r21 * m.e12 + r22 * m.e22;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
        }
        else {
            e00 = m.e00 * r00 + m.e01 * r10 + m.e02 * r20;
            e01 = m.e00 * r01 + m.e01 * r11 + m.e02 * r21;
            e02 = m.e00 * r02 + m.e01 * r12 + m.e02 * r22;
            e10 = m.e10 * r00 + m.e11 * r10 + m.e12 * r20;
            e11 = m.e10 * r01 + m.e11 * r11 + m.e12 * r21;
            e12 = m.e10 * r02 + m.e11 * r12 + m.e12 * r22;
            e20 = m.e20 * r00 + m.e21 * r10 + m.e22 * r20;
            e21 = m.e20 * r01 + m.e21 * r11 + m.e22 * r21;
            e22 = m.e20 * r02 + m.e21 * r12 + m.e22 * r22;
            this.e00 = e00;
            this.e01 = e01;
            this.e02 = e02;
            this.e10 = e10;
            this.e11 = e11;
            this.e12 = e12;
            this.e20 = e20;
            this.e21 = e21;
            this.e22 = e22;
        }
        return this;
    }
    
    /**
	 * Set this matrix to the transposed matrix of m.
	 * @param	m
	 * @return
	 */
    inline public function transpose(m:Mat33):Mat33 {
        var e01:Float = m.e10;
        var e02:Float = m.e20;
        var e10:Float = m.e01;
        var e12:Float = m.e21;
        var e20:Float = m.e02;
        var e21:Float = m.e12;
        e00 = m.e00;
        this.e01 = e01;
        this.e02 = e02;
        this.e10 = e10;
        e11 = m.e11;
        this.e12 = e12;
        this.e20 = e20;
        this.e21 = e21;
        e22 = m.e22;
        return this;
    }
    
    /**
	 * Set this matrix to the rotation matrix of q.
	 * @param	q
	 * @return
	 */
    public function setQuat(q:Quat):Mat33 {
        var x2:Float = 2 * q.x;
        var y2:Float = 2 * q.y;
        var z2:Float = 2 * q.z;
        var xx:Float = q.x * x2;
        var yy:Float = q.y * y2;
        var zz:Float = q.z * z2;
        var xy:Float = q.x * y2;
        var yz:Float = q.y * z2;
        var xz:Float = q.x * z2;
        var sx:Float = q.s * x2;
        var sy:Float = q.s * y2;
        var sz:Float = q.s * z2;
        e00 = 1 - yy - zz;
        e01 = xy - sz;
        e02 = xz + sy;
        e10 = xy + sz;
        e11 = 1 - xx - zz;
        e12 = yz - sx;
        e20 = xz - sy;
        e21 = yz + sx;
        e22 = 1 - xx - yy;
        return this;
    }
    
    /**
	 * this = m ^ -1
	 * @param	m
	 * @return
	 */
    inline public function invert(m:Mat33):Mat33 {
        var det:Float = 
        m.e00 * (m.e11 * m.e22 - m.e21 * m.e12) +
        m.e10 * (m.e21 * m.e02 - m.e01 * m.e22) +
        m.e20 * (m.e01 * m.e12 - m.e11 * m.e02);
        if (det != 0)             det = 1 / det;
        var t00:Float = m.e11 * m.e22 - m.e12 * m.e21;
        var t01:Float = m.e02 * m.e21 - m.e01 * m.e22;
        var t02:Float = m.e01 * m.e12 - m.e02 * m.e11;
        var t10:Float = m.e12 * m.e20 - m.e10 * m.e22;
        var t11:Float = m.e00 * m.e22 - m.e02 * m.e20;
        var t12:Float = m.e02 * m.e10 - m.e00 * m.e12;
        var t20:Float = m.e10 * m.e21 - m.e11 * m.e20;
        var t21:Float = m.e01 * m.e20 - m.e00 * m.e21;
        var t22:Float = m.e00 * m.e11 - m.e01 * m.e10;
        e00 = det * t00;
        e01 = det * t01;
        e02 = det * t02;
        e10 = det * t10;
        e11 = det * t11;
        e12 = det * t12;
        e20 = det * t20;
        e21 = det * t21;
        e22 = det * t22;
        return this;
    }
    
    /**
	 * this = m
	 * @param	m
	 * @return
	 */
    inline public function copy(m:Mat33):Mat33 {
        e00 = m.e00;
        e01 = m.e01;
        e02 = m.e02;
        e10 = m.e10;
        e11 = m.e11;
        e12 = m.e12;
        e20 = m.e20;
        e21 = m.e21;
        e22 = m.e22;
        return this;
    }
    
    /**
	 * this = m
	 * @param	m
	 * @return
	 */
    inline public function copyMat44(m:Mat44):Mat33 {
        e00 = m.e00;
        e01 = m.e01;
        e02 = m.e02;
        e10 = m.e10;
        e11 = m.e11;
        e12 = m.e12;
        e20 = m.e20;
        e21 = m.e21;
        e22 = m.e22;
        return this;
    }
    
    /**
	 * Get the clone of the matrix.
	 * @return
	 */
    public function clone():Mat33{
        return new Mat33(e00, e01, e02, e10, e11, e12, e20, e21, e22);
    }
    
    /**
	 * Get the string of the matrix.
	 * @return
	 */
    public function toString():String {
        var text : String = 
        "Mat33|" + e00 + ", " + e01 + ", " + e02 + "|\n" +
        "     |" + e10 + ", " + e11 + ", " + e12 + "|\n" +
        "     |" + e20 + ", " + e21 + ", " + e22 + "|";
        return text;
    }
}

