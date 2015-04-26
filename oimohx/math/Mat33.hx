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
import com.babylonhx.utils.typedarray.Float32Array;

/**
 * A 3x3 matrix. This supports rotation, skewing, and scaling transformations.
 * @author saharan
 */
class Mat33 {
	
    #if html5
	public var elements:Float32Array = new Float32Array(9);
	#else
	public var elements:Array<Float> = [];	
	#end
	    
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
        var te = this.elements;
        te[0] = e00; te[1] = e01; te[2] = e02;
        te[3] = e10; te[4] = e11; te[5] = e12;
        te[6] = e20; te[7] = e21; te[8] = e22;
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
        var te = this.elements;
        te[0] = e00; te[1] = e01; te[2] = e02;
        te[3] = e10; te[4] = e11; te[5] = e12;
        te[6] = e20; te[7] = e21; te[8] = e22;
        return this;
    }
    
    /**
	 * this = m1 + m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function add(m1:Mat33, m2:Mat33):Mat33 {
        var te = this.elements;
		var tem1 = m1.elements;
		var tem2 = m2.elements;
        te[0] = tem1[0] + tem2[0]; te[1] = tem1[1] + tem2[1]; te[2] = tem1[2] + tem2[2];
        te[3] = tem1[3] + tem2[3]; te[4] = tem1[4] + tem2[4]; te[5] = tem1[5] + tem2[5];
        te[6] = tem1[6] + tem2[6]; te[7] = tem1[7] + tem2[7]; te[8] = tem1[8] + tem2[8];
        return this;
    }
    
    /**
	 * this = this + m
	 * @param	m
	 * @return
	 */
    inline public function addEqual(m:Mat33):Mat33 {
        var te = this.elements;
		var tem = m.elements;
        te[0] += tem[0]; te[1] += tem[1]; te[2] += tem[2];
        te[3] += tem[3]; te[4] += tem[4]; te[5] += tem[5];
        te[6] += tem[6]; te[7] += tem[7]; te[8] += tem[8];
        return this;
    }
    
    /**
	 * this = m1 - m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function sub(m1:Mat33, m2:Mat33):Mat33 {
        var te = this.elements;
		var tem1 = m1.elements;
		var tem2 = m2.elements;
        te[0] = tem1[0] - tem2[0]; te[1] = tem1[1] - tem2[1]; te[2] = tem1[2] - tem2[2];
        te[3] = tem1[3] - tem2[3]; te[4] = tem1[4] - tem2[4]; te[5] = tem1[5] - tem2[5];
        te[6] = tem1[6] - tem2[6]; te[7] = tem1[7] - tem2[7]; te[8] = tem1[8] - tem2[8];
        return this;
    }
    
    /**
	 * this = this - m
	 * @param	m
	 * @return
	 */
    inline public function subEqual(m:Mat33):Mat33 {
        var te = this.elements;
		var tem = m.elements;
        te[0] -= tem[0]; te[1] -= tem[1]; te[2] -= tem[2];
        te[3] -= tem[3]; te[4] -= tem[4]; te[5] -= tem[5];
        te[6] -= tem[6]; te[7] -= tem[7]; te[8] -= tem[8];
        return this;
    }
    
    /**
	 * this = m * s
	 * @param	m
	 * @param	s
	 * @return
	 */
    inline public function scale(m:Mat33, s:Float):Mat33 {
        var te = this.elements;
		var tm = m.elements;
        te[0] = tm[0] * s; te[1] = tm[1] * s; te[2] = tm[2] * s;
        te[3] = tm[3] * s; te[4] = tm[4] * s; te[5] = tm[5] * s;
        te[6] = tm[6] * s; te[7] = tm[7] * s; te[8] = tm[8] * s;
        return this;
    }
    
    
    /**
	 * this = this * s
	 * @param	s
	 * @return
	 */
    inline public function scaleEqual(s:Float):Mat33 {
        var te = this.elements;
        te[0] *= s; te[1] *= s; te[2] *= s;
        te[3] *= s; te[4] *= s; te[5] *= s;
        te[6] *= s; te[7] *= s; te[8] *= s;
        return this;
    }
    
    /**
	 * this = m1 * m2
	 * @param	m1
	 * @param	m2
	 * @return
	 */
    inline public function mul(m1:Mat33, m2:Mat33):Mat33 {
        var te = this.elements;
		var tm1 = m1.elements;
		var tm2 = m2.elements;
        var a0 = tm1[0], a3 = tm1[3], a6 = tm1[6],
        a1 = tm1[1], a4 = tm1[4], a7 = tm1[7],
        a2 = tm1[2], a5 = tm1[5], a8 = tm1[8],
        b0 = tm2[0], b3 = tm2[3], b6 = tm2[6],
        b1 = tm2[1], b4 = tm2[4], b7 = tm2[7],
        b2 = tm2[2], b5 = tm2[5], b8 = tm2[8];
        te[0] = a0*b0 + a1*b3 + a2*b6;
        te[1] = a0*b1 + a1*b4 + a2*b7;
        te[2] = a0*b2 + a1*b5 + a2*b8;
        te[3] = a3*b0 + a4*b3 + a5*b6;
        te[4] = a3*b1 + a4*b4 + a5*b7;
        te[5] = a3*b2 + a4*b5 + a5*b8;
        te[6] = a6*b0 + a7*b3 + a8*b6;
        te[7] = a6*b1 + a7*b4 + a8*b7;
        te[8] = a6*b2 + a7*b5 + a8*b8;
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
        var te = this.elements;
		var tm = m.elements;
        if (prepend) {	
            te[0] = sx * tm[0]; te[1] = sx * tm[1]; te[2] = sx * tm[2];
            te[3] = sy * tm[3]; te[4] = sy * tm[4]; te[5] = sy * tm[5];
            te[6] = sz * tm[6]; te[7] = sz * tm[7]; te[8] = sz * tm[8];
        } else {	
            te[0] = tm[0] * sx; te[1] = tm[1] * sy; te[2] = tm[2] * sz;
            te[3] = tm[3] * sx; te[4] = tm[4] * sy; te[5] = tm[5] * sz;
            te[6] = tm[6] * sx; te[7] = tm[7] * sy; te[8] = tm[8] * sz;
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
		
        var tm = m.elements;
		
        var a0 = tm[0], a3 = tm[3], a6 = tm[6];
        var a1 = tm[1], a4 = tm[4], a7 = tm[7];
        var a2 = tm[2], a5 = tm[5], a8 = tm[8];
		
        var te = this.elements;
        
        if (prepend) {	
            te[0] = r00 * a0 + r01 * a3 + r02 * a6;
            te[1] = r00 * a1 + r01 * a4 + r02 * a7;
            te[2] = r00 * a2 + r01 * a5 + r02 * a8;
            te[3] = r10 * a0 + r11 * a3 + r12 * a6;
            te[4] = r10 * a1 + r11 * a4 + r12 * a7;
            te[5] = r10 * a2 + r11 * a5 + r12 * a8;
            te[6] = r20 * a0 + r21 * a3 + r22 * a6;
            te[7] = r20 * a1 + r21 * a4 + r22 * a7;
            te[8] = r20 * a2 + r21 * a5 + r22 * a8;
        } else {	
            te[0] = a0 * r00 + a1 * r10 + a2 * r20;
            te[1] = a0 * r01 + a1 * r11 + a2 * r21;
            te[2] = a0 * r02 + a1 * r12 + a2 * r22;
            te[3] = a3 * r00 + a4 * r10 + a5 * r20;
            te[4] = a3 * r01 + a4 * r11 + a5 * r21;
            te[5] = a3 * r02 + a4 * r12 + a5 * r22;
            te[6] = a6 * r00 + a7 * r10 + a8 * r20;
            te[7] = a6 * r01 + a7 * r11 + a8 * r21;
            te[8] = a6 * r02 + a7 * r12 + a8 * r22;
        }
        return this;
    }
    
    /**
	 * Set this matrix to the transposed matrix of m.
	 * @param	m
	 * @return
	 */
    inline public function transpose(m:Mat33):Mat33 {
        var te = this.elements;
		var tm = m.elements;
        te[0] = tm[0]; te[1] = tm[3]; te[2] = tm[6];
        te[3] = tm[1]; te[4] = tm[4]; te[5] = tm[7];
        te[6] = tm[2]; te[7] = tm[5]; te[8] = tm[8];
        return this;
    }
    
    /**
	 * Set this matrix to the rotation matrix of q.
	 * @param	q
	 * @return
	 */
    public function setQuat(q:Quat):Mat33 {
        var te = this.elements;
        var x2 = 2 * q.x, y2 = 2 * q.y, z2 = 2 * q.z;
        var xx = q.x * x2, yy = q.y * y2, zz = q.z * z2;
        var xy = q.x * y2, yz = q.y * z2, xz = q.x * z2;
        var sx = q.s * x2, sy = q.s * y2, sz = q.s * z2;
        
        te[0] = 1 - yy - zz;
        te[1] = xy - sz;
        te[2] = xz + sy;
        te[3] = xy + sz;
        te[4] = 1 - xx - zz;
        te[5] = yz - sx;
        te[6] = xz - sy;
        te[7] = yz + sx;
        te[8] = 1 - xx - yy;
        return this;
    }
    
    /**
	 * this = m ^ -1
	 * @param	m
	 * @return
	 */
    inline public function invert(m:Mat33):Mat33 {
        var te = this.elements;
		var tm = m.elements;
        var a0 = tm[0], a3 = tm[3], a6 = tm[6];
        var a1 = tm[1], a4 = tm[4], a7 = tm[7];
        var a2 = tm[2], a5 = tm[5], a8 = tm[8];
        var b01 = a4 * a8 - a7 * a5;
        var b11 = a7 * a2 - a1 * a8;
        var b21 = a1 * a5 - a4 * a2;
        var dt = a0 * (b01) + a3 * (b11) + a6 * (b21);
		
        if (dt != 0) {
			dt = 1.0 / dt;
		}
		
        te[0] = dt * b01;
        te[1] = dt * b11;
        te[2] = dt * b21;
        te[3] = dt * (a5 * a6 - a3 * a8);
        te[4] = dt * (a0 * a8 - a2 * a6);
        te[5] = dt * (a2 * a3 - a0 * a5);
        te[6] = dt * (a3 * a7 - a4 * a6);
        te[7] = dt * (a1 * a6 - a0 * a7);
        te[8] = dt * (a0 * a4 - a1 * a3);
        return this;
    }
    
    /**
	 * this = m
	 * @param	m
	 * @return
	 */
    inline public function copy(m:Mat33):Mat33 {
        var te = this.elements;
		var tem = m.elements;
        te[0] = tem[0]; te[1] = tem[1]; te[2] = tem[2];
        te[3] = tem[3]; te[4] = tem[4]; te[5] = tem[5];
        te[6] = tem[6]; te[7] = tem[7]; te[8] = tem[8];
        return this;
    }
        
    /**
	 * Get the clone of the matrix.
	 * @return
	 */
    public function clone():Mat33 {
        var te = this.elements;
		
        return new Mat33(
            te[0], te[1], te[2],
            te[3], te[4], te[5],
            te[6], te[7], te[8]
        );
    }
	
}
