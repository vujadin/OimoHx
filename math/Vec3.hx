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


/**
 * A 3D vector. This supports three-dimansional vector operations.
 * @author saharan
 */
class Vec3 { 
	
    public var x:Float;
    public var y:Float;
    public var z:Float;
    
    /**
	 * Constructor.
	 * If the parameters are empty, the vector will be set to the zero vector.
	 * @param	x
	 * @param	y
	 * @param	z
	 */
    public function new(x:Float = 0, y:Float = 0, z:Float = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    /**
	 * Initialize the vector.
	 * If the parameters are empty, the vector will be set to the zero vector.
	 * @param	x
	 * @param	y
	 * @param	z
	 * @return
	 */
    public function init(x:Float = 0, y:Float = 0, z:Float = 0):Vec3 {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }
    
    /**
	 * this = v1 + v2
	 * @param	v1
	 * @param	v2
	 * @return
	 */
    inline public function add(v1:Vec3, v2:Vec3):Vec3 {
        x = v1.x + v2.x;
        y = v1.y + v2.y;
        z = v1.z + v2.z;
        return this;
    }
    
    /**
	 * this = this + v
	 * @param	v
	 * @return
	 */
    inline public function addEqual(v:Vec3):Vec3 {
        x += v.x;
        y += v.y;
        z += v.z;
        return this;
    }
	
	inline public function addTime(v:Vec3, t:Float):Vec3 {
        this.x += v.x * t;
        this.y += v.y * t;
        this.z += v.z * t;
        return this;
    }
    
    /**
	 * this = v1 - v2
	 * @param	v1
	 * @param	v2
	 * @return
	 */
    inline public function sub(v1:Vec3, v2:Vec3):Vec3 {
        x = v1.x - v2.x;
        y = v1.y - v2.y;
        z = v1.z - v2.z;
        return this;
    }
    
    /**
	 * this = this - v
	 * @param	v
	 * @return
	 */
    inline public function subEqual(v:Vec3):Vec3 {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return this;
    }
    
    /**
	 * this = v * s
	 * @param	v
	 * @param	s
	 * @return
	 */
    inline public function scale(v:Vec3, s:Float):Vec3 {
        x = v.x * s;
        y = v.y * s;
        z = v.z * s;
        return this;
    }
    
    /**
	 * this = this * s
	 * @param	s
	 * @return
	 */
    inline public function scaleEqual(s:Float):Vec3 {
        x *= s;
        y *= s;
        z *= s;
        return this;
    }
    
    /**
	 * Get the dot production of this vector and v.
	 * @param	v
	 * @return
	 */
    inline public function dot(v:Vec3):Float {
        return x * v.x + y * v.y + z * v.z;
    }
    
    /**
	 * Set this vector to the cross product of v1 and v2.
	 * @param	v1
	 * @param	v2
	 * @return
	 */
    inline public function cross(v1:Vec3, v2:Vec3):Vec3 {
        this.x = v1.y * v2.z - v1.z * v2.y;
        this.y = v1.z * v2.x - v1.x * v2.z;
        this.z = v1.x * v2.y - v1.y * v2.x;
        return this;
    }
	
	inline public function mul(o:Vec3, v:Vec3, m:Mat33):Vec3 {
        var te = m.elements;
        this.x = o.x + v.x * te[0] + v.y * te[1] + v.z * te[2];
        this.y = o.y + v.x * te[3] + v.y * te[4] + v.z * te[5];
        this.z = o.z + v.x * te[6] + v.y * te[7] + v.z * te[8];
        return this;
    }
    
    /**
	 * this = m * v
	 * @param	m
	 * @param	v
	 * @return
	 */
    inline public function mulMat(m:Mat33, v:Vec3):Vec3 {
        this.x = m.elements[0] * v.x + m.elements[1] * v.y + m.elements[2] * v.z;
        this.y = m.elements[3] * v.x + m.elements[4] * v.y + m.elements[5] * v.z;
        this.z = m.elements[6] * v.x + m.elements[7] * v.y + m.elements[8] * v.z;
        return this;
    }
    
    /**
	 * Set this vector to the normalized vector of v.
	 * @param	v
	 * @return
	 */
    inline public function normalize(v:Vec3):Vec3 {
        var length:Float = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (length > 0) {
			length = 1 / length;
		}
        x = v.x * length;
        y = v.y * length;
        z = v.z * length;
        return this;
    }
    
    /**
	 * this = -v
	 * @param	v
	 * @return
	 */
    inline public function invert(v:Vec3):Vec3 {
        x = -v.x;
        y = -v.y;
        z = -v.z;
        return this;
    }
	
	inline public function testZero():Bool {
        if (this.x != 0 || this.y != 0 || this.z != 0) {
			return true;
		}
        else {
			return false;
		}
    }
	
    inline public function testDiff(v:Vec3):Bool {
        if (this.x != v.x || this.y != v.y || this.z != v.z) {
			return true;
		}
        else {
			return false;
		}
    }
    
    /**
	 * Get the length of the vector.
	 * @return
	 */
    inline public function length():Float {
        return Math.sqrt(x * x + y * y + z * z);
    }
    
    /**
	 * this = v
	 * @param	v
	 * @return
	 */
    inline public function copy(v:Vec3):Vec3 {
        x = v.x;
        y = v.y;
        z = v.z;
        return this;
    }
    
    /**
	 * Get the clone of the vector.
	 * @return
	 */
    public function clone():Vec3 {
        return new Vec3(x, y, z);
    }
    
    /**
	 * Get the string of the vector.
	 * @return
	 */
    public function toString():String {
        return "Vec3[" + x + ", " + y + ", " + z + "]";
    }
	
}

