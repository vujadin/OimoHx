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
package oimohx.physics.constraint.joint;

import oimohx.physics.constraint.joint.LimitMotor;

import oimohx.math.Mat33;
import oimohx.math.Quat;
import oimohx.math.Vec3;
import oimohx.physics.constraint.joint.base.LinearConstraint;
import oimohx.physics.constraint.joint.base.Rotational3Constraint;
import oimohx.physics.dynamics.RigidBody;

/**
 * A hinge joint allows only for relative rotation of rigid bodies along the axis.
 * @author saharan
 */
class HingeJoint extends Joint {
    /**
	 * The axis in the first body's coordinate system.
	 */
    public var localAxis1:Vec3;
    
    /**
	 * The axis in the second body's coordinate system.
	 */
    public var localAxis2:Vec3;
    
    /**
	 * The rotational limit and motor information of the joint.
	 */
    public var limitMotor:LimitMotor;
    
    private var localAxis1X:Float;
    private var localAxis1Y:Float;
    private var localAxis1Z:Float;
    
    private var localAxis2X:Float;
    private var localAxis2Y:Float;
    private var localAxis2Z:Float;
    
    private var localAngAxis1X:Float;
    private var localAngAxis1Y:Float;
    private var localAngAxis1Z:Float;
    
    private var localAngAxis2X:Float;
    private var localAngAxis2Y:Float;
    private var localAngAxis2Z:Float;
    
    private var lc:LinearConstraint;
    private var r3:Rotational3Constraint;
    
    private var nor:Vec3;
    private var tan:Vec3;
    private var bin:Vec3;
	
    
    public function new(config:JointConfig, lowerAngleLimit:Float = 1, upperAngleLimit:Float = 0) {
        super(config);
        localAxis1 = new Vec3().normalize(config.localAxis1);
        localAxis2 = new Vec3().normalize(config.localAxis2);
        
        var len:Float;
        localAxis1X = localAxis1.x;
        localAxis1Y = localAxis1.y;
        localAxis1Z = localAxis1.z;
        localAngAxis1X = localAxis1Y * localAxis1X - localAxis1Z * localAxis1Z;
        localAngAxis1Y = -localAxis1Z * localAxis1Y - localAxis1X * localAxis1X;
        localAngAxis1Z = localAxis1X * localAxis1Z + localAxis1Y * localAxis1Y;
        len = 1 / Math.sqrt(localAngAxis1X * localAngAxis1X + localAngAxis1Y * localAngAxis1Y + localAngAxis1Z * localAngAxis1Z);
        localAngAxis1X *= len;
        localAngAxis1Y *= len;
        localAngAxis1Z *= len;
        
        localAxis2X = localAxis2.x;
        localAxis2Y = localAxis2.y;
        localAxis2Z = localAxis2.z;
        
        // make angle axis 2
        var arc:Mat33 = new Mat33().setQuat(new Quat().arc(localAxis1, localAxis2));
        localAngAxis2X = localAngAxis1X * arc.elements[0] + localAngAxis1Y * arc.elements[1] + localAngAxis1Z * arc.elements[2];
        localAngAxis2Y = localAngAxis1X * arc.elements[3] + localAngAxis1Y * arc.elements[4] + localAngAxis1Z * arc.elements[5];
        localAngAxis2Z = localAngAxis1X * arc.elements[6] + localAngAxis1Y * arc.elements[7] + localAngAxis1Z * arc.elements[7];
        
        type = Joint.JOINT_HINGE;
        
        nor = new Vec3();
        tan = new Vec3();
        bin = new Vec3();
        limitMotor = new LimitMotor(nor, false);
        limitMotor.lowerLimit = lowerAngleLimit;
        limitMotor.upperLimit = upperAngleLimit;
        
        lc = new LinearConstraint(this);
        r3 = new Rotational3Constraint(this, limitMotor, new LimitMotor(tan, true), new LimitMotor(bin, true));
    }
    
    /**
	 * @inheritDoc
	 */
    override public function preSolve(timeStep:Float, invTimeStep:Float) {
        var tmpM:Mat33;
        var tmp1X:Float;
        var tmp1Y:Float;
        var tmp1Z:Float;
        
        updateAnchorPoints();
        
        tmpM = body1.rotation;
        var axis1X:Float = localAxis1X * tmpM.elements[0] + localAxis1Y * tmpM.elements[1] + localAxis1Z * tmpM.elements[2];
        var axis1Y:Float = localAxis1X * tmpM.elements[3] + localAxis1Y * tmpM.elements[4] + localAxis1Z * tmpM.elements[5];
        var axis1Z:Float = localAxis1X * tmpM.elements[6] + localAxis1Y * tmpM.elements[7] + localAxis1Z * tmpM.elements[8];
        var angAxis1X:Float = localAngAxis1X * tmpM.elements[0] + localAngAxis1Y * tmpM.elements[1] + localAngAxis1Z * tmpM.elements[2];
        var angAxis1Y:Float = localAngAxis1X * tmpM.elements[3] + localAngAxis1Y * tmpM.elements[4] + localAngAxis1Z * tmpM.elements[5];
        var angAxis1Z:Float = localAngAxis1X * tmpM.elements[6] + localAngAxis1Y * tmpM.elements[7] + localAngAxis1Z * tmpM.elements[8];
        tmpM = body2.rotation;
        var axis2X:Float = localAxis2X * tmpM.elements[0] + localAxis2Y * tmpM.elements[1] + localAxis2Z * tmpM.elements[2];
        var axis2Y:Float = localAxis2X * tmpM.elements[3] + localAxis2Y * tmpM.elements[4] + localAxis2Z * tmpM.elements[5];
        var axis2Z:Float = localAxis2X * tmpM.elements[6] + localAxis2Y * tmpM.elements[7] + localAxis2Z * tmpM.elements[8];
        var angAxis2X:Float = localAngAxis2X * tmpM.elements[0] + localAngAxis2Y * tmpM.elements[1] + localAngAxis2Z * tmpM.elements[2];
        var angAxis2Y:Float = localAngAxis2X * tmpM.elements[3] + localAngAxis2Y * tmpM.elements[4] + localAngAxis2Z * tmpM.elements[5];
        var angAxis2Z:Float = localAngAxis2X * tmpM.elements[6] + localAngAxis2Y * tmpM.elements[7] + localAngAxis2Z * tmpM.elements[8];
        var nx:Float = axis1X * body2.inverseMass + axis2X * body1.inverseMass;
        var ny:Float = axis1Y * body2.inverseMass + axis2Y * body1.inverseMass;
        var nz:Float = axis1Z * body2.inverseMass + axis2Z * body1.inverseMass;
        tmp1X = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (tmp1X > 0) {
			tmp1X = 1 / tmp1X;
		}
        nx *= tmp1X;
        ny *= tmp1X;
        nz *= tmp1X;
        var tx:Float = ny * nx - nz * nz;
        var ty:Float = -nz * ny - nx * nx;
        var tz:Float = nx * nz + ny * ny;
        tmp1X = 1 / Math.sqrt(tx * tx + ty * ty + tz * tz);
        tx *= tmp1X;
        ty *= tmp1X;
        tz *= tmp1X;
        var bx:Float = ny * tz - nz * ty;
        var by:Float = nz * tx - nx * tz;
        var bz:Float = nx * ty - ny * tx;
        
        nor.init(nx, ny, nz);
        tan.init(tx, ty, tz);
        bin.init(bx, by, bz);
        
        // ----------------------------------------------
        //            calculate hinge angle
        // ----------------------------------------------
        
        if (
            nx * (angAxis1Y * angAxis2Z - angAxis1Z * angAxis2Y) +
            ny * (angAxis1Z * angAxis2X - angAxis1X * angAxis2Z) +
            nz * (angAxis1X * angAxis2Y - angAxis1Y * angAxis2X) < 0) { // cross product
            limitMotor.angle = -acosClamp(angAxis1X * angAxis2X + angAxis1Y * angAxis2Y + angAxis1Z * angAxis2Z);
        }
        else {
            limitMotor.angle = acosClamp(angAxis1X * angAxis2X + angAxis1Y * angAxis2Y + angAxis1Z * angAxis2Z);
        }  // angular error  
        
        tmp1X = axis1Y * axis2Z - axis1Z * axis2Y;
        tmp1Y = axis1Z * axis2X - axis1X * axis2Z;
        tmp1Z = axis1X * axis2Y - axis1Y * axis2X;
        
        r3.limitMotor2.angle = tx * tmp1X + ty * tmp1Y + tz * tmp1Z;
        r3.limitMotor3.angle = bx * tmp1X + by * tmp1Y + bz * tmp1Z;
        
        r3.preSolve(timeStep, invTimeStep);
        lc.preSolve(timeStep, invTimeStep);
    }
    
    /**
	 * @inheritDoc
	 */
    override public function solve() {
        r3.solve();
        lc.solve();
    }
    
    /**
	 * @inheritDoc
	 */
    override public function postSolve() {
        
    }
    
    private function acosClamp(cos:Float):Float {
        if (cos > 1) {
            return 0;
		}
        else if (cos < -1) {
            return Math.PI;
		}
        else {
			return Math.acos(cos);
		}
    }
	
}
