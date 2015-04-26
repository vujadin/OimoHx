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


import oimohx.math.Mat33;
import oimohx.math.Quat;
import oimohx.math.Vec3;
import oimohx.physics.constraint.joint.base.AngularConstraint;
import oimohx.physics.constraint.joint.base.Rotational3Constraint;
import oimohx.physics.constraint.joint.base.RotationalConstraint;
import oimohx.physics.constraint.joint.base.Translational3Constraint;
import oimohx.physics.constraint.joint.base.TranslationalConstraint;
import oimohx.physics.dynamics.RigidBody;

/**
 * A prismatic joint allows only for relative translation of rigid bodies along the axis.
 * @author saharan
 */
class PrismaticJoint extends Joint {
    /**
	 * The axis in the first body's coordinate system.
	 */
    public var localAxis1:Vec3;
    
    /**
	 * The axis in the second body's coordinate system.
	 */
    public var localAxis2:Vec3;
    
    /**
	 * The translational limit and motor information of the joint.
	 */
    public var limitMotor:LimitMotor;
    
    private var localAxis1X:Float;
    private var localAxis1Y:Float;
    private var localAxis1Z:Float;
    
    private var localAxis2X:Float;
    private var localAxis2Y:Float;
    private var localAxis2Z:Float;
    
    private var t3:Translational3Constraint;
    private var ac:AngularConstraint;
    
    private var nor:Vec3;
    private var tan:Vec3;
    private var bin:Vec3;
    
	
    public function new(config:JointConfig, lowerTranslation:Float, upperTranslation:Float) {
        super(config);
        localAxis1 = new Vec3().normalize(config.localAxis1);
        localAxis2 = new Vec3().normalize(config.localAxis2);
        
        localAxis1X = localAxis1.x;
        localAxis1Y = localAxis1.y;
        localAxis1Z = localAxis1.z;
        localAxis2X = localAxis2.x;
        localAxis2Y = localAxis2.y;
        localAxis2Z = localAxis2.z;
        
        type = Joint.JOINT_PRISMATIC;
        
        nor = new Vec3();
        tan = new Vec3();
        bin = new Vec3();
        ac = new AngularConstraint(this, new Quat().arc(localAxis1, localAxis2));
        
        limitMotor = new LimitMotor(nor, true);
        limitMotor.lowerLimit = lowerTranslation;
        limitMotor.upperLimit = upperTranslation;
        t3 = new Translational3Constraint(this, limitMotor, new LimitMotor(tan, true), new LimitMotor(bin, true));
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
        tmpM = body2.rotation;
        var axis2X:Float = localAxis2X * tmpM.elements[0] + localAxis2Y * tmpM.elements[1] + localAxis2Z * tmpM.elements[2];
        var axis2Y:Float = localAxis2X * tmpM.elements[3] + localAxis2Y * tmpM.elements[4] + localAxis2Z * tmpM.elements[5];
        var axis2Z:Float = localAxis2X * tmpM.elements[6] + localAxis2Y * tmpM.elements[7] + localAxis2Z * tmpM.elements[8];
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
        
        ac.preSolve(timeStep, invTimeStep);
        t3.preSolve(timeStep, invTimeStep);
    }
    
    /**
	 * @inheritDoc
	 */
    override public function solve() {
        ac.solve();
        t3.solve();
    }
    
    /**
	 * @inheritDoc
	 */
    override public function postSolve() {
        
    }
	
}
