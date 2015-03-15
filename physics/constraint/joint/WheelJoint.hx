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
import oimohx.physics.constraint.joint.base.LinearConstraint;
import oimohx.physics.constraint.joint.base.Rotational3Constraint;
import oimohx.physics.constraint.joint.base.Translational3Constraint;
import oimohx.physics.dynamics.RigidBody;
/**
	 * A wheel joint allows for relative rotation between two rigid bodies along two axes.
	 * The wheel joint also allows for relative translation for the suspension.
	 */
class WheelJoint extends Joint
{
    /**
		 * The first axis in local coordinate system.
		 */
    public var localAxis1 : Vec3;
    
    /**
		 * The second axis in local coordinate system.
		 */
    public var localAxis2 : Vec3;
    
    /**
		 * The first rotational limit and motor information of the joint.
		 */
    public var rotationalLimitMotor1 : LimitMotor;
    
    /**
		 * The second rotational limit and motor information of the joint.
		 */
    public var rotationalLimitMotor2 : LimitMotor;
    
    /**
		 * The translational limit and motor information of the joint.
		 */
    public var translationalLimitMotor : LimitMotor;
    
    private var nor : Vec3;
    private var tan : Vec3;
    private var bin : Vec3;
    
    private var localAxis1X : Float;
    private var localAxis1Y : Float;
    private var localAxis1Z : Float;
    
    private var localAxis2X : Float;
    private var localAxis2Y : Float;
    private var localAxis2Z : Float;
    
    private var localAngAxis1X : Float;
    private var localAngAxis1Y : Float;
    private var localAngAxis1Z : Float;
    
    private var localAngAxis2X : Float;
    private var localAngAxis2Y : Float;
    private var localAngAxis2Z : Float;
    
    private var r3 : Rotational3Constraint;
    private var t3 : Translational3Constraint;
    
    public function new(config : JointConfig)
    {
        super(config);
        localAxis1 = new Vec3().normalize(config.localAxis1);
        localAxis2 = new Vec3().normalize(config.localAxis2);
        
        var len : Float;
        localAxis1X = localAxis1.x;
        localAxis1Y = localAxis1.y;
        localAxis1Z = localAxis1.z;
        localAxis2X = localAxis2.x;
        localAxis2Y = localAxis2.y;
        localAxis2Z = localAxis2.z;
        
        var dot : Float = localAxis1X * localAxis2X + localAxis1Y * localAxis2Y + localAxis1Z * localAxis2Z;
        if (dot > -1 && dot < 1) {
            localAngAxis1X = localAxis2X - dot * localAxis1X;
            localAngAxis1Y = localAxis2Y - dot * localAxis1Y;
            localAngAxis1Z = localAxis2Z - dot * localAxis1Z;
            localAngAxis2X = localAxis1X - dot * localAxis2X;
            localAngAxis2Y = localAxis1Y - dot * localAxis2Y;
            localAngAxis2Z = localAxis1Z - dot * localAxis2Z;
            len = 1 / Math.sqrt(localAngAxis1X * localAngAxis1X + localAngAxis1Y * localAngAxis1Y + localAngAxis1Z * localAngAxis1Z);
            localAngAxis1X *= len;
            localAngAxis1Y *= len;
            localAngAxis1Z *= len;
            len = 1 / Math.sqrt(localAngAxis2X * localAngAxis2X + localAngAxis2Y * localAngAxis2Y + localAngAxis2Z * localAngAxis2Z);
            localAngAxis2X *= len;
            localAngAxis2Y *= len;
            localAngAxis2Z *= len;
        }
        else {
            localAngAxis1X = localAxis1Y * localAxis1X - localAxis1Z * localAxis1Z;
            localAngAxis1Y = -localAxis1Z * localAxis1Y - localAxis1X * localAxis1X;
            localAngAxis1Z = localAxis1X * localAxis1Z + localAxis1Y * localAxis1Y;
            len = 1 / Math.sqrt(localAngAxis1X * localAngAxis1X + localAngAxis1Y * localAngAxis1Y + localAngAxis1Z * localAngAxis1Z);
            localAngAxis1X *= len;
            localAngAxis1Y *= len;
            localAngAxis1Z *= len;
            
            // make angle axis 2
            var arc : Mat33 = new Mat33().setQuat(new Quat().arc(localAxis1, localAxis2));
            localAngAxis2X = localAngAxis1X * arc.e00 + localAngAxis1Y * arc.e01 + localAngAxis1Z * arc.e02;
            localAngAxis2Y = localAngAxis1X * arc.e10 + localAngAxis1Y * arc.e11 + localAngAxis1Z * arc.e12;
            localAngAxis2Z = localAngAxis1X * arc.e20 + localAngAxis1Y * arc.e21 + localAngAxis1Z * arc.e22;
        }
        
        type = Joint.JOINT_WHEEL;
        
        nor = new Vec3();
        tan = new Vec3();
        bin = new Vec3();
        
        translationalLimitMotor = new LimitMotor(tan, true);
        translationalLimitMotor.frequency = 8;
        translationalLimitMotor.dampingRatio = 1;
        rotationalLimitMotor1 = new LimitMotor(tan, false);
        rotationalLimitMotor2 = new LimitMotor(bin, false);
        
        t3 = new Translational3Constraint(this, new LimitMotor(nor, true), translationalLimitMotor, new LimitMotor(bin, true));
        t3.weight = 1;
        r3 = new Rotational3Constraint(this, new LimitMotor(nor, true), rotationalLimitMotor1, rotationalLimitMotor2);
    }
    
    /**
		 * @inheritDoc
		 */
    override public function preSolve(timeStep : Float, invTimeStep : Float) : Void{
        var tmpM : Mat33;
        var tmp1X : Float;
        var tmp1Y : Float;
        var tmp1Z : Float;
        
        updateAnchorPoints();
        
        tmpM = body1.rotation;
        var x1 : Float = localAxis1X * tmpM.e00 + localAxis1Y * tmpM.e01 + localAxis1Z * tmpM.e02;
        var y1 : Float = localAxis1X * tmpM.e10 + localAxis1Y * tmpM.e11 + localAxis1Z * tmpM.e12;
        var z1 : Float = localAxis1X * tmpM.e20 + localAxis1Y * tmpM.e21 + localAxis1Z * tmpM.e22;
        var angAxis1X : Float = localAngAxis1X * tmpM.e00 + localAngAxis1Y * tmpM.e01 + localAngAxis1Z * tmpM.e02;
        var angAxis1Y : Float = localAngAxis1X * tmpM.e10 + localAngAxis1Y * tmpM.e11 + localAngAxis1Z * tmpM.e12;
        var angAxis1Z : Float = localAngAxis1X * tmpM.e20 + localAngAxis1Y * tmpM.e21 + localAngAxis1Z * tmpM.e22;
        tmpM = body2.rotation;
        var x2 : Float = localAxis2X * tmpM.e00 + localAxis2Y * tmpM.e01 + localAxis2Z * tmpM.e02;
        var y2 : Float = localAxis2X * tmpM.e10 + localAxis2Y * tmpM.e11 + localAxis2Z * tmpM.e12;
        var z2 : Float = localAxis2X * tmpM.e20 + localAxis2Y * tmpM.e21 + localAxis2Z * tmpM.e22;
        var angAxis2X : Float = localAngAxis2X * tmpM.e00 + localAngAxis2Y * tmpM.e01 + localAngAxis2Z * tmpM.e02;
        var angAxis2Y : Float = localAngAxis2X * tmpM.e10 + localAngAxis2Y * tmpM.e11 + localAngAxis2Z * tmpM.e12;
        var angAxis2Z : Float = localAngAxis2X * tmpM.e20 + localAngAxis2Y * tmpM.e21 + localAngAxis2Z * tmpM.e22;
        r3.limitMotor1.angle = x1 * x2 + y1 * y2 + z1 * z2;
        
        if (
            x1 * (angAxis1Y * z2 - angAxis1Z * y2) +
            y1 * (angAxis1Z * x2 - angAxis1X * z2) +
            z1 * (angAxis1X * y2 - angAxis1Y * x2) < 0) {	// cross product
            rotationalLimitMotor1.angle = -acosClamp(angAxis1X * x2 + angAxis1Y * y2 + angAxis1Z * z2);
        }
        else {
            rotationalLimitMotor1.angle = acosClamp(angAxis1X * x2 + angAxis1Y * y2 + angAxis1Z * z2);
        }
        
        if (
            x2 * (angAxis2Y * z1 - angAxis2Z * y1) +
            y2 * (angAxis2Z * x1 - angAxis2X * z1) +
            z2 * (angAxis2X * y1 - angAxis2Y * x1) < 0) {	// cross product
            rotationalLimitMotor2.angle = acosClamp(angAxis2X * x1 + angAxis2Y * y1 + angAxis2Z * z1);
        }
        else {
            rotationalLimitMotor2.angle = -acosClamp(angAxis2X * x1 + angAxis2Y * y1 + angAxis2Z * z1);
        }
        
        var nx : Float = y2 * z1 - z2 * y1;
        var ny : Float = z2 * x1 - x2 * z1;
        var nz : Float = x2 * y1 - y2 * x1;
        
        tmp1X = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (tmp1X > 0)             tmp1X = 1 / tmp1X;
        nx *= tmp1X;
        ny *= tmp1X;
        nz *= tmp1X;
        
        var tx : Float = ny * z2 - nz * y2;
        var ty : Float = nz * x2 - nx * z2;
        var tz : Float = nx * y2 - ny * x2;
        
        tmp1X = Math.sqrt(tx * tx + ty * ty + tz * tz);
        if (tmp1X > 0)             tmp1X = 1 / tmp1X;
        tx *= tmp1X;
        ty *= tmp1X;
        tz *= tmp1X;
        
        var bx : Float = y1 * nz - z1 * ny;
        var by : Float = z1 * nx - x1 * nz;
        var bz : Float = x1 * ny - y1 * nx;
        
        tmp1X = Math.sqrt(bx * bx + by * by + bz * bz);
        if (tmp1X > 0)             tmp1X = 1 / tmp1X;
        bx *= tmp1X;
        by *= tmp1X;
        bz *= tmp1X;
        
        nor.init(nx, ny, nz);
        tan.init(tx, ty, tz);
        bin.init(bx, by, bz);
        
        r3.preSolve(timeStep, invTimeStep);
        t3.preSolve(timeStep, invTimeStep);
    }
    
    /**
		 * @inheritDoc
		 */
    override public function solve() : Void{
        r3.solve();
        t3.solve();
    }
    
    /**
		 * @inheritDoc
		 */
    override public function postSolve() : Void{
        
    }
    
    private function acosClamp(cos : Float) : Float{
        if (cos > 1)             return 0
        else if (cos < -1)             return Math.PI
        else return Math.acos(cos);
    }
}

