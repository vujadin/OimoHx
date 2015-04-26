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
import oimohx.math.Vec3;
import oimohx.physics.constraint.joint.base.TranslationalConstraint;
import oimohx.physics.dynamics.RigidBody;
/**
	 * A distance joint limits the distance between two anchor points on rigid bodies.
	 * @author saharan
	 */
class DistanceJoint extends Joint
{
    /**
		 * The limit and motor information of the joint.
		 */
    public var limitMotor : LimitMotor;
    
    private var t : TranslationalConstraint;
    private var normal : Vec3;
    
    public function new(config : JointConfig, minDistance : Float, maxDistance : Float)
    {
        super(config);
        type = Joint.JOINT_DISTANCE;
        
        normal = new Vec3();
        limitMotor = new LimitMotor(normal, true);
        limitMotor.lowerLimit = minDistance;
        limitMotor.upperLimit = maxDistance;
        t = new TranslationalConstraint(this, limitMotor);
    }
    
    /**
		 * @inheritDoc
		 */
    override public function preSolve(timeStep : Float, invTimeStep : Float) : Void{
        updateAnchorPoints();
        var nx : Float = anchorPoint2.x - anchorPoint1.x;
        var ny : Float = anchorPoint2.y - anchorPoint1.y;
        var nz : Float = anchorPoint2.z - anchorPoint1.z;
        var len : Float = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 0)             len = 1 / len;
        normal.init(nx * len, ny * len, nz * len);
        t.preSolve(timeStep, invTimeStep);
    }
    
    /**
		 * @inheritDoc
		 */
    override public function solve() : Void{
        t.solve();
    }
    
    /**
		 * @inheritDoc
		 */
    override public function postSolve() : Void{
        
    }
}

