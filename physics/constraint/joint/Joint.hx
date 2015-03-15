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

import oimohx.physics.constraint.joint.JointLink;

import oimohx.math.Mat33;
import oimohx.math.Vec3;
import oimohx.physics.constraint.Constraint;
import oimohx.physics.dynamics.RigidBody;
import oimohx.physics.dynamics.World;
/**
	 * Joints are used to constrain the motion between two rigid bodies.
	 * @author saharan
	 */
class Joint extends Constraint
{
    /**
		 * Distance joint
		 */
    public static inline var JOINT_DISTANCE : Int = 0x1;
    
    /**
		 * Ball-and-socket joint
		 */
    public static inline var JOINT_BALL_AND_SOCKET : Int = 0x2;
    
    /**
		 * Hinge joint
		 */
    public static inline var JOINT_HINGE : Int = 0x3;
    
    /**
		 * Wheel joint
		 */
    public static inline var JOINT_WHEEL : Int = 0x4;
    
    /**
		 * Slider joint
		 */
    public static inline var JOINT_SLIDER : Int = 0x5;
    
    /**
		 * Prismatic joint
		 */
    public static inline var JOINT_PRISMATIC : Int = 0x6;
    
    private var b1Link : JointLink;
    private var b2Link : JointLink;
    
    /**
		 * The type of the joint.
		 */
    public var type : Int;
    
    /**
		 * Whether allow collision between connected rigid bodies or not.
		 */
    public var allowCollision : Bool;
    
    /**
		 * The anchor point on the first rigid body in local coordinate system.
		 */
    public var localAnchorPoint1 : Vec3;
    
    /**
		 * The anchor point on the second rigid body in local coordinate system.
		 */
    public var localAnchorPoint2 : Vec3;
    
    /**
		 * The anchor point on the first rigid body in world coordinate system relative to the body's origin.
		 */
    public var relativeAnchorPoint1 : Vec3;
    
    /**
		 * The anchor point on the second rigid body in world coordinate system relative to the body's origin.
		 */
    public var relativeAnchorPoint2 : Vec3;
    
    /**
		 * The anchor point on the first rigid body in world coordinate system.
		 */
    public var anchorPoint1 : Vec3;
    
    /**
		 * The anchor point on the second rigid body in world coordinate system.
		 */
    public var anchorPoint2 : Vec3;
    
    /**
		 * The previous joint in the world.
		 */
    public var prev : Joint;
    
    /**
		 * The next joint in the world.
		 */
    public var next : Joint;
    
    public function new(config : JointConfig)
    {
        super();
        body1 = config.body1;
        body2 = config.body2;
        localAnchorPoint1 = new Vec3().copy(config.localAnchorPoint1);
        localAnchorPoint2 = new Vec3().copy(config.localAnchorPoint2);
        relativeAnchorPoint1 = new Vec3();
        relativeAnchorPoint2 = new Vec3();
        anchorPoint1 = new Vec3();
        anchorPoint2 = new Vec3();
        allowCollision = config.allowCollision;
        b1Link = new JointLink(this);
        b2Link = new JointLink(this);
    }
    
    /**
		 * Update all the anchor points.
		 */
    public function updateAnchorPoints() : Void{
        var p1 : Vec3 = body1.position;
        var p2 : Vec3 = body2.position;
        var r1 : Mat33 = body1.rotation;
        var r2 : Mat33 = body2.rotation;
        var l1x : Float = localAnchorPoint1.x;
        var l1y : Float = localAnchorPoint1.y;
        var l1z : Float = localAnchorPoint1.z;
        var l2x : Float = localAnchorPoint2.x;
        var l2y : Float = localAnchorPoint2.y;
        var l2z : Float = localAnchorPoint2.z;
        var r1x : Float = l1x * r1.e00 + l1y * r1.e01 + l1z * r1.e02;
        var r1y : Float = l1x * r1.e10 + l1y * r1.e11 + l1z * r1.e12;
        var r1z : Float = l1x * r1.e20 + l1y * r1.e21 + l1z * r1.e22;
        var r2x : Float = l2x * r2.e00 + l2y * r2.e01 + l2z * r2.e02;
        var r2y : Float = l2x * r2.e10 + l2y * r2.e11 + l2z * r2.e12;
        var r2z : Float = l2x * r2.e20 + l2y * r2.e21 + l2z * r2.e22;
        relativeAnchorPoint1.x = r1x;
        relativeAnchorPoint1.y = r1y;
        relativeAnchorPoint1.z = r1z;
        relativeAnchorPoint2.x = r2x;
        relativeAnchorPoint2.y = r2y;
        relativeAnchorPoint2.z = r2z;
        var p1x : Float = r1x + p1.x;
        var p1y : Float = r1y + p1.y;
        var p1z : Float = r1z + p1.z;
        var p2x : Float = r2x + p2.x;
        var p2y : Float = r2y + p2.y;
        var p2z : Float = r2z + p2.z;
        anchorPoint1.x = p1x;
        anchorPoint1.y = p1y;
        anchorPoint1.z = p1z;
        anchorPoint2.x = p2x;
        anchorPoint2.y = p2y;
        anchorPoint2.z = p2z;
    }
    
    /**
		 * Attach the joint to the bodies.
		 */
    public function attach() : Void{
        b1Link.body = body2;
        b2Link.body = body1;
        
        if (body1.jointLink != null)             (b1Link.next = body1.jointLink).prev = b1Link
        else b1Link.next = null;
        body1.jointLink = b1Link;
        body1.numJoints++;
        
        if (body2.jointLink != null)             (b2Link.next = body2.jointLink).prev = b2Link
        else b2Link.next = null;
        body2.jointLink = b2Link;
        body2.numJoints++;
    }
    
    /**
		 * Attach the joint from the bodies.
		 */
    public function detach() : Void{
        var prev : JointLink = b1Link.prev;
        var next : JointLink = b1Link.next;
        if (prev != null)             prev.next = next;
        if (next != null)             next.prev = prev;
        if (body1.jointLink == b1Link)             body1.jointLink = next;
        b1Link.prev = null;
        b1Link.next = null;
        b1Link.body = null;
        body1.numJoints--;
        
        prev = b2Link.prev;
        next = b2Link.next;
        if (prev != null)             prev.next = next;
        if (next != null)             next.prev = prev;
        if (body2.jointLink == b2Link)             body2.jointLink = next;
        b2Link.prev = null;
        b2Link.next = null;
        b2Link.body = null;
        body2.numJoints--;
        
        b1Link.body = null;
        b2Link.body = null;
    }
    
    /**
		 * Awake the bodies.
		 */
    public function awake() : Void{
        body1.awake();
        body2.awake();
    }
    
    /**
		 * @inheritDoc
		 */
    override public function preSolve(timeStep : Float, invTimeStep : Float) : Void{
        super.preSolve(timeStep, invTimeStep);
    }
    
    /**
		 * @inheritDoc
		 */
    override public function solve() : Void{
        super.solve();
    }
    
    /**
		 * @inheritDoc
		 */
    override public function postSolve() : Void{
        super.postSolve();
    }
}

