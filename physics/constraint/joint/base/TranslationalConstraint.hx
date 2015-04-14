package oimohx.physics.constraint.joint.base;

import oimohx.math.Mat33;
import oimohx.math.Vec3;
import oimohx.physics.constraint.joint.Joint;
import oimohx.physics.constraint.joint.LimitMotor;
import oimohx.physics.dynamics.RigidBody;

/**
 * A translational constraint for various joints.
 * @author saharan
 */
class TranslationalConstraint {
	
    private var limitMotor:LimitMotor;
    private var b1:RigidBody;
    private var b2:RigidBody;
    private var p1:Vec3;
    private var p2:Vec3;
    private var r1:Vec3;
    private var r2:Vec3;
    private var l1:Vec3;
    private var l2:Vec3;
    private var a1:Vec3;
    private var a2:Vec3;
    private var i1:Mat33;
    private var i2:Mat33;
    
    private var cfm:Float;
    
    private var m1:Float;
    private var m2:Float;
    private var i1e00:Float;
    private var i1e01:Float;
    private var i1e02:Float;
    private var i1e10:Float;
    private var i1e11:Float;
    private var i1e12:Float;
    private var i1e20:Float;
    private var i1e21:Float;
    private var i1e22:Float;
    private var i2e00:Float;
    private var i2e01:Float;
    private var i2e02:Float;
    private var i2e10:Float;
    private var i2e11:Float;
    private var i2e12:Float;
    private var i2e20:Float;
    private var i2e21:Float;
    private var i2e22:Float;
    private var motorDenom:Float;
    private var invMotorDenom:Float;
    private var invDenom:Float;
    private var ax:Float;
    private var ay:Float;
    private var az:Float;
    private var r1x:Float;
    private var r1y:Float;
    private var r1z:Float;
    private var r2x:Float;
    private var r2y:Float;
    private var r2z:Float;
    private var t1x:Float;
    private var t1y:Float;
    private var t1z:Float;
    private var t2x:Float;
    private var t2y:Float;
    private var t2z:Float;
    private var l1x:Float;
    private var l1y:Float;
    private var l1z:Float;
    private var l2x:Float;
    private var l2y:Float;
    private var l2z:Float;
    private var a1x:Float;
    private var a1y:Float;
    private var a1z:Float;
    private var a2x:Float;
    private var a2y:Float;
    private var a2z:Float;
    
    private var lowerLimit:Float;
    private var upperLimit:Float;
    private var limitVelocity:Float;
    private var limitImpulse:Float;
    private var limitState:Int;  // -1: at lower, 0: locked, 1: at upper, 2: free  
    
    private var enableMotor:Bool;
    private var motorSpeed:Float;
    private var maxMotorForce:Float;
    private var maxMotorImpulse:Float;
    private var motorImpulse:Float;
	
    
    public function new(joint:Joint, limitMotor:LimitMotor) {
        this.limitMotor = limitMotor;
        b1 = joint.body1;
        b2 = joint.body2;
        p1 = joint.anchorPoint1;
        p2 = joint.anchorPoint2;
        r1 = joint.relativeAnchorPoint1;
        r2 = joint.relativeAnchorPoint2;
        l1 = b1.linearVelocity;
        l2 = b2.linearVelocity;
        a1 = b1.angularVelocity;
        a2 = b2.angularVelocity;
        i1 = b1.inverseInertia;
        i2 = b2.inverseInertia;
        limitImpulse = 0;
        motorImpulse = 0;
    }
    
    public function preSolve(timeStep:Float, invTimeStep:Float) {
        ax = limitMotor.axis.x;
        ay = limitMotor.axis.y;
        az = limitMotor.axis.z;
        lowerLimit = limitMotor.lowerLimit;
        upperLimit = limitMotor.upperLimit;
        motorSpeed = limitMotor.motorSpeed;
        maxMotorForce = limitMotor.maxMotorForce;
        enableMotor = maxMotorForce > 0;
        m1 = b1.inverseMass;
        m2 = b2.inverseMass;
        i1e00 = i1.e00;
        i1e01 = i1.e01;
        i1e02 = i1.e02;
        i1e10 = i1.e10;
        i1e11 = i1.e11;
        i1e12 = i1.e12;
        i1e20 = i1.e20;
        i1e21 = i1.e21;
        i1e22 = i1.e22;
        i2e00 = i2.e00;
        i2e01 = i2.e01;
        i2e02 = i2.e02;
        i2e10 = i2.e10;
        i2e11 = i2.e11;
        i2e12 = i2.e12;
        i2e20 = i2.e20;
        i2e21 = i2.e21;
        i2e22 = i2.e22;
        
        var dx:Float = p2.x - p1.x;
        var dy:Float = p2.y - p1.y;
        var dz:Float = p2.z - p1.z;
        var d:Float = dx * ax + dy * ay + dz * az;
        
        var frequency:Float = limitMotor.frequency;
        var enableSpring:Bool = frequency > 0;
        var enableLimit:Bool = lowerLimit <= upperLimit;
        
        if (enableSpring && d > 20 || d < -20) {
            enableSpring = false;
        }
        
        if (enableLimit) {
            if (lowerLimit == upperLimit) {
                if (limitState != 0) {
                    limitState = 0;
                    limitImpulse = 0;
                }
                limitVelocity = lowerLimit - d;
                if (!enableSpring) {
                    d = lowerLimit;
				}
            }
            else if (d < lowerLimit) {
                if (limitState != -1) {
                    limitState = -1;
                    limitImpulse = 0;
                }
                limitVelocity = lowerLimit - d;
                if (!enableSpring) {
                    d = lowerLimit;
				}
            }
            else if (d > upperLimit) {
                if (limitState != 1) {
                    limitState = 1;
                    limitImpulse = 0;
                }
                limitVelocity = upperLimit - d;
                if (!enableSpring) {
					d = upperLimit;
				}
            }
            else {
                limitState = 2;
                limitImpulse = 0;
                limitVelocity = 0;
            }
            if (!enableSpring) {
                if (limitVelocity > 0.005) {
					limitVelocity -= 0.005;
				}
                else if (limitVelocity < -0.005) {
                    limitVelocity += 0.005;
				}
                else {
					limitVelocity = 0;
				}
            }
        }
        else {
            limitState = 2;
            limitImpulse = 0;
        }
        
        if (enableMotor && (limitState != 0 || enableSpring)) {
            maxMotorImpulse = maxMotorForce * timeStep;
        }
        else {
            motorImpulse = 0;
            maxMotorImpulse = 0;
        }
        
        var rdx:Float = d * ax;
        var rdy:Float = d * ay;
        var rdz:Float = d * az;
        var w1:Float = m1 / (m1 + m2);
        var w2:Float = 1 - w1;
        r1x = r1.x + rdx * w1;
        r1y = r1.y + rdy * w1;
        r1z = r1.z + rdz * w1;
        r2x = r2.x - rdx * w2;
        r2y = r2.y - rdy * w2;
        r2z = r2.z - rdz * w2;
        
        t1x = r1y * az - r1z * ay;
        t1y = r1z * ax - r1x * az;
        t1z = r1x * ay - r1y * ax;
        t2x = r2y * az - r2z * ay;
        t2y = r2z * ax - r2x * az;
        t2z = r2x * ay - r2y * ax;
        l1x = ax * m1;
        l1y = ay * m1;
        l1z = az * m1;
        l2x = ax * m2;
        l2y = ay * m2;
        l2z = az * m2;
        a1x = t1x * i1e00 + t1y * i1e01 + t1z * i1e02;
        a1y = t1x * i1e10 + t1y * i1e11 + t1z * i1e12;
        a1z = t1x * i1e20 + t1y * i1e21 + t1z * i1e22;
        a2x = t2x * i2e00 + t2y * i2e01 + t2z * i2e02;
        a2y = t2x * i2e10 + t2y * i2e11 + t2z * i2e12;
        a2z = t2x * i2e20 + t2y * i2e21 + t2z * i2e22;
        motorDenom =
                m1 + m2 +
                ax * (a1y * r1z - a1z * r1y + a2y * r2z - a2z * r2y) +
                ay * (a1z * r1x - a1x * r1z + a2z * r2x - a2x * r2z) +
                az * (a1x * r1y - a1y * r1x + a2x * r2y - a2y * r2x);
        invMotorDenom = 1 / motorDenom;
        
        if (enableSpring && limitState != 2) {
            var omega:Float = 6.2831853 * frequency;
            var k:Float = omega * omega * timeStep;
            var dmp:Float = invTimeStep / (k + 2 * limitMotor.dampingRatio * omega);
            cfm = motorDenom * dmp;
            limitVelocity *= k * dmp;
        }
        else {
            cfm = 0;
            limitVelocity *= invTimeStep * 0.05;
        }
        
        invDenom = 1 / (motorDenom + cfm);
        
        var totalImpulse:Float = limitImpulse + motorImpulse;
        l1.x += totalImpulse * l1x;
        l1.y += totalImpulse * l1y;
        l1.z += totalImpulse * l1z;
        a1.x += totalImpulse * a1x;
        a1.y += totalImpulse * a1y;
        a1.z += totalImpulse * a1z;
        l2.x -= totalImpulse * l2x;
        l2.y -= totalImpulse * l2y;
        l2.z -= totalImpulse * l2z;
        a2.x -= totalImpulse * a2x;
        a2.y -= totalImpulse * a2y;
        a2.z -= totalImpulse * a2z;
    }
    
    inline public function solve() {
        var rvn:Float = 
        ax * (l2.x - l1.x) + ay * (l2.y - l1.y) + az * (l2.z - l1.z) +
        t2x * a2.x - t1x * a1.x + t2y * a2.y - t1y * a1.y + t2z * a2.z - t1z * a1.z;
        
        // motor part
        var newMotorImpulse:Float;
        if (enableMotor) {
            newMotorImpulse = (rvn - motorSpeed) * invMotorDenom;
            var oldMotorImpulse:Float = motorImpulse;
            motorImpulse += newMotorImpulse;
            if (motorImpulse > maxMotorImpulse) {
                motorImpulse = maxMotorImpulse;
			}
            else if (motorImpulse < -maxMotorImpulse) {
                motorImpulse = -maxMotorImpulse;
			}
            newMotorImpulse = motorImpulse - oldMotorImpulse;
            rvn -= newMotorImpulse * motorDenom;
        }
        else {
			newMotorImpulse = 0;
		}
        
        // limit part
        var newLimitImpulse:Float;
        if (limitState != 2) {
            newLimitImpulse = (rvn - limitVelocity - limitImpulse * cfm) * invDenom;
            var oldLimitImpulse:Float = limitImpulse;
            limitImpulse += newLimitImpulse;
            if (limitImpulse * limitState < 0) {
                limitImpulse = 0;
			}
            newLimitImpulse = limitImpulse - oldLimitImpulse;
        }
        else {
			newLimitImpulse = 0;
		}
        
        var totalImpulse:Float = newLimitImpulse + newMotorImpulse;
        l1.x += totalImpulse * l1x;
        l1.y += totalImpulse * l1y;
        l1.z += totalImpulse * l1z;
        a1.x += totalImpulse * a1x;
        a1.y += totalImpulse * a1y;
        a1.z += totalImpulse * a1z;
        l2.x -= totalImpulse * l2x;
        l2.y -= totalImpulse * l2y;
        l2.z -= totalImpulse * l2z;
        a2.x -= totalImpulse * a2x;
        a2.y -= totalImpulse * a2y;
        a2.z -= totalImpulse * a2z;
    }
	
}
