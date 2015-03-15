package oimohx.physics.constraint.joint.base;


import oimohx.math.Mat33;
import oimohx.math.Vec3;
import oimohx.physics.constraint.joint.Joint;
import oimohx.physics.constraint.joint.LimitMotor;
import oimohx.physics.dynamics.RigidBody;
/**
	 * A rotational constraint for various joints.
	 * @author saharan
	 */
class RotationalConstraint
{
    public var limitMotor : LimitMotor;
    private var b1 : RigidBody;
    private var b2 : RigidBody;
    private var a1 : Vec3;
    private var a2 : Vec3;
    private var i1 : Mat33;
    private var i2 : Mat33;
    
    private var cfm : Float;
    
    private var i1e00 : Float;
    private var i1e01 : Float;
    private var i1e02 : Float;
    private var i1e10 : Float;
    private var i1e11 : Float;
    private var i1e12 : Float;
    private var i1e20 : Float;
    private var i1e21 : Float;
    private var i1e22 : Float;
    private var i2e00 : Float;
    private var i2e01 : Float;
    private var i2e02 : Float;
    private var i2e10 : Float;
    private var i2e11 : Float;
    private var i2e12 : Float;
    private var i2e20 : Float;
    private var i2e21 : Float;
    private var i2e22 : Float;
    private var motorDenom : Float;
    private var invMotorDenom : Float;
    private var invDenom : Float;
    private var ax : Float;
    private var ay : Float;
    private var az : Float;
    private var a1x : Float;
    private var a1y : Float;
    private var a1z : Float;
    private var a2x : Float;
    private var a2y : Float;
    private var a2z : Float;
    
    private var enableLimit : Bool;
    private var lowerLimit : Float;
    private var upperLimit : Float;
    private var limitVelocity : Float;
    private var limitImpulse : Float;
    private var limitState : Int;  // -1: at lower, 0: locked, 1: at upper, 2: free  
    
    private var enableMotor : Bool;
    private var motorSpeed : Float;
    private var maxMotorForce : Float;
    private var maxMotorImpulse : Float;
    private var motorImpulse : Float;
    
    public function new(joint : Joint, limitMotor : LimitMotor)
    {
        this.limitMotor = limitMotor;
        b1 = joint.body1;
        b2 = joint.body2;
        a1 = b1.angularVelocity;
        a2 = b2.angularVelocity;
        i1 = b1.inverseInertia;
        i2 = b2.inverseInertia;
        limitImpulse = 0;
        motorImpulse = 0;
    }
    
    public function preSolve(timeStep : Float, invTimeStep : Float) : Void{
        ax = limitMotor.axis.x;
        ay = limitMotor.axis.y;
        az = limitMotor.axis.z;
        lowerLimit = limitMotor.lowerLimit;
        upperLimit = limitMotor.upperLimit;
        motorSpeed = limitMotor.motorSpeed;
        maxMotorForce = limitMotor.maxMotorForce;
        enableMotor = maxMotorForce > 0;
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
        
        var frequency : Float = limitMotor.frequency;
        var enableSpring : Bool = frequency > 0;
        var enableLimit : Bool = lowerLimit <= upperLimit;
        
        var angle : Float = limitMotor.angle;
        if (enableLimit) {
            if (lowerLimit == upperLimit) {
                if (limitState != 0) {
                    limitState = 0;
                    limitImpulse = 0;
                }
                limitVelocity = lowerLimit - angle;
            }
            else if (angle < lowerLimit) {
                if (limitState != -1) {
                    limitState = -1;
                    limitImpulse = 0;
                }
                limitVelocity = lowerLimit - angle;
            }
            else if (angle > upperLimit) {
                if (limitState != 1) {
                    limitState = 1;
                    limitImpulse = 0;
                }
                limitVelocity = upperLimit - angle;
            }
            else {
                limitState = 2;
                limitImpulse = 0;
                limitVelocity = 0;
            }
            if (!enableSpring) {
                if (limitVelocity > 0.02)                     limitVelocity -= 0.02
                else if (limitVelocity < -0.02)                     limitVelocity += 0.02
                else limitVelocity = 0;
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
        
        a1x = ax * i1e00 + ay * i1e01 + az * i1e02;
        a1y = ax * i1e10 + ay * i1e11 + az * i1e12;
        a1z = ax * i1e20 + ay * i1e21 + az * i1e22;
        a2x = ax * i2e00 + ay * i2e01 + az * i2e02;
        a2y = ax * i2e10 + ay * i2e11 + az * i2e12;
        a2z = ax * i2e20 + ay * i2e21 + az * i2e22;
        motorDenom = ax * (a1x + a2x) + ay * (a1y + a2y) + az * (a1z + a2z);
        invMotorDenom = 1 / motorDenom;
        
        if (enableSpring && limitState != 2) {
            var omega : Float = 6.2831853 * frequency;
            var k : Float = omega * omega * timeStep;
            var dmp : Float = invTimeStep / (k + 2 * limitMotor.dampingRatio * omega);
            cfm = motorDenom * dmp;
            limitVelocity *= k * dmp;
        }
        else {
            cfm = 0;
            limitVelocity *= invTimeStep * 0.05;
        }
        
        invDenom = 1 / (motorDenom + cfm);
        
        limitImpulse *= 0.95;
        motorImpulse *= 0.95;
        var totalImpulse : Float = limitImpulse + motorImpulse;
        a1.x += totalImpulse * a1x;
        a1.y += totalImpulse * a1y;
        a1.z += totalImpulse * a1z;
        a2.x -= totalImpulse * a2x;
        a2.y -= totalImpulse * a2y;
        a2.z -= totalImpulse * a2z;
    }
    
    public function solve() : Void{
        var rvn : Float = ax * (a2.x - a1.x) + ay * (a2.y - a1.y) + az * (a2.z - a1.z);
        
        // motor part
        var newMotorImpulse : Float;
        if (enableMotor) {
            newMotorImpulse = (rvn - motorSpeed) * invMotorDenom;
            var oldMotorImpulse : Float = motorImpulse;
            motorImpulse += newMotorImpulse;
            if (motorImpulse > maxMotorImpulse)                 motorImpulse = maxMotorImpulse
            else if (motorImpulse < -maxMotorImpulse)                 motorImpulse = -maxMotorImpulse;
            newMotorImpulse = motorImpulse - oldMotorImpulse;
            rvn -= newMotorImpulse * motorDenom;
        }
        else newMotorImpulse = 0;
        
        // limit part
        var newLimitImpulse : Float;
        if (limitState != 2) {
            newLimitImpulse = (rvn - limitVelocity - limitImpulse * cfm) * invDenom;
            var oldLimitImpulse : Float = limitImpulse;
            limitImpulse += newLimitImpulse;
            if (limitImpulse * limitState < 0)                 limitImpulse = 0;
            newLimitImpulse = limitImpulse - oldLimitImpulse;
        }
        else newLimitImpulse = 0;
        
        var totalImpulse : Float = newLimitImpulse + newMotorImpulse;
        a1.x += totalImpulse * a1x;
        a1.y += totalImpulse * a1y;
        a1.z += totalImpulse * a1z;
        a2.x -= totalImpulse * a2x;
        a2.y -= totalImpulse * a2y;
        a2.z -= totalImpulse * a2z;
    }
}

