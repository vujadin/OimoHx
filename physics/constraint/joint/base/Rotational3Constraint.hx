package oimohx.physics.constraint.joint.base;


import oimohx.math.Mat33;
import oimohx.math.Vec3;
import oimohx.physics.constraint.joint.Joint;
import oimohx.physics.constraint.joint.LimitMotor;
import oimohx.physics.dynamics.RigidBody;
/**
	 * A three-axis rotational constraint for various joints.
	 * @author saharan
	 */
class Rotational3Constraint
{
    public var limitMotor1 : LimitMotor;
    public var limitMotor2 : LimitMotor;
    public var limitMotor3 : LimitMotor;
    
    private var cfm1 : Float;
    private var cfm2 : Float;
    private var cfm3 : Float;
    
    private var b1 : RigidBody;
    private var b2 : RigidBody;
    private var a1 : Vec3;
    private var a2 : Vec3;
    private var i1 : Mat33;
    private var i2 : Mat33;
    
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
    private var ax1 : Float;
    private var ay1 : Float;
    private var az1 : Float;
    private var ax2 : Float;
    private var ay2 : Float;
    private var az2 : Float;
    private var ax3 : Float;
    private var ay3 : Float;
    private var az3 : Float;
    
    private var a1x1 : Float;  // jacoians  
    private var a1y1 : Float;
    private var a1z1 : Float;
    private var a2x1 : Float;
    private var a2y1 : Float;
    private var a2z1 : Float;
    private var a1x2 : Float;
    private var a1y2 : Float;
    private var a1z2 : Float;
    
    private var a2x2 : Float;
    private var a2y2 : Float;
    private var a2z2 : Float;
    private var a1x3 : Float;
    private var a1y3 : Float;
    private var a1z3 : Float;
    private var a2x3 : Float;
    private var a2y3 : Float;
    private var a2z3 : Float;
    
    private var lowerLimit1 : Float;
    private var upperLimit1 : Float;
    private var limitVelocity1 : Float;
    private var limitImpulse1 : Float;
    private var limitState1 : Int;  // -1: at lower, 0: locked, 1: at upper, 2: free  
    
    private var enableMotor1 : Bool;
    private var motorSpeed1 : Float;
    private var maxMotorForce1 : Float;
    private var maxMotorImpulse1 : Float;
    private var motorImpulse1 : Float;
    
    private var lowerLimit2 : Float;
    private var upperLimit2 : Float;
    private var limitVelocity2 : Float;
    private var limitImpulse2 : Float;
    private var limitState2 : Int;  // -1: at lower, 0: locked, 1: at upper, 2: free  
    
    private var enableMotor2 : Bool;
    private var motorSpeed2 : Float;
    private var maxMotorForce2 : Float;
    private var maxMotorImpulse2 : Float;
    private var motorImpulse2 : Float;
    
    private var lowerLimit3 : Float;
    private var upperLimit3 : Float;
    private var limitVelocity3 : Float;
    private var limitImpulse3 : Float;
    private var limitState3 : Int;  // -1: at lower, 0: locked, 1: at upper, 2: free  
    
    private var enableMotor3 : Bool;
    private var motorSpeed3 : Float;
    private var maxMotorForce3 : Float;
    private var maxMotorImpulse3 : Float;
    private var motorImpulse3 : Float;
    
    private var k00 : Float;  // K = J*M*JT  
    private var k01 : Float;
    private var k02 : Float;
    private var k10 : Float;
    private var k11 : Float;
    private var k12 : Float;
    private var k20 : Float;
    private var k21 : Float;
    private var k22 : Float;
    
    private var kv00 : Float;  // diagonals without CFMs  
    private var kv11 : Float;
    private var kv22 : Float;
    
    private var dv00 : Float;  // ...inverted  
    private var dv11 : Float;
    private var dv22 : Float;
    
    private var d00 : Float;  // K^-1  
    private var d01 : Float;
    private var d02 : Float;
    private var d10 : Float;
    private var d11 : Float;
    private var d12 : Float;
    private var d20 : Float;
    private var d21 : Float;
    private var d22 : Float;
    
    public function new(joint : Joint, limitMotor1 : LimitMotor, limitMotor2 : LimitMotor, limitMotor3 : LimitMotor)
    {
        this.limitMotor1 = limitMotor1;
        this.limitMotor2 = limitMotor2;
        this.limitMotor3 = limitMotor3;
        b1 = joint.body1;
        b2 = joint.body2;
        a1 = b1.angularVelocity;
        a2 = b2.angularVelocity;
        i1 = b1.inverseInertia;
        i2 = b2.inverseInertia;
        limitImpulse1 = 0;
        motorImpulse1 = 0;
        limitImpulse2 = 0;
        motorImpulse2 = 0;
        limitImpulse3 = 0;
        motorImpulse3 = 0;
    }
    
    public function preSolve(timeStep : Float, invTimeStep : Float) : Void{
        ax1 = limitMotor1.axis.x;
        ay1 = limitMotor1.axis.y;
        az1 = limitMotor1.axis.z;
        ax2 = limitMotor2.axis.x;
        ay2 = limitMotor2.axis.y;
        az2 = limitMotor2.axis.z;
        ax3 = limitMotor3.axis.x;
        ay3 = limitMotor3.axis.y;
        az3 = limitMotor3.axis.z;
        lowerLimit1 = limitMotor1.lowerLimit;
        upperLimit1 = limitMotor1.upperLimit;
        motorSpeed1 = limitMotor1.motorSpeed;
        maxMotorForce1 = limitMotor1.maxMotorForce;
        enableMotor1 = maxMotorForce1 > 0;
        lowerLimit2 = limitMotor2.lowerLimit;
        upperLimit2 = limitMotor2.upperLimit;
        motorSpeed2 = limitMotor2.motorSpeed;
        maxMotorForce2 = limitMotor2.maxMotorForce;
        enableMotor2 = maxMotorForce2 > 0;
        lowerLimit3 = limitMotor3.lowerLimit;
        upperLimit3 = limitMotor3.upperLimit;
        motorSpeed3 = limitMotor3.motorSpeed;
        maxMotorForce3 = limitMotor3.maxMotorForce;
        enableMotor3 = maxMotorForce3 > 0;
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
        
        var frequency1 : Float = limitMotor1.frequency;
        var frequency2 : Float = limitMotor2.frequency;
        var frequency3 : Float = limitMotor3.frequency;
        var enableSpring1 : Bool = frequency1 > 0;
        var enableSpring2 : Bool = frequency2 > 0;
        var enableSpring3 : Bool = frequency3 > 0;
        var enableLimit1 : Bool = lowerLimit1 <= upperLimit1;
        var enableLimit2 : Bool = lowerLimit2 <= upperLimit2;
        var enableLimit3 : Bool = lowerLimit3 <= upperLimit3;
        
        var angle1 : Float = limitMotor1.angle;
        if (enableLimit1) {
            if (lowerLimit1 == upperLimit1) {
                if (limitState1 != 0) {
                    limitState1 = 0;
                    limitImpulse1 = 0;
                }
                limitVelocity1 = lowerLimit1 - angle1;
            }
            else if (angle1 < lowerLimit1) {
                if (limitState1 != -1) {
                    limitState1 = -1;
                    limitImpulse1 = 0;
                }
                limitVelocity1 = lowerLimit1 - angle1;
            }
            else if (angle1 > upperLimit1) {
                if (limitState1 != 1) {
                    limitState1 = 1;
                    limitImpulse1 = 0;
                }
                limitVelocity1 = upperLimit1 - angle1;
            }
            else {
                limitState1 = 2;
                limitImpulse1 = 0;
                limitVelocity1 = 0;
            }
            if (!enableSpring1) {
                if (limitVelocity1 > 0.02)                     limitVelocity1 -= 0.02
                else if (limitVelocity1 < -0.02)                     limitVelocity1 += 0.02
                else limitVelocity1 = 0;
            }
        }
        else {
            limitState1 = 2;
            limitImpulse1 = 0;
        }
        
        var angle2 : Float = limitMotor2.angle;
        if (enableLimit2) {
            if (lowerLimit2 == upperLimit2) {
                if (limitState2 != 0) {
                    limitState2 = 0;
                    limitImpulse2 = 0;
                }
                limitVelocity2 = lowerLimit2 - angle2;
            }
            else if (angle2 < lowerLimit2) {
                if (limitState2 != -1) {
                    limitState2 = -1;
                    limitImpulse2 = 0;
                }
                limitVelocity2 = lowerLimit2 - angle2;
            }
            else if (angle2 > upperLimit2) {
                if (limitState2 != 1) {
                    limitState2 = 1;
                    limitImpulse2 = 0;
                }
                limitVelocity2 = upperLimit2 - angle2;
            }
            else {
                limitState2 = 2;
                limitImpulse2 = 0;
                limitVelocity2 = 0;
            }
            if (!enableSpring2) {
                if (limitVelocity2 > 0.02)                     limitVelocity2 -= 0.02
                else if (limitVelocity2 < -0.02)                     limitVelocity2 += 0.02
                else limitVelocity2 = 0;
            }
        }
        else {
            limitState2 = 2;
            limitImpulse2 = 0;
        }
        
        var angle3 : Float = limitMotor3.angle;
        if (enableLimit3) {
            if (lowerLimit3 == upperLimit3) {
                if (limitState3 != 0) {
                    limitState3 = 0;
                    limitImpulse3 = 0;
                }
                limitVelocity3 = lowerLimit3 - angle3;
            }
            else if (angle3 < lowerLimit3) {
                if (limitState3 != -1) {
                    limitState3 = -1;
                    limitImpulse3 = 0;
                }
                limitVelocity3 = lowerLimit3 - angle3;
            }
            else if (angle3 > upperLimit3) {
                if (limitState3 != 1) {
                    limitState3 = 1;
                    limitImpulse3 = 0;
                }
                limitVelocity3 = upperLimit3 - angle3;
            }
            else {
                limitState3 = 2;
                limitImpulse3 = 0;
                limitVelocity3 = 0;
            }
            if (!enableSpring3) {
                if (limitVelocity3 > 0.02)                     limitVelocity3 -= 0.02
                else if (limitVelocity3 < -0.02)                     limitVelocity3 += 0.02
                else limitVelocity3 = 0;
            }
        }
        else {
            limitState3 = 2;
            limitImpulse3 = 0;
        }
        
        if (enableMotor1 && (limitState1 != 0 || enableSpring1)) {
            maxMotorImpulse1 = maxMotorForce1 * timeStep;
        }
        else {
            motorImpulse1 = 0;
            maxMotorImpulse1 = 0;
        }
        
        if (enableMotor2 && (limitState2 != 0 || enableSpring2)) {
            maxMotorImpulse2 = maxMotorForce2 * timeStep;
        }
        else {
            motorImpulse2 = 0;
            maxMotorImpulse2 = 0;
        }
        
        if (enableMotor3 && (limitState3 != 0 || enableSpring3)) {
            maxMotorImpulse3 = maxMotorForce3 * timeStep;
        }
        else {
            motorImpulse3 = 0;
            maxMotorImpulse3 = 0;
        }  // build jacobians  
        
        
        
        a1x1 = ax1 * i1e00 + ay1 * i1e01 + az1 * i1e02;
        a1y1 = ax1 * i1e10 + ay1 * i1e11 + az1 * i1e12;
        a1z1 = ax1 * i1e20 + ay1 * i1e21 + az1 * i1e22;
        a2x1 = ax1 * i2e00 + ay1 * i2e01 + az1 * i2e02;
        a2y1 = ax1 * i2e10 + ay1 * i2e11 + az1 * i2e12;
        a2z1 = ax1 * i2e20 + ay1 * i2e21 + az1 * i2e22;
        
        a1x2 = ax2 * i1e00 + ay2 * i1e01 + az2 * i1e02;
        a1y2 = ax2 * i1e10 + ay2 * i1e11 + az2 * i1e12;
        a1z2 = ax2 * i1e20 + ay2 * i1e21 + az2 * i1e22;
        a2x2 = ax2 * i2e00 + ay2 * i2e01 + az2 * i2e02;
        a2y2 = ax2 * i2e10 + ay2 * i2e11 + az2 * i2e12;
        a2z2 = ax2 * i2e20 + ay2 * i2e21 + az2 * i2e22;
        
        a1x3 = ax3 * i1e00 + ay3 * i1e01 + az3 * i1e02;
        a1y3 = ax3 * i1e10 + ay3 * i1e11 + az3 * i1e12;
        a1z3 = ax3 * i1e20 + ay3 * i1e21 + az3 * i1e22;
        a2x3 = ax3 * i2e00 + ay3 * i2e01 + az3 * i2e02;
        a2y3 = ax3 * i2e10 + ay3 * i2e11 + az3 * i2e12;
        a2z3 = ax3 * i2e20 + ay3 * i2e21 + az3 * i2e22;
        
        // build an impulse matrix
        k00 = ax1 * (a1x1 + a2x1) + ay1 * (a1y1 + a2y1) + az1 * (a1z1 + a2z1);
        k01 = ax1 * (a1x2 + a2x2) + ay1 * (a1y2 + a2y2) + az1 * (a1z2 + a2z2);
        k02 = ax1 * (a1x3 + a2x3) + ay1 * (a1y3 + a2y3) + az1 * (a1z3 + a2z3);
        k10 = ax2 * (a1x1 + a2x1) + ay2 * (a1y1 + a2y1) + az2 * (a1z1 + a2z1);
        k11 = ax2 * (a1x2 + a2x2) + ay2 * (a1y2 + a2y2) + az2 * (a1z2 + a2z2);
        k12 = ax2 * (a1x3 + a2x3) + ay2 * (a1y3 + a2y3) + az2 * (a1z3 + a2z3);
        k20 = ax3 * (a1x1 + a2x1) + ay3 * (a1y1 + a2y1) + az3 * (a1z1 + a2z1);
        k21 = ax3 * (a1x2 + a2x2) + ay3 * (a1y2 + a2y2) + az3 * (a1z2 + a2z2);
        k22 = ax3 * (a1x3 + a2x3) + ay3 * (a1y3 + a2y3) + az3 * (a1z3 + a2z3);
        
        kv00 = k00;
        kv11 = k11;
        kv22 = k22;
        dv00 = 1 / kv00;
        dv11 = 1 / kv11;
        dv22 = 1 / kv22;
        
		var omega:Float = 0.0;
		var k:Float = 0.0;
		var dmp:Float = 0.0;
		
        if (enableSpring1 && limitState1 != 2) {
            omega = 6.2831853 * frequency1;
            k = omega * omega * timeStep;
            dmp = invTimeStep / (k + 2 * limitMotor1.dampingRatio * omega);
            cfm1 = kv00 * dmp;
            limitVelocity1 *= k * dmp;
        }
        else {
            cfm1 = 0;
            limitVelocity1 *= invTimeStep * 0.05;
        }
        
        if (enableSpring2 && limitState2 != 2) {
            omega = 6.2831853 * frequency2;
            k = omega * omega * timeStep;
            dmp = invTimeStep / (k + 2 * limitMotor2.dampingRatio * omega);
            cfm2 = kv11 * dmp;
            limitVelocity2 *= k * dmp;
        }
        else {
            cfm2 = 0;
            limitVelocity2 *= invTimeStep * 0.05;
        }
        
        if (enableSpring3 && limitState3 != 2) {
            omega = 6.2831853 * frequency3;
            k = omega * omega * timeStep;
            dmp = invTimeStep / (k + 2 * limitMotor3.dampingRatio * omega);
            cfm3 = kv22 * dmp;
            limitVelocity3 *= k * dmp;
        }
        else {
            cfm3 = 0;
            limitVelocity3 *= invTimeStep * 0.05;
        }
        
        k00 += cfm1;
        k11 += cfm2;
        k22 += cfm3;
        
        var inv : Float = 1 / (
        k00 * (k11 * k22 - k21 * k12) +
        k10 * (k21 * k02 - k01 * k22) +
        k20 * (k01 * k12 - k11 * k02));
        d00 = (k11 * k22 - k12 * k21) * inv;
        d01 = (k02 * k21 - k01 * k22) * inv;
        d02 = (k01 * k12 - k02 * k11) * inv;
        d10 = (k12 * k20 - k10 * k22) * inv;
        d11 = (k00 * k22 - k02 * k20) * inv;
        d12 = (k02 * k10 - k00 * k12) * inv;
        d20 = (k10 * k21 - k11 * k20) * inv;
        d21 = (k01 * k20 - k00 * k21) * inv;
        d22 = (k00 * k11 - k01 * k10) * inv;
        
        limitImpulse1 *= 0.95;
        motorImpulse1 *= 0.95;
        limitImpulse2 *= 0.95;
        motorImpulse2 *= 0.95;
        limitImpulse3 *= 0.95;
        motorImpulse3 *= 0.95;
        var totalImpulse1 : Float = limitImpulse1 + motorImpulse1;
        var totalImpulse2 : Float = limitImpulse2 + motorImpulse2;
        var totalImpulse3 : Float = limitImpulse3 + motorImpulse3;
        a1.x += totalImpulse1 * a1x1 + totalImpulse2 * a1x2 + totalImpulse3 * a1x3;
        a1.y += totalImpulse1 * a1y1 + totalImpulse2 * a1y2 + totalImpulse3 * a1y3;
        a1.z += totalImpulse1 * a1z1 + totalImpulse2 * a1z2 + totalImpulse3 * a1z3;
        a2.x -= totalImpulse1 * a2x1 + totalImpulse2 * a2x2 + totalImpulse3 * a2x3;
        a2.y -= totalImpulse1 * a2y1 + totalImpulse2 * a2y2 + totalImpulse3 * a2y3;
        a2.z -= totalImpulse1 * a2z1 + totalImpulse2 * a2z2 + totalImpulse3 * a2z3;
    }
    
    public function solve_() : Void{
        var rvx : Float = a2.x - a1.x;
        var rvy : Float = a2.y - a1.y;
        var rvz : Float = a2.z - a1.z;
        
        limitVelocity3 = 30;
        var rvn1 : Float = rvx * ax1 + rvy * ay1 + rvz * az1 - limitVelocity1;
        var rvn2 : Float = rvx * ax2 + rvy * ay2 + rvz * az2 - limitVelocity2;
        var rvn3 : Float = rvx * ax3 + rvy * ay3 + rvz * az3 - limitVelocity3;
        
        var dLimitImpulse1 : Float = rvn1 * d00 + rvn2 * d01 + rvn3 * d02;
        var dLimitImpulse2 : Float = rvn1 * d10 + rvn2 * d11 + rvn3 * d12;
        var dLimitImpulse3 : Float = rvn1 * d20 + rvn2 * d21 + rvn3 * d22;
        
        limitImpulse1 += dLimitImpulse1;
        limitImpulse2 += dLimitImpulse2;
        limitImpulse3 += dLimitImpulse3;
        
        a1.x += dLimitImpulse1 * a1x1 + dLimitImpulse2 * a1x2 + dLimitImpulse3 * a1x3;
        a1.y += dLimitImpulse1 * a1y1 + dLimitImpulse2 * a1y2 + dLimitImpulse3 * a1y3;
        a1.z += dLimitImpulse1 * a1z1 + dLimitImpulse2 * a1z2 + dLimitImpulse3 * a1z3;
        a2.x -= dLimitImpulse1 * a2x1 + dLimitImpulse2 * a2x2 + dLimitImpulse3 * a2x3;
        a2.y -= dLimitImpulse1 * a2y1 + dLimitImpulse2 * a2y2 + dLimitImpulse3 * a2y3;
        a2.z -= dLimitImpulse1 * a2z1 + dLimitImpulse2 * a2z2 + dLimitImpulse3 * a2z3;
    }
    
    public function solve() : Void{
        var rvx : Float = a2.x - a1.x;
        var rvy : Float = a2.y - a1.y;
        var rvz : Float = a2.z - a1.z;
        
        var rvn1 : Float = rvx * ax1 + rvy * ay1 + rvz * az1;
        var rvn2 : Float = rvx * ax2 + rvy * ay2 + rvz * az2;
        var rvn3 : Float = rvx * ax3 + rvy * ay3 + rvz * az3;
        
        var oldMotorImpulse1 : Float = motorImpulse1;
        var oldMotorImpulse2 : Float = motorImpulse2;
        var oldMotorImpulse3 : Float = motorImpulse3;
        
        var dMotorImpulse1 : Float = 0;
        var dMotorImpulse2 : Float = 0;
        var dMotorImpulse3 : Float = 0;
        
        if (enableMotor1) {
            dMotorImpulse1 = (rvn1 - motorSpeed1) * dv00;
            motorImpulse1 += dMotorImpulse1;
            if (motorImpulse1 > maxMotorImpulse1) {  // clamp motor impulse  
                motorImpulse1 = maxMotorImpulse1;
            }
            else if (motorImpulse1 < -maxMotorImpulse1) {
                motorImpulse1 = -maxMotorImpulse1;
            }
            dMotorImpulse1 = motorImpulse1 - oldMotorImpulse1;
        }
        
        if (enableMotor2) {
            dMotorImpulse2 = (rvn2 - motorSpeed2) * dv11;
            motorImpulse2 += dMotorImpulse2;
            if (motorImpulse2 > maxMotorImpulse2) {  // clamp motor impulse  
                motorImpulse2 = maxMotorImpulse2;
            }
            else if (motorImpulse2 < -maxMotorImpulse2) {
                motorImpulse2 = -maxMotorImpulse2;
            }
            dMotorImpulse2 = motorImpulse2 - oldMotorImpulse2;
        }
        
        if (enableMotor3) {
            dMotorImpulse3 = (rvn3 - motorSpeed3) * dv22;
            motorImpulse3 += dMotorImpulse3;
            if (motorImpulse3 > maxMotorImpulse3) {  // clamp motor impulse  
                motorImpulse3 = maxMotorImpulse3;
            }
            else if (motorImpulse3 < -maxMotorImpulse3) {
                motorImpulse3 = -maxMotorImpulse3;
            }
            dMotorImpulse3 = motorImpulse3 - oldMotorImpulse3;
        }  // apply motor impulse to relative velocity  
        
        
        
        rvn1 += dMotorImpulse1 * kv00 + dMotorImpulse2 * k01 + dMotorImpulse3 * k02;
        rvn2 += dMotorImpulse1 * k10 + dMotorImpulse2 * kv11 + dMotorImpulse3 * k12;
        rvn3 += dMotorImpulse1 * k20 + dMotorImpulse2 * k21 + dMotorImpulse3 * kv22;
        
        // subtract target velocity and applied impulse
        rvn1 -= limitVelocity1 + limitImpulse1 * cfm1;
        rvn2 -= limitVelocity2 + limitImpulse2 * cfm2;
        rvn3 -= limitVelocity3 + limitImpulse3 * cfm3;
        
        var oldLimitImpulse1 : Float = limitImpulse1;
        var oldLimitImpulse2 : Float = limitImpulse2;
        var oldLimitImpulse3 : Float = limitImpulse3;
        
        var dLimitImpulse1 : Float = rvn1 * d00 + rvn2 * d01 + rvn3 * d02;
        var dLimitImpulse2 : Float = rvn1 * d10 + rvn2 * d11 + rvn3 * d12;
        var dLimitImpulse3 : Float = rvn1 * d20 + rvn2 * d21 + rvn3 * d22;
        
        limitImpulse1 += dLimitImpulse1;
        limitImpulse2 += dLimitImpulse2;
        limitImpulse3 += dLimitImpulse3;
        
        // clamp
        var clampState : Int = 0;
        if (limitState1 == 2 || limitImpulse1 * limitState1 < 0) {
            dLimitImpulse1 = -oldLimitImpulse1;
            rvn2 += dLimitImpulse1 * k10;
            rvn3 += dLimitImpulse1 * k20;
            clampState |= 1;
        }
        if (limitState2 == 2 || limitImpulse2 * limitState2 < 0) {
            dLimitImpulse2 = -oldLimitImpulse2;
            rvn1 += dLimitImpulse2 * k01;
            rvn3 += dLimitImpulse2 * k21;
            clampState |= 2;
        }
        if (limitState3 == 2 || limitImpulse3 * limitState3 < 0) {
            dLimitImpulse3 = -oldLimitImpulse3;
            rvn1 += dLimitImpulse3 * k02;
            rvn2 += dLimitImpulse3 * k12;
            clampState |= 4;
        }  // TODO: isolate division    // update un-clamped impulse  
        
        
        
        
        
        var det : Float;
        switch (clampState)
        {
            case 1:  // update 2 3  
                det = 1 / (k11 * k22 - k12 * k21);
                dLimitImpulse2 = (k22 * rvn2 + -k12 * rvn3) * det;
                dLimitImpulse3 = (-k21 * rvn2 + k11 * rvn3) * det;
            case 2:  // update 1 3  
                det = 1 / (k00 * k22 - k02 * k20);
                dLimitImpulse1 = (k22 * rvn1 + -k02 * rvn3) * det;
                dLimitImpulse3 = (-k20 * rvn1 + k00 * rvn3) * det;
            case 3:  // update 3  
            dLimitImpulse3 = rvn3 / k22;
            case 4:  // update 1 2  
                det = 1 / (k00 * k11 - k01 * k10);
                dLimitImpulse1 = (k11 * rvn1 + -k01 * rvn2) * det;
                dLimitImpulse2 = (-k10 * rvn1 + k00 * rvn2) * det;
            case 5:  // update 2  
            dLimitImpulse2 = rvn2 / k11;
            case 6:  // update 1  
            dLimitImpulse1 = rvn1 / k00;
        }
        
        limitImpulse1 = dLimitImpulse1 + oldLimitImpulse1;
        limitImpulse2 = dLimitImpulse2 + oldLimitImpulse2;
        limitImpulse3 = dLimitImpulse3 + oldLimitImpulse3;
        
        var dImpulse1 : Float = dMotorImpulse1 + dLimitImpulse1;
        var dImpulse2 : Float = dMotorImpulse2 + dLimitImpulse2;
        var dImpulse3 : Float = dMotorImpulse3 + dLimitImpulse3;
        
        // apply impulse
        a1.x += dImpulse1 * a1x1 + dImpulse2 * a1x2 + dImpulse3 * a1x3;
        a1.y += dImpulse1 * a1y1 + dImpulse2 * a1y2 + dImpulse3 * a1y3;
        a1.z += dImpulse1 * a1z1 + dImpulse2 * a1z2 + dImpulse3 * a1z3;
        a2.x -= dImpulse1 * a2x1 + dImpulse2 * a2x2 + dImpulse3 * a2x3;
        a2.y -= dImpulse1 * a2y1 + dImpulse2 * a2y2 + dImpulse3 * a2y3;
        a2.z -= dImpulse1 * a2z1 + dImpulse2 * a2z2 + dImpulse3 * a2z3;
        rvx = a2.x - a1.x;
        rvy = a2.y - a1.y;
        rvz = a2.z - a1.z;
        
        rvn2 = rvx * ax2 + rvy * ay2 + rvz * az2;
    }
}

