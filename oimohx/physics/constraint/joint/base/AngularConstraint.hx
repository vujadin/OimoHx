package oimohx.physics.constraint.joint.base;

import oimohx.math.Mat33;
import oimohx.math.Quat;
import oimohx.math.Vec3;
import oimohx.physics.constraint.joint.Joint;
import oimohx.physics.dynamics.RigidBody;

/**
 * An angular constraint for all axes for various joints.
 * @author saharan
 */
class AngularConstraint {
    private var joint:Joint;
    private var b1:RigidBody;
    private var b2:RigidBody;
    private var a1:Vec3;
    private var a2:Vec3;
    private var i1:Mat33;
    private var i2:Mat33;
    
    private var targetOrientation:Quat;
    private var relativeOrientation:Quat;
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
    private var d00:Float;
    private var d01:Float;
    private var d02:Float;
    private var d10:Float;
    private var d11:Float;
    private var d12:Float;
    private var d20:Float;
    private var d21:Float;
    private var d22:Float;
    private var ax:Float;
    private var ay:Float;
    private var az:Float;
    private var impx:Float;
    private var impy:Float;
    private var impz:Float;
    private var velx:Float;
    private var vely:Float;
    private var velz:Float;
    
    public function new(joint:Joint, targetOrientation:Quat) {
        this.joint = joint;
        this.targetOrientation = new Quat().invert(targetOrientation);
        relativeOrientation = new Quat();
        b1 = joint.body1;
        b2 = joint.body2;
        a1 = b1.angularVelocity;
        a2 = b2.angularVelocity;
        i1 = b1.inverseInertia;
        i2 = b2.inverseInertia;
        impx = 0;
        impy = 0;
        impz = 0;
    }
    
    inline public function preSolve(timeStep:Float, invTimeStep:Float) {
        i1e00 = i1.elements[0];
        i1e01 = i1.elements[1];
        i1e02 = i1.elements[2];
        i1e10 = i1.elements[3];
        i1e11 = i1.elements[4];
        i1e12 = i1.elements[5];
        i1e20 = i1.elements[6];
        i1e21 = i1.elements[7];
        i1e22 = i1.elements[8];
        i2e00 = i2.elements[0];
        i2e01 = i2.elements[1];
        i2e02 = i2.elements[2];
        i2e10 = i2.elements[3];
        i2e11 = i2.elements[4];
        i2e12 = i2.elements[5];
        i2e20 = i2.elements[6];
        i2e21 = i2.elements[7];
        i2e22 = i2.elements[8];
		
        var v00:Float = i1e00 + i2e00;
        var v01:Float = i1e01 + i2e01;
        var v02:Float = i1e02 + i2e02;
        var v10:Float = i1e10 + i2e10;
        var v11:Float = i1e11 + i2e11;
        var v12:Float = i1e12 + i2e12;
        var v20:Float = i1e20 + i2e20;
        var v21:Float = i1e21 + i2e21;
        var v22:Float = i1e22 + i2e22;
        var inv:Float = 1 / (
        v00 * (v11 * v22 - v21 * v12) +
        v10 * (v21 * v02 - v01 * v22) +
        v20 * (v01 * v12 - v11 * v02));
        d00 = (v11 * v22 - v12 * v21) * inv;
        d01 = (v02 * v21 - v01 * v22) * inv;
        d02 = (v01 * v12 - v02 * v11) * inv;
        d10 = (v12 * v20 - v10 * v22) * inv;
        d11 = (v00 * v22 - v02 * v20) * inv;
        d12 = (v02 * v10 - v00 * v12) * inv;
        d20 = (v10 * v21 - v11 * v20) * inv;
        d21 = (v01 * v20 - v00 * v21) * inv;
        d22 = (v00 * v11 - v01 * v10) * inv;
        
        relativeOrientation.invert(b1.orientation);  // error = b2 - b1 - target  
        relativeOrientation.mul(targetOrientation, relativeOrientation);
        relativeOrientation.mul(b2.orientation, relativeOrientation);
		
        inv = relativeOrientation.s * 2;
        velx = relativeOrientation.x * inv;
        vely = relativeOrientation.y * inv;
        velz = relativeOrientation.z * inv;
		
        var len:Float = Math.sqrt(velx * velx + vely * vely + velz * velz);
        if (len > 0.02) {
            len = (0.02 - len) / len * invTimeStep * 0.05;
            velx *= len;
            vely *= len;
            velz *= len;
        }
        else {
            velx = 0;
            vely = 0;
            velz = 0;
        }
		
        a1.x += impx * i1e00 + impy * i1e01 + impz * i1e02;
        a1.y += impx * i1e10 + impy * i1e11 + impz * i1e12;
        a1.z += impx * i1e20 + impy * i1e21 + impz * i1e22;
        a2.x -= impx * i2e00 + impy * i2e01 + impz * i2e02;
        a2.y -= impx * i2e10 + impy * i2e11 + impz * i2e12;
        a2.z -= impx * i2e20 + impy * i2e21 + impz * i2e22;
    }
    
    inline public function solve() {
        var rvx:Float = a2.x - a1.x - velx;
        var rvy:Float = a2.y - a1.y - vely;
        var rvz:Float = a2.z - a1.z - velz;
		
        var nimpx:Float = rvx * d00 + rvy * d01 + rvz * d02;
        var nimpy:Float = rvx * d10 + rvy * d11 + rvz * d12;
        var nimpz:Float = rvx * d20 + rvy * d21 + rvz * d22;
		
        impx += nimpx;
        impy += nimpy;
        impz += nimpz;
		
        a1.x += nimpx * i1e00 + nimpy * i1e01 + nimpz * i1e02;
        a1.y += nimpx * i1e10 + nimpy * i1e11 + nimpz * i1e12;
        a1.z += nimpx * i1e20 + nimpy * i1e21 + nimpz * i1e22;
        a2.x -= nimpx * i2e00 + nimpy * i2e01 + nimpz * i2e02;
        a2.y -= nimpx * i2e10 + nimpy * i2e11 + nimpz * i2e12;
        a2.z -= nimpx * i2e20 + nimpy * i2e21 + nimpz * i2e22;
    }
	
}
