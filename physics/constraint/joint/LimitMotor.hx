package oimohx.physics.constraint.joint;


import oimohx.math.Vec3;
/**
	 * An information of limit and motor.
	 * @author saharan
	 */
class LimitMotor
{
    /**
		 * The current angle for rotational constraints.
		 */
    public var angle : Float;
    
    /**
		 * The axis of the constraint.
		 */
    public var axis : Vec3;
    
    /**
		 * The lower limit. Set lower > upper to disable.
		 */
    public var lowerLimit : Float;
    
    /**
		 * The upper limit. Set lower > upper to disable.
		 */
    public var upperLimit : Float;
    
    /**
		 * The target motor speed.
		 */
    public var motorSpeed : Float;
    
    /**
		 * The maximum motor force or torque. Set 0 to disable.
		 */
    public var maxMotorForce : Float;
    
    /**
		 * The frequency of the spring. Set 0 to disable.
		 */
    public var frequency : Float;
    
    /**
		 * The damping ratio of the spring. Set 0 for no damping, 1 for critical damping.
		 */
    public var dampingRatio : Float;
    
    public function new(axis : Vec3, fixed : Bool)
    {
        this.axis = axis;
        angle = 0;
        if (fixed)             lowerLimit = 0
        else lowerLimit = 1;
        upperLimit = 0;
        motorSpeed = 0;
        maxMotorForce = 0;
        frequency = 0;
        dampingRatio = 0;
    }
    
    /**
		 * Set limit data into this constraint.
		 * @param	lowerLimit
		 * @param	upperLimit
		 */
    public function setLimit(lowerLimit : Float, upperLimit : Float) : Void{
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
    }
    
    /**
		 * Set motor data into this constraint.
		 * @param	motorSpeed
		 * @param	maxMotorForce
		 */
    public function setMotor(motorSpeed : Float, maxMotorForce : Float) : Void{
        this.motorSpeed = motorSpeed;
        this.maxMotorForce = maxMotorForce;
    }
    
    /**
		 * Set spring data into this constraint.
		 * @param	frequency
		 * @param	dampingRatio
		 */
    public function setSpring(frequency : Float, dampingRatio : Float) : Void{
        this.frequency = frequency;
        this.dampingRatio = dampingRatio;
    }
}

