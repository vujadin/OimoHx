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
package oimohx.physics.collision.shape;

import oimohx.physics.collision.shape.ShapeConfig;

import oimohx.physics.collision.broadphase.AABB;
import oimohx.physics.collision.broadphase.Proxy;
import oimohx.physics.constraint.contact.ContactLink;
import oimohx.physics.dynamics.RigidBody;
import oimohx.math.Mat33;
import oimohx.math.Vec3;

/**
 * A shape is used to detect collisions of rigid bodies.
 * @author saharan
 */
class Shape {
	
    /**
	 * Global identification of next shape.
	 * This will be incremented every time a shape is created.
	 */	
    public static var nextID:Int = 0;
    
    /**
	 * Sphere shape.
	 */
    public static inline var SHAPE_SPHERE:Int = 0x1;
    
    /**
	 * Box shape.
	 */
    public static inline var SHAPE_BOX:Int = 0x2;
    
    /**
	 * The previous shape in parent rigid body.
	 */
    public var prev:Shape;
    
    /**
	 * The next shape in parent rigid body.
	 */
    public var next:Shape;
    
    /**
	 * The global identification of the shape.
	 * This value should be unique to the shape.
	 */
    public var id:Int;
    
    /**
	 * The type of the shape.
	 */
    public var type:Int;
    
    /**
	 * The center of gravity of the shape in world coordinate system.
	 */
    public var position:Vec3;
    
    /**
	 * The rotation matrix of the shape in world coordinate system.
	 */
    public var rotation:Mat33;
    
    /**
	 * The position of the shape in parent's coordinate system.
	 */
    public var relativePosition:Vec3;
    
    /**
	 * The rotation matrix of the shape in parent's coordinate system.
	 */
    public var relativeRotation:Mat33;
    
    /**
	 * The coefficient of friction of the shape.
	 */
    public var friction:Float;
    
    /**
	 * The coefficient of restitution of the shape.
	 */
    public var restitution:Float;
    
    /**
	 * The density of the shape.
	 */
    public var density:Float;
    
    /**
	 * The axis-aligned bounding box of the shape.
	 */
    public var aabb:AABB;
    
    /**
	 * The proxy of the shape.
	 * This is used for broad-phase collision detection.
	 */
    public var proxy:Proxy;
    
    /**
	 * The parent rigid body of the shape.
	 */
    public var parent:RigidBody = null;
    
    /**
	 * The linked list of the contacts with the shape.
	 */
    public var contactLink:ContactLink;
    
    /**
	 * The number of the contacts with the shape.
	 */
    public var numContacts:Int;
    
    /**
	 * The bits of the collision groups to which the shape belongs.
	 */
    public var belongsTo:Int;
    
    /**
	 * The bits of the collision groups with which the shape collides.
	 */
    public var collidesWith:Int;
	
    
    public function new(config:ShapeConfig) {
        id = ++nextID;
        position = new Vec3();
        relativePosition = new Vec3().copy(config.relativePosition);
        rotation = new Mat33();
        relativeRotation = new Mat33().copy(config.relativeRotation);
        aabb = new AABB();
        density = config.density;
        friction = config.friction;
        restitution = config.restitution;
        belongsTo = config.belongsTo;
        collidesWith = config.collidesWith;
    }
    
    /**
	 * Calculate the mass information of the shape.
	 * @param	out
	 */
	public var calculateMassInfo:MassInfo->Void;
    
    /**
	 * Update the proxy of the shape.
	 */
	public var updateProxy:Void->Void;
	
}
