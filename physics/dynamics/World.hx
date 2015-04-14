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
package oimohx.physics.dynamics;

import oimohx.math.Quat;
import oimohx.math.Vec3;
import oimohx.physics.collision.broadphase.AABB;
import oimohx.physics.collision.broadphase.BroadPhase;
import oimohx.physics.collision.broadphase.BruteForceBroadPhase;
import oimohx.physics.collision.broadphase.dbvt.DBVTBroadPhase;
import oimohx.physics.collision.broadphase.Pair;
import oimohx.physics.collision.broadphase.sap.SAPBroadPhase;
import oimohx.physics.collision.narrowphase.BoxBoxCollisionDetector;
import oimohx.physics.collision.narrowphase.CollisionDetector;
import oimohx.physics.collision.narrowphase.SphereBoxCollisionDetector;
import oimohx.physics.collision.narrowphase.SphereSphereCollisionDetector;
import oimohx.physics.collision.shape.Shape;
import oimohx.physics.constraint.Constraint;
import oimohx.physics.constraint.contact.Contact;
import oimohx.physics.constraint.contact.ContactLink;
import oimohx.physics.constraint.joint.Joint;
import oimohx.physics.constraint.joint.JointLink;
import oimohx.physics.util.Performance;

/**
 * 物理演算ワールドのクラスです。
 * 全ての物理演算オブジェクトはワールドに追加する必要があります。
 * @author saharan
 */
class World {
    /**
	 * The rigid body list.
	 */
    public var rigidBodies:RigidBody;
    
    /**
	 * The number of rigid bodies.
	 */
    public var numRigidBodies:Int;
    
    /**
	 * The contact list.
	 */
    public var contacts:Contact;
    private var unusedContacts:Contact;
    
    /**
	 * The number of contacts.
	 */
    public var numContacts:Int;
    
    /**
	 * The number of contact points.
	 */
    public var numContactPoints:Int;
    
    /**
	 * The joint list.
	 */
    public var joints:Joint;
    
    /**
	 * The number of joints.
	 */
    public var numJoints:Int;
    
    /**
	 * The number of simulation islands.
	 */
    public var numIslands:Int;
    
    /**
	 * 1回のステップで進む時間の長さです。
	 */
    public var timeStep:Float;
    
    /**
	 * The gravity in the world.
	 */
    public static var gravityX:Float = 0;
    public static var gravityY:Float = 0;
    public static var gravityZ:Float = -9.8;
    public var gravity:Vec3;
    
    /**
	 * The number of iterations for constraint solvers.
	 */
    public var numIterations:Int;
    
    /**
	 * Whether the constraints randomizer is enabled or not.
	 */
    public var enableRandomizer:Bool;
    
    /**
	 * パフォーマンスの詳細情報です。
	 * 計算に要した時間などが記録されています。
	 */
    public var performance:Performance;
    
    /**
	 * 詳細な衝突判定をできるだけ削減するために使用される広域衝突判定です。
	 */
    public var broadPhase:BroadPhase;
    
    private var detectors:Array<Array<CollisionDetector>>;
    
    private var islandStack:Array<RigidBody>;
    private var islandRigidBodies:Array<RigidBody>;
    private var maxIslandRigidBodies:Int;
    private var islandConstraints:Array<Constraint>;
    private var maxIslandConstraints:Int;
    
    private var randX:Int;
    private var randA:Int;
    private var randB:Int;
	
    
    public function new(stepPerSecond:Float = 60, broadPhaseType:Int = BroadPhase.BROAD_PHASE_SWEEP_AND_PRUNE) {
        trace("OimoPhysics *** Copyright (c) 2012-2013 EL-EMENT saharan");
        timeStep = 1 / stepPerSecond;
        switch (broadPhaseType) {
            case BroadPhase.BROAD_PHASE_BRUTE_FORCE:
                broadPhase = new BruteForceBroadPhase();
				
            case BroadPhase.BROAD_PHASE_SWEEP_AND_PRUNE:
                broadPhase = new SAPBroadPhase();
				
            case BroadPhase.BROAD_PHASE_DYNAMIC_BOUNDING_VOLUME_TREE:
                broadPhase = new DBVTBroadPhase();
				
            default:
                //throw new Error("Invalid type.");
        }
		
        numIterations = 8;
        gravity = new Vec3(gravityX, gravityY, gravityZ);
        performance = new Performance();
        var numShapeTypes:Int = 3;
        detectors = new Array<Array<CollisionDetector>>();
		
        for (i in 0...numShapeTypes){
            detectors[i] = new Array<CollisionDetector>();
        }
		
        detectors[Shape.SHAPE_SPHERE][Shape.SHAPE_SPHERE] = new SphereSphereCollisionDetector();
        detectors[Shape.SHAPE_SPHERE][Shape.SHAPE_BOX] = new SphereBoxCollisionDetector(false);
        detectors[Shape.SHAPE_BOX][Shape.SHAPE_SPHERE] = new SphereBoxCollisionDetector(true);
        detectors[Shape.SHAPE_BOX][Shape.SHAPE_BOX] = new BoxBoxCollisionDetector();
		
        randX = 65535;
        randA = 98765;
        randB = 123456789;
        maxIslandRigidBodies = 64;
        islandRigidBodies = new Array<RigidBody>();
        islandStack = new Array<RigidBody>();
        maxIslandConstraints = 128;
        islandConstraints = new Array<Constraint>();
        enableRandomizer = true;
    }
    
    /**
	 * Reset the randomizer and remove all rigid bodies, shapes, joints and any object from the world.
	 */
    public function clear() {
        randX = 65535;
        while (joints != null){
            removeJoint(joints);
        }
        while (contacts != null){
            removeContact(contacts);
        }
        while (rigidBodies != null){
            removeRigidBody(rigidBodies);
        }
    }
    
    /**
	 * ワールドに剛体を追加します。
	 * 追加された剛体はステップ毎の演算対象になります。
	 * @param	rigidBody 追加する剛体
	 */
    public function addRigidBody(rigidBody:RigidBody) {
        if (rigidBody.parent != null) {
            trace("Body already has a parent!");
			return;
        }
        rigidBody.parent = this;
        rigidBody.awake();
        var shape:Shape = rigidBody.shapes;
		
        while (shape != null) {
            addShape(shape);
            shape = shape.next;
        }
		
        if (rigidBodies != null) {
			(rigidBodies.prev = rigidBody).next = rigidBodies;
		}
		
        rigidBodies = rigidBody;
        numRigidBodies++;
    }
    
    /**
	 * ワールドから剛体を削除します。
	 * 削除された剛体はステップ毎の演算対象から外されます。
	 * @param	rigidBody 削除する剛体
	 */
    public function removeRigidBody(rigidBody:RigidBody) {
        var remove:RigidBody = rigidBody;
        if (remove.parent != this) {
			return;
		}
		
        remove.awake();
        var js:JointLink = remove.jointLink;
        while (js != null) {
            var joint:Joint = js.joint;
            js = js.next;
            removeJoint(joint);
        }
		
        var shape:Shape = rigidBody.shapes;
        while (shape != null) {
            removeShape(shape);
            shape = shape.next;
        }
		
        var prev:RigidBody = remove.prev;
        var next:RigidBody = remove.next;
        if (prev != null) {
			prev.next = next;
		}
        if (next != null) {
			next.prev = prev;
		}
        if (rigidBodies == remove) {
			rigidBodies = next;
		}
		
        remove.prev = null;
        remove.next = null;
        remove.parent = null;
        numRigidBodies--;
        
        remove.setupMass();
        remove.position.x = 9999;
        remove.position.y = 9999;
        remove.position.z = 9999;
    }
    
    /**
	 * ワールドに形状を追加します。
	 * <strong>剛体をワールドに追加、およびワールドに追加されている剛体に形状を追加すると、
	 * 自動で形状もワールドに追加されるので、このメソッドは外部から呼ばないでください。</strong>
	 * @param	shape 追加する形状
	 */
    public function addShape(shape:Shape) {
        if (shape.parent == null || shape.parent.parent == null) {
            //throw new Error("ワールドに形状を単体で追加することはできません");
        }
        shape.proxy = broadPhase.createProxy(shape);
        shape.updateProxy();
        broadPhase.addProxy(shape.proxy);
    }
    
    /**
	 * ワールドから形状を削除します。
	 * <strong>剛体をワールドから削除、およびワールドに追加されている剛体から形状を削除すると、
	 * 自動で形状もワールドから削除されるので、このメソッドは外部から呼ばないでください。</strong>
	 * @param	shape 削除する形状
	 */
    public function removeShape(shape:Shape) {
        broadPhase.removeProxy(shape.proxy);
        shape.proxy = null;
    }
    
    /**
	 * ワールドにジョイントを追加します。
	 * 追加されたジョイントはステップ毎の演算対象になります。
	 * @param	joint 追加するジョイント
	 */
    public function addJoint(joint:Joint) {
        if (joint.parent != null) {
            //throw new Error("一つのジョイントを複数ワールドに追加することはできません");
        }
        if (joints != null) {
			(joints.prev = joint).next = joints;
		}
        joints = joint;
        joint.parent = this;
        numJoints++;
        joint.awake();
        joint.attach();
    }
    
    /**
	 * ワールドからジョイントを削除します。
	 * 削除されたジョイントはステップ毎の演算対象から外されます。
	 * @param	joint 削除するジョイント
	 */
    public function removeJoint(joint:Joint) {
        var remove:Joint = joint;
        var prev:Joint = remove.prev;
        var next:Joint = remove.next;
        if (prev != null) {
			prev.next = next;
		}
        if (next != null) {
			next.prev = prev;
		}
        if (joints == remove) {
			joints = next;
		}
        remove.prev = null;
        remove.next = null;
        numJoints--;
        remove.awake();
        remove.detach();
        remove.parent = null;
    }
    
    /**
	 * ワールドの時間をタイムステップ秒だけ進めます。
	 */
    inline public function step(dt:Float) {
        timeStep = dt;
        var time1:Int = Math.round(haxe.Timer.stamp() * 1000);
        var body:RigidBody = rigidBodies;
		var lv:Vec3 = null;
		var p:Vec3 = null;
		var sp:Vec3 = null;
		var o:Quat = null;
		var so:Quat = null;
        while (body != null) {	
            if (body.prestep != null) {
				body.prestep();
			}
            body.addedToIsland = false;
            if (body.sleeping) {
                lv = body.linearVelocity;
                p = body.position;
                sp = body.sleepPosition;
                o = body.orientation;
                so = body.sleepOrientation;
                if (
                    lv.x != 0 || lv.y != 0 || lv.z != 0 ||
                    p.x != sp.x || p.y != sp.y || p.z != sp.z ||
                    o.s != so.s || o.x != so.x || o.y != so.y || o.z != so.z) {  // awake the body  
                    body.awake();
                }
            }
            body = body.next;
        }
		
        updateContacts();
        solveIslands();
        var time2:Int = Math.round(haxe.Timer.stamp() * 1000);
        performance.totalTime = time2 - time1;
        performance.updatingTime = performance.totalTime - (performance.broadPhaseTime + performance.narrowPhaseTime + performance.solvingTime);
    }
    
    inline private function updateContacts() {
        // broad phase
        var time1:Int = Math.round(haxe.Timer.stamp() * 1000);
        broadPhase.detectPairs();
        var pairs:Array<Pair> = broadPhase.pairs;
        var numPairs:Int = broadPhase.numPairs;
		
        for (i in 0...numPairs) {
            var pair:Pair = pairs[i];
            var s1:Shape;
            var s2:Shape;
            if (pair.shape1.id < pair.shape2.id) {
                s1 = pair.shape1;
                s2 = pair.shape2;
            }
            else {
                s1 = pair.shape2;
                s2 = pair.shape1;
            }
            var link:ContactLink;
            if (s1.numContacts < s2.numContacts) {
                link = s1.contactLink;
            }
            else {
                link = s2.contactLink;
            }
            var exists:Bool = false;
            while (link != null){
                var contact:Contact = link.contact;
                if (contact.shape1 == s1 && contact.shape2 == s2) {
                    contact.persisting = true;
                    exists = true;  // contact already exists  
                    break;
                }
                link = link.next;
            }
            if (!exists) {
                addContact(s1, s2);
            }
        }
        
        var time2:Int = Math.round(haxe.Timer.stamp() * 1000);
        performance.broadPhaseTime = time2 - time1;
        
        // update & narrow phase
        numContactPoints = 0;
        var contact = contacts;
        while (contact != null) {
            if (!contact.persisting) {
                var aabb1:AABB = contact.shape1.aabb;
                var aabb2:AABB = contact.shape2.aabb;
                if (
                    aabb1.minX > aabb2.maxX || aabb1.maxX < aabb2.minX ||
                    aabb1.minY > aabb2.maxY || aabb1.maxY < aabb2.minY ||
                    aabb1.minZ > aabb2.maxZ || aabb1.maxZ < aabb2.minZ) {
                    var next:Contact = contact.next;
                    removeContact(contact);
                    contact = next;
                    continue;
                }
            }
			
            var b1:RigidBody = contact.body1;
            var b2:RigidBody = contact.body2;
            if (b1.isDynamic && !b1.sleeping || b2.isDynamic && !b2.sleeping) {
                contact.updateManifold();
            }
            numContactPoints += contact.manifold.numPoints;
            contact.persisting = false;
            contact.constraint.addedToIsland = false;
            contact = contact.next;
        }
        
        var time3:Int = Math.round(haxe.Timer.stamp() * 1000);
        performance.narrowPhaseTime = time3 - time2;
    }
    
    private function addContact(s1:Shape, s2:Shape) {
        var newContact:Contact;
        if (unusedContacts != null) {
            newContact = unusedContacts;
            unusedContacts = unusedContacts.next;
        }
        else {
            newContact = new Contact();
        }
        newContact.attach(s1, s2);
        newContact.detector = detectors[s1.type][s2.type];
        if (contacts != null) {
			(contacts.prev = newContact).next = contacts;
		}
        contacts = newContact;
        numContacts++;
    }
    
    private function removeContact(contact:Contact) {
        var prev:Contact = contact.prev;
        var next:Contact = contact.next;
        if (next != null) {
			next.prev = prev;
		}
        if (prev != null) {
			prev.next = next;
		}
        if (contacts == contact) {
			contacts = next;
		}
		
        contact.prev = null;
        contact.next = null;
        contact.detach();
        contact.next = unusedContacts;
        unusedContacts = contact;
        numContacts--;
    }

    public function checkContact(name1:String, name2:String):Bool {
        if (getContact(name1, name2) != null) {
			return true;
		}
        
        return false;
    }

    public function getContact(name1:String, name2:String):Contact {
        var n1:String = "";
		var n2:String = "";
        var contact = this.contacts;
        while (contact != null) {
            n1 = contact.body1.name;
            n2 = contact.body2.name;
            if((n1 == name1 && n2 == name2) || (n2 == name1 && n1 == name2)) {
                if (contact.touching) {
					return contact;
				}
                else {
					return null;
				}
            }
            else {
				contact = contact.next;
			}
        }
        return null;
    }
    
    private function calSleep(body:RigidBody):Bool {
        if (!body.allowSleep) {
			return false;
		}
        var v:Vec3 = body.linearVelocity;
        if (v.x * v.x + v.y * v.y + v.z * v.z > 0.04) {
			return false;
		}
        v = body.angularVelocity;
        if (v.x * v.x + v.y * v.y + v.z * v.z > 0.25) {
			return false;
		}
        return true;
    }
    
    private function solveIslands() {
        var invTimeStep:Float = 1 / timeStep;
        var body:RigidBody;
        var joint:Joint;
        var constraint:Constraint;
        //var num : Int;
        
        joint = joints;
        while (joint != null) {
            joint.addedToIsland = false;
            joint = joint.next;
        }  // expand island buffers         
        
        if (maxIslandRigidBodies < numRigidBodies) {
            maxIslandRigidBodies = numRigidBodies << 1;
            islandRigidBodies = new Array<RigidBody>();
            islandStack = new Array<RigidBody>();
        }
        var numConstraints:Int = numJoints + numContacts;
        if (maxIslandConstraints < numConstraints) {
            maxIslandConstraints = numConstraints << 1;
            islandConstraints = new Array<Constraint>();
        }
        
        var time1:Int = Math.round(haxe.Timer.stamp() * 1000);
        numIslands = 0;
        // build and solve simulation islands
        var base:RigidBody = rigidBodies;
        while (base != null) {
            if (base.addedToIsland || base.isStatic || base.sleeping) {
                base = base.next;
				continue;
            }
            if (base.isLonely()) {  // update single body  
                if (base.isDynamic) {
                    base.linearVelocity.x += gravity.x * timeStep;
                    base.linearVelocity.y += gravity.y * timeStep;
                    base.linearVelocity.z += gravity.z * timeStep;
                }
                if (calSleep(base)) {
                    base.sleepTime += timeStep;
                    if (base.sleepTime > 0.5) {
                        base.sleep();
                    }
                    else {
                        base.updatePosition(timeStep);
                    }
                }
                else {
                    base.sleepTime = 0;
                    base.updatePosition(timeStep);
                }
                numIslands++;
                base = base.next;
				continue;
                
            }
            var islandNumRigidBodies:Int = 0;
            var islandNumConstraints:Int = 0;
            var stackCount:Int = 1;
            // add rigid body to stack
            islandStack[0] = base;
            base.addedToIsland = true;
            // build an island
            do {
                // get rigid body from stack
                if (stackCount == 0) {
					break; // TODO
				}
                body = islandStack[--stackCount];
                islandStack[stackCount] = null;  // gc
                body.sleeping = false;
                // add rigid body to the island
                islandRigidBodies[islandNumRigidBodies++] = body;
                if (body.isStatic) {
                    //base = base.next; // TODO
                    continue;
                }
				
                // search connections  
                var cs:ContactLink = body.contactLink;
                while (cs != null) {
                    var contact:Contact = cs.contact;
                    constraint = contact.constraint;
                    if (constraint.addedToIsland || !contact.touching) {
                        cs = cs.next;
                        continue;
                    } 
                    // add constraint to the island 
                    islandConstraints[islandNumConstraints++] = constraint;
                    constraint.addedToIsland = true;
                    var next:RigidBody = cs.body;
                    if (next.addedToIsland) {
                        cs = cs.next;
                        continue;
                    } 
                    // add rigid body to stack 
                    islandStack[stackCount++] = next;
                    next.addedToIsland = true;
                    cs = cs.next;
                }
				
                var js:JointLink = body.jointLink;
                while (js != null){
                    constraint = js.joint;
                    if (constraint.addedToIsland) {
                        js = js.next;
                        continue;
                    }
					
                    // add constraint to the island
                    islandConstraints[islandNumConstraints++] = constraint;
                    constraint.addedToIsland = true;
                    var next = js.body;
                    if (next.addedToIsland || !next.isDynamic) {
                        js = js.next;
                        continue;
                    }
					
                    // add rigid body to stack
                    islandStack[stackCount++] = next;
                    next.addedToIsland = true;
                    js = js.next;
                }
            } while (stackCount != 0);
            
            // update velocities
            var gx:Float = gravity.x * timeStep;
            var gy:Float = gravity.y * timeStep;
            var gz:Float = gravity.z * timeStep;
            for (j in 0...islandNumRigidBodies) {
                body = islandRigidBodies[j];
                if (body.isDynamic) {
                    body.linearVelocity.x += gx;
                    body.linearVelocity.y += gy;
                    body.linearVelocity.z += gz;
                }
            }  // randomizing order            
            
            if (enableRandomizer) {
                for (j in 1...islandNumConstraints) {
                    randX = (randX * randA + randB & 0x7fffffff);
                    var swap:Int = Std.int((randX) / 2147483648.0 * j) | 0;
                    constraint = islandConstraints[j];
                    islandConstraints[j] = islandConstraints[swap];
                    islandConstraints[swap] = constraint;
                }
            }  // solve contraints             
            
            for (j in 0...islandNumConstraints) {
                islandConstraints[j].preSolve(timeStep, invTimeStep);
            }
            for (k in 0...numIterations){
                for (j in 0...islandNumConstraints) {
                    islandConstraints[j].solve();
                }
            }
            for (j in 0...islandNumConstraints) {
                islandConstraints[j].postSolve();  // post-solve  
                islandConstraints[j] = null;
            }  // sleeping check  
            
            var sleepTime:Float = 10;
            for (j in 0...islandNumRigidBodies) {
                body = islandRigidBodies[j];
                if (calSleep(body)) {
                    body.sleepTime += timeStep;
                    if (body.sleepTime < sleepTime) {
						sleepTime = body.sleepTime;
					}
                }
                else {
                    body.sleepTime = 0;
                    sleepTime = 0;
                    continue; 
                }
            }
            if (sleepTime > 0.5) {
                // sleep the island
                for (j in 0...islandNumRigidBodies) {
                    islandRigidBodies[j].sleep();
                    islandRigidBodies[j] = null;
                }
            }
            else {
                // update positions
                for (j in 0...islandNumRigidBodies) {
                    islandRigidBodies[j].updatePosition(timeStep);
                    islandRigidBodies[j] = null;
                }
            }
            numIslands++;
            if (base != null) {
				/*TODO*/ 
				base = base.next;
			}
        }
        var time2:Int = Math.round(haxe.Timer.stamp() * 1000);
        performance.solvingTime = time2 - time1;
    }
	
}

