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

import oimohx.math.Mat44;
import oimohx.physics.dynamics.World;

import oimohx.math.Mat33;
import oimohx.math.Quat;
import oimohx.math.Vec3;
import oimohx.physics.collision.shape.MassInfo;
import oimohx.physics.collision.shape.Shape;
import oimohx.physics.constraint.contact.ContactLink;
import oimohx.physics.constraint.joint.JointLink;
import com.babylonhx.utils.typedarray.Float32Array;

/**
 * 剛体のクラスです。
 * 剛体は衝突処理用に単数あるいは複数の形状を持ち、
 * それぞれ個別にパラメーターを設定できます。
 * @author saharan
 */
class RigidBody {
	
    /**
	 * 動的な剛体を表す剛体の種類です。
	 */
    public static inline var BODY_DYNAMIC:Int = 0x1;
    
    /**
	 * 静的な剛体を表す剛体の種類です。
	 */
    public static inline var BODY_STATIC:Int = 0x2;
    
    /**
	 * 一つの剛体に追加できる形状の最大数です。
	 */
    public static inline var MAX_SHAPES:Int = 64;
    
    public var prev:RigidBody;
    public var next:RigidBody;
    
    /**
	 * 剛体の種類を表します。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 * 
	 * 剛体の種類を変更する場合は、必ず
	 * setupMass メソッドの引数に設定したい種類を指定してください。
	 */
    public var name:String = "";
    public var type:Int;
    
    /**
	 * この剛体が動的な剛体であるかどうかを示します。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var isDynamic:Bool;
    
    /**
	 * この剛体が静的な剛体であるかどうかを示します。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var isStatic:Bool;
    
    /**
	 * 重心のワールド座標です。
	 */
    public var position:Vec3 = new Vec3(0, 0, 0);
    
    /**
	 * 姿勢を表すクォータニオンです。
	 */
    public var orientation:Quat;
    
    /**
	 * スリープ直前での重心のワールド座標です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var sleepPosition:Vec3;
    
    /**
	 * スリープ直前での姿勢を表すクォータニオンです。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var sleepOrientation:Quat;
    
    /**
	 * 並進速度です。
	 */
    public var linearVelocity:Vec3;
    
    /**
	 * 角速度です。
	 */
    public var angularVelocity:Vec3;
    
    /**
	 * 姿勢を表す回転行列です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 * 
	 * 回転行列は、ステップ毎にクォータニオンから再計算されます。
	 */
    public var rotation:Mat33;
	
	public var matrix:Mat44;

	public var newRotation:Vec3;
	public var newPosition:Vec3;
	public var newOrientation:Quat;
	
	public var controlPos:Bool = false;
	public var controlRot:Bool = false;
    
    /**
	 * 質量です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 * 
	 * 質量は setupMass メソッドを呼び出すと、
	 * 含まれている形状から自動で再計算されます。
	 */
    public var mass:Float;
    
    /**
	 * 質量の逆数です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 * 
	 * 質量は setupMass メソッドを呼び出すと、
	 * 含まれている形状から自動で再計算されます。
	 */
    public var inverseMass:Float;
    
    /**
	 * ワールド系での慣性テンソルの逆行列です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 * 
	 * ワールド系での慣性テンソルの逆行列は、ステップ毎に
	 * 姿勢と初期状態の慣性テンソルの逆数から再計算されます。
	 */
    public var inverseInertia:Mat33;
    
    /**
	 * 初期状態での慣性テンソルです。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 * 
	 * 慣性テンソルは setupMass メソッドを呼び出すと、
	 * 含まれている形状から自動で再計算されます。
	 */
    public var localInertia:Mat33;
    
    /**
	 * 初期状態での慣性テンソルの逆行列です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 * 
	 * 慣性テンソルは setupMass メソッドを呼び出すと、
	 * 含まれている形状から自動で再計算されます。
	 */
    public var inverseLocalInertia:Mat33;
    
    /**
	 * 剛体に含まれている形状の配列です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var shapes:Shape;
    
    /**
	 * 剛体に含まれている形状の数です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var numShapes:Int = 0;
    
    /**
	 * 剛体が追加されているワールドです。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var parent:World;
    
    public var contactLink:ContactLink;
    public var numContacts:Int;
    
    /**
	 * 剛体に接続されているジョイントのリンク配列です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var jointLink:JointLink;
    
    /**
	 * 剛体に接続されているジョイントの数です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var numJoints:Int;
    
    /**
	 * 剛体がシミュレーションアイランドに追加されたかどうかを示します。
	 * この変数は内部でのみ使用されます。
	 */
    public var addedToIsland:Bool;
    
    /**
	 * 剛体が静止してからの時間です。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 */
    public var sleepTime:Float;
    
    /**
	 * 剛体がスリープ状態であるかどうかを示します。
	 * <strong>この変数は外部から変更しないでください。</strong>
	 * 剛体をスリープさせる場合は sleep メソッドを、
	 * 剛体のスリープ状態を解除する場合は awake メソッドを呼び出してください。
	 */
    public var sleeping:Bool;
    
    /**
	 * 剛体をスリープさせるかを示します。
	 * シミュレーションアイランド内の全ての剛体が静止している状態が一定時間続くと、
	 * そのシミュレーションアイランドはスリープ状態に入ります。
	 * スリープしている剛体は awake メソッドが呼び出されるか、
	 * 外部からの干渉を受けるまで、スリープ状態が解除されることはありません。
	 */
    public var allowSleep:Bool;
    
    private var massInfo:MassInfo = new MassInfo();

    public var prestep:Void->Void;
    
    /**
	 * 新しく RigidBody オブジェクトを作成します。
	 * 回転成分を指定することもできます。
	 * @param	x
	 * @param	y
	 * @param	z
	 * @param	rad ラジアンでの回転角度
	 * @param	ax 回転軸の x 成分
	 * @param	ay 回転軸の y 成分
	 * @param	az 回転軸の z 成分
	 */
    public function new(x:Float = 0, y:Float = 0, z:Float = 0, rad:Float = 0, ax:Float = 0, ay:Float = 0, az:Float = 0) {
        position = new Vec3(x, y, z);
        var len:Float = ax * ax + ay * ay + az * az;
        if (len > 0) {
            len = 1 / Math.sqrt(len);
            ax *= len;
            ay *= len;
            az *= len;
        }
        var sin:Float = Math.sin(rad * 0.5);
        var cos:Float = Math.cos(rad * 0.5);
        orientation = new Quat(cos, sin * ax, sin * ay, sin * az);
        linearVelocity = new Vec3();
        angularVelocity = new Vec3();
        sleepPosition = new Vec3();
        sleepOrientation = new Quat();
        rotation = new Mat33();
        inverseInertia = new Mat33();
        localInertia = new Mat33();
        inverseLocalInertia = new Mat33();
        allowSleep = true;
        sleepTime = 0;
		
		newOrientation = new Quat();
		newRotation = new Vec3(0, 0, 0);
		newPosition = new Vec3(0, 0, 0);
		matrix = new Mat44();
    }
    
    /**
	 * 剛体に形状を追加します。
	 * 形状を追加した場合、次のステップ開始までに setupMass メソッドを呼び出してください。
	 * @param	shape 追加する形状
	 */
    public function addShape(shape:Shape) {
		// TODO: temp fix for BabylonHx
		if(shape != null) {
			if (shape.parent != null) {
				throw("Shape already has a parent!");
			}
			if (shapes != null) {
				(shapes.prev = shape).next = shapes;
			}
			shapes = shape;
			shape.parent = this;
			if (parent != null) {
				parent.addShape(shape);
			}
			numShapes++;
		}
    }
    
    /**
	 * 剛体から形状を削除します。
	 * 形状を削除した場合、次のステップ開始までに setupMass メソッドを呼び出してください。
	 * @param	shape 削除する形状
	 */
    public function removeShape(shape:Shape) {
        var remove:Shape = shape;
        if (remove.parent != this) {
            return;
		}
        var prev:Shape = remove.prev;
        var next:Shape = remove.next;
        if (prev != null) {
            prev.next = next;
		}
        if (next != null) {
            next.prev = prev;
		}
        if (shapes == remove) {
			shapes = next;
		}
        remove.prev = null;
        remove.next = null;
        remove.parent = null;
        if (parent != null) {
            parent.removeShape(remove);
		}
        numShapes--;
    }
    
    /**
	 * Calulates mass datas(center of gravity, mass, moment inertia, etc...).
	 * If the parameter type is set to BODY_STATIC, the rigid body will be fixed to the space.
	 * If the parameter adjustPosition is set to true, the shapes' relative positions and
	 * the rigid body's position will be adjusted to the center of gravity.
	 * @param	type
	 * @param	adjustPosition
	 */
    public function setupMass(type:Int = RigidBody.BODY_DYNAMIC, adjustPosition:Bool = true) {
        this.type = type;
        isDynamic = type == RigidBody.BODY_DYNAMIC;
        isStatic = type == RigidBody.BODY_STATIC;
        mass = 0;
        localInertia.init(0, 0, 0, 0, 0, 0, 0, 0, 0);
        var tmpM:Mat33 = new Mat33();
        var tmpV:Vec3 = new Vec3();
        var shape:Shape = shapes;
        while (shape != null) {
            shape.calculateMassInfo(massInfo);
            var shapeMass:Float = massInfo.mass;
            var relX:Float = shape.relativePosition.x;
            var relY:Float = shape.relativePosition.y;
            var relZ:Float = shape.relativePosition.z;
            
            tmpV.x += relX * shapeMass;
            tmpV.y += relY * shapeMass;
            tmpV.z += relZ * shapeMass;
            mass += shapeMass;
            
            rotateInertia(shape.relativeRotation, massInfo.inertia, tmpM);
            localInertia.addEqual(tmpM);
            
            // add offset inertia
            localInertia.elements[0] += shapeMass * (relY * relY + relZ * relZ);
            localInertia.elements[1] += shapeMass * (relX * relX + relZ * relZ);
            localInertia.elements[2] += shapeMass * (relX * relX + relY * relY);
            var xy:Float = shapeMass * relX * relY;
            var yz:Float = shapeMass * relY * relZ;
            var zx:Float = shapeMass * relZ * relX;
            localInertia.elements[3] -= xy;
            localInertia.elements[4] -= xy;
            localInertia.elements[5] -= yz;
            localInertia.elements[6] -= yz;
            localInertia.elements[7] -= zx;
            localInertia.elements[8] -= zx;
            shape = shape.next;
        }
        inverseMass = 1 / mass;
        tmpV.scaleEqual(inverseMass);
        
        if (adjustPosition) {
            position.addEqual(tmpV);
            shape = shapes;
            while (shape != null) {
                shape.relativePosition.subEqual(tmpV);
                shape = shape.next;
            }  // subtract offset inertia  
            
            var relX = tmpV.x;
            var relY = tmpV.y;
            var relZ = tmpV.z;
            localInertia.elements[0] -= mass * (relY * relY + relZ * relZ);
            localInertia.elements[1] -= mass * (relX * relX + relZ * relZ);
            localInertia.elements[2] -= mass * (relX * relX + relY * relY);
            var xy = mass * relX * relY;
            var yz = mass * relY * relZ;
            var zx = mass * relZ * relX;
            localInertia.elements[3] += xy;
            localInertia.elements[4] += xy;
            localInertia.elements[5] += yz;
            localInertia.elements[6] += yz;
            localInertia.elements[7] += zx;
            localInertia.elements[8] += zx;
        }
        
        inverseLocalInertia.invert(localInertia);
        
        if (type == RigidBody.BODY_STATIC) {
            inverseMass = 0;
            inverseLocalInertia.init(0, 0, 0, 0, 0, 0, 0, 0, 0);
        }
        
        syncShapes();
        awake();
    }
    
    /**
	 * Awake the rigid body.
	 */
    inline public function awake() {
        if (!allowSleep || !sleeping) {
            return;
		}
        sleeping = false;
        sleepTime = 0;
        // awake connected constraints
        var cs:ContactLink = contactLink;
        while (cs != null) {
            cs.body.sleepTime = 0;
            cs.body.sleeping = false;
            cs = cs.next;
        }
        var js:JointLink = jointLink;
        while (js != null) {
            js.body.sleepTime = 0;
            js.body.sleeping = false;
            js = js.next;
        }
        var shape:Shape = shapes;
        while (shape != null) {
            shape.updateProxy();
            shape = shape.next;
        }
    }
    
    /**
	 * Sleep the rigid body.
	 */
    inline public function sleep() {
        if (!allowSleep || sleeping) {
            return;
		}
        linearVelocity.x = 0;
        linearVelocity.y = 0;
        linearVelocity.z = 0;
        angularVelocity.x = 0;
        angularVelocity.y = 0;
        angularVelocity.z = 0;
        sleepPosition.x = position.x;
        sleepPosition.y = position.y;
        sleepPosition.z = position.z;
        sleepOrientation.s = orientation.s;
        sleepOrientation.x = orientation.x;
        sleepOrientation.y = orientation.y;
        sleepOrientation.z = orientation.z;
        sleepTime = 0;
        sleeping = true;
        var shape:Shape = shapes;
        while (shape != null) {
            shape.updateProxy();
            shape = shape.next;
        }
    }
    
    /**
	 * Get whether the rigid body has not any connection with others.
	 * @return
	 */
    inline public function isLonely():Bool {
        return numJoints == 0 && numContacts == 0;
    }
    
    /**
	 * 剛体の運動を時間積分し、形状などの情報を更新します。
	 * このメソッドはワールドのステップを呼ぶと自動で呼び出されるので、
	 * 通常は外部から呼ぶ必要はありません。
	 * @param	timeStep 時間刻み幅
	 */
    inline public function updatePosition(timeStep:Float) {
        switch (type) {
            case RigidBody.BODY_STATIC:
                linearVelocity.x = 0;
                linearVelocity.y = 0;
                linearVelocity.z = 0;
                angularVelocity.x = 0;
                angularVelocity.y = 0;
                angularVelocity.z = 0;
				
            case RigidBody.BODY_DYNAMIC:
                var vx:Float = linearVelocity.x;
                var vy:Float = linearVelocity.y;
                var vz:Float = linearVelocity.z;
                position.x += vx * timeStep;
                position.y += vy * timeStep;
                position.z += vz * timeStep;
                vx = angularVelocity.x;
                vy = angularVelocity.y;
                vz = angularVelocity.z;
                var os:Float = orientation.s;
                var ox:Float = orientation.x;
                var oy:Float = orientation.y;
                var oz:Float = orientation.z;
                timeStep *= 0.5;
                var s:Float = (-vx * ox - vy * oy - vz * oz) * timeStep;
                var x:Float = (vx * os + vy * oz - vz * oy) * timeStep;
                var y:Float = (-vx * oz + vy * os + vz * ox) * timeStep;
                var z:Float = (vx * oy - vy * ox + vz * os) * timeStep;
                os += s;
                ox += x;
                oy += y;
                oz += z;
                s = 1 / Math.sqrt(os * os + ox * ox + oy * oy + oz * oz);
                orientation.s = os * s;
                orientation.x = ox * s;
                orientation.y = oy * s;
                orientation.z = oz * s;
				
				/*if(this.controlPos){
                    this.angularVelocity.init();
                    this.linearVelocity.init();
                    this.linearVelocity.x = (this.newPosition.x - this.position.x) / timeStep;
                    this.linearVelocity.y = (this.newPosition.y - this.position.y) / timeStep;
                    this.linearVelocity.z = (this.newPosition.z - this.position.z) / timeStep;
                    this.controlPos = false;
                }
                if(this.controlRot){
                    this.angularVelocity.init();
                    this.orientation.copy(this.newOrientation);
                    
                    this.controlRot = false;
                }

                this.position.addTime(this.linearVelocity, timeStep);
                this.orientation.addTime(this.angularVelocity, timeStep);*/
				
            default:
                //throw new Error("Invalid type.");
        }
        syncShapes();
    }
    
    inline private function rotateInertia(rot:Mat33, inertia:Mat33, out:Mat33) {
		/*var e00:Float = rot.e00 * inertia.e00 + rot.e01 * inertia.e10 + rot.e02 * inertia.e20;
        var e01:Float = rot.e00 * inertia.e01 + rot.e01 * inertia.e11 + rot.e02 * inertia.e21;
        var e02:Float = rot.e00 * inertia.e02 + rot.e01 * inertia.e12 + rot.e02 * inertia.e22;
        var e10:Float = rot.e10 * inertia.e00 + rot.e11 * inertia.e10 + rot.e12 * inertia.e20;
        var e11:Float = rot.e10 * inertia.e01 + rot.e11 * inertia.e11 + rot.e12 * inertia.e21;
        var e12:Float = rot.e10 * inertia.e02 + rot.e11 * inertia.e12 + rot.e12 * inertia.e22;
        var e20:Float = rot.e20 * inertia.e00 + rot.e21 * inertia.e10 + rot.e22 * inertia.e20;
        var e21:Float = rot.e20 * inertia.e01 + rot.e21 * inertia.e11 + rot.e22 * inertia.e21;
        var e22:Float = rot.e20 * inertia.e02 + rot.e21 * inertia.e12 + rot.e22 * inertia.e22;
        out.e00 = e00 * rot.e00 + e01 * rot.e01 + e02 * rot.e02;
        out.e01 = e00 * rot.e10 + e01 * rot.e11 + e02 * rot.e12;
        out.e02 = e00 * rot.e20 + e01 * rot.e21 + e02 * rot.e22;
        out.e10 = e10 * rot.e00 + e11 * rot.e01 + e12 * rot.e02;
        out.e11 = e10 * rot.e10 + e11 * rot.e11 + e12 * rot.e12;
        out.e12 = e10 * rot.e20 + e11 * rot.e21 + e12 * rot.e22;
        out.e20 = e20 * rot.e00 + e21 * rot.e01 + e22 * rot.e02;
        out.e21 = e20 * rot.e10 + e21 * rot.e11 + e22 * rot.e12;
        out.e22 = e20 * rot.e20 + e21 * rot.e21 + e22 * rot.e22;*/
		
        var r00:Float = rot.elements[0];
        var r01:Float = rot.elements[1];
        var r02:Float = rot.elements[2];
        var r10:Float = rot.elements[3];
        var r11:Float = rot.elements[4];
        var r12:Float = rot.elements[5];
        var r20:Float = rot.elements[6];
        var r21:Float = rot.elements[7];
        var r22:Float = rot.elements[8];
        var i00:Float = inertia.elements[0];
        var i01:Float = inertia.elements[1];
        var i02:Float = inertia.elements[2];
        var i10:Float = inertia.elements[3];
        var i11:Float = inertia.elements[4];
        var i12:Float = inertia.elements[5];
        var i20:Float = inertia.elements[6];
        var i21:Float = inertia.elements[7];
        var i22:Float = inertia.elements[8];
        var e00:Float = r00 * i00 + r01 * i10 + r02 * i20;
        var e01:Float = r00 * i01 + r01 * i11 + r02 * i21;
        var e02:Float = r00 * i02 + r01 * i12 + r02 * i22;
        var e10:Float = r10 * i00 + r11 * i10 + r12 * i20;
        var e11:Float = r10 * i01 + r11 * i11 + r12 * i21;
        var e12:Float = r10 * i02 + r11 * i12 + r12 * i22;
        var e20:Float = r20 * i00 + r21 * i10 + r22 * i20;
        var e21:Float = r20 * i01 + r21 * i11 + r22 * i21;
        var e22:Float = r20 * i02 + r21 * i12 + r22 * i22;
        out.elements[0] = e00 * r00 + e01 * r01 + e02 * r02;
        out.elements[1] = e00 * r10 + e01 * r11 + e02 * r12;
        out.elements[2] = e00 * r20 + e01 * r21 + e02 * r22;
        out.elements[3] = e10 * r00 + e11 * r01 + e12 * r02;
        out.elements[4] = e10 * r10 + e11 * r11 + e12 * r12;
        out.elements[5] = e10 * r20 + e11 * r21 + e12 * r22;
        out.elements[6] = e20 * r00 + e21 * r01 + e22 * r02;
        out.elements[7] = e20 * r10 + e21 * r11 + e22 * r12;
        out.elements[8] = e20 * r20 + e21 * r21 + e22 * r22;
		
		/*var tm1 = rot.elements;
        var tm2 = inertia.elements;

        var a0 = tm1[0], a3 = tm1[3], a6 = tm1[6];
        var a1 = tm1[1], a4 = tm1[4], a7 = tm1[7];
        var a2 = tm1[2], a5 = tm1[5], a8 = tm1[8];

        var b0 = tm2[0], b3 = tm2[3], b6 = tm2[6];
        var b1 = tm2[1], b4 = tm2[4], b7 = tm2[7];
        var b2 = tm2[2], b5 = tm2[5], b8 = tm2[8];
        
        var e00 = a0*b0 + a1*b3 + a2*b6;
        var e01 = a0*b1 + a1*b4 + a2*b7;
        var e02 = a0*b2 + a1*b5 + a2*b8;
        var e10 = a3*b0 + a4*b3 + a5*b6;
        var e11 = a3*b1 + a4*b4 + a5*b7;
        var e12 = a3*b2 + a4*b5 + a5*b8;
        var e20 = a6*b0 + a7*b3 + a8*b6;
        var e21 = a6*b1 + a7*b4 + a8*b7;
        var e22 = a6*b2 + a7*b5 + a8*b8;

        var oe = out.elements;
        out.e00 = oe[0] = e00*a0 + e01*a1 + e02*a2;
        out.e01 = oe[1] = e00*a3 + e01*a4 + e02*a5;
        out.e02 = oe[2] = e00*a6 + e01*a7 + e02*a8;
        out.e10 = oe[3] = e10*a0 + e11*a1 + e12*a2;
        out.e11 = oe[4] = e10*a3 + e11*a4 + e12*a5;
        out.e12 = oe[5] = e10*a6 + e11*a7 + e12*a8;
        out.e20 = oe[6] = e20*a0 + e21*a1 + e22*a2;
        out.e21 = oe[7] = e20*a3 + e21*a4 + e22*a5;
        out.e22 = oe[8] = e20*a6 + e21*a7 + e22*a8;*/
    }
    
    inline public function syncShapes() {
        /*var s:Float = orientation.s;
        var x:Float = orientation.x;
        var y:Float = orientation.y;
        var z:Float = orientation.z;
        var x2:Float = 2 * x;
        var y2:Float = 2 * y;
        var z2:Float = 2 * z;
        var xx:Float = x * x2;
        var yy:Float = y * y2;
        var zz:Float = z * z2;
        var xy:Float = x * y2;
        var yz:Float = y * z2;
        var xz:Float = x * z2;
        var sx:Float = s * x2;
        var sy:Float = s * y2;
        var sz:Float = s * z2;
        rotation.e00 = 1 - yy - zz;
        rotation.e01 = xy - sz;
        rotation.e02 = xz + sy;
        rotation.e10 = xy + sz;
        rotation.e11 = 1 - xx - zz;
        rotation.e12 = yz - sx;
        rotation.e20 = xz - sy;
        rotation.e21 = yz + sx;
        rotation.e22 = 1 - xx - yy;
        var r00:Float = rotation.e00;
        var r01:Float = rotation.e01;
        var r02:Float = rotation.e02;
        var r10:Float = rotation.e10;
        var r11:Float = rotation.e11;
        var r12:Float = rotation.e12;
        var r20:Float = rotation.e20;
        var r21:Float = rotation.e21;
        var r22:Float = rotation.e22;
        rotateInertia(rotation, inverseLocalInertia, inverseInertia);
        var shape:Shape = shapes;
        while (shape != null){
            var relPos:Vec3 = shape.relativePosition;
            var relRot:Mat33 = shape.relativeRotation;
            var rot:Mat33 = shape.rotation;
            var lx:Float = relPos.x;
            var ly:Float = relPos.y;
            var lz:Float = relPos.z;
            shape.position.x = position.x + lx * r00 + ly * r01 + lz * r02;
            shape.position.y = position.y + lx * r10 + ly * r11 + lz * r12;
            shape.position.z = position.z + lx * r20 + ly * r21 + lz * r22;
            rot.mul(relRot, rotation);
            shape.updateProxy();
            shape = shape.next;
        }*/
		
		var s = this.orientation.s;
        var x = this.orientation.x;
        var y = this.orientation.y;
        var z = this.orientation.z;
        var x2 = 2 * x;
        var y2 = 2 * y;
        var z2 = 2 * z;
        var xx = x * x2;
        var yy = y * y2;
        var zz = z * z2;
        var xy = x * y2;
        var yz = y * z2;
        var xz = x * z2;
        var sx = s * x2;
        var sy = s * y2;
        var sz = s * z2;
		
        var tr = this.rotation.elements;
        this.rotation.elements[0] = tr[0] = 1 - yy - zz;
        this.rotation.elements[1] = tr[1] = xy - sz;
        this.rotation.elements[2] = tr[2] = xz + sy;
        this.rotation.elements[3] = tr[3] = xy + sz;
        this.rotation.elements[4] = tr[4] = 1 - xx - zz;
        this.rotation.elements[5] = tr[5] = yz - sx;
        this.rotation.elements[6] = tr[6] = xz - sy;
        this.rotation.elements[7] = tr[7] = yz + sx;
        this.rotation.elements[8] = tr[8] = 1 - xx - yy;
		
        this.rotateInertia(this.rotation,this.inverseLocalInertia,this.inverseInertia);
        var shape:Shape = shapes;
        while (shape != null) {
            shape.position.mul(this.position, shape.relativePosition, this.rotation);
            shape.rotation.mul(shape.relativeRotation, this.rotation);
            shape.updateProxy();
			shape = shape.next;
        }
    }
    
    inline public function applyImpulse(position:Vec3, force:Vec3) {
        linearVelocity.x += force.x * inverseMass;
        linearVelocity.y += force.y * inverseMass;
        linearVelocity.z += force.z * inverseMass;
        var rel:Vec3 = new Vec3();
        rel.sub(position, this.position).cross(rel, force).mulMat(inverseInertia, rel);
		angularVelocity.addEqual(rel);
    }

    inline public function setImpulse(position:Vec3, force:Vec3) {
        linearVelocity.x = force.x * inverseMass;
        linearVelocity.y = force.y * inverseMass;
        linearVelocity.z = force.z * inverseMass;
        var rel:Vec3 = new Vec3();
        rel.sub(position, this.position).cross(rel, force).mulMat(inverseInertia, rel);
        angularVelocity.x = rel.x;
        angularVelocity.y = rel.y;
        angularVelocity.z = rel.z;
    }
	
	//---------------------------------------------
    //
    // FOR BabylonHx
    //
    //---------------------------------------------

    inline public function rotationVectToQuad(rot:Vec3):Quat {
        var r = com.babylonhx.physics.plugins.Body.EulerToAxis(rot.x * World.TO_RAD, rot.y * World.TO_RAD, rot.z * World.TO_RAD);
        return this.rotationAxisToQuad(r[0], r[1], r[2], r[3]);
    }

    inline public function rotationAxisToQuad(rad:Float, ax:Float, ay:Float, az:Float):Quat { // in radian
        var len = ax * ax + ay * ay + az * az; 
        if (len > 0) {	
            len = 1 / Math.sqrt(len);
            ax *= len;
            ay *= len;
            az *= len;
        }
        var sin = Math.sin(rad * 0.5);
        var cos = Math.cos(rad * 0.5);
        return new Quat(cos, sin * ax, sin * ay, sin * az);
    }

    //---------------------------------------------
    // SET DYNAMIQUE POSITION AND ROTATION
    //---------------------------------------------

    inline public function setPosition(pos:Vec3) {
        this.newPosition.init(pos.x * World.INV_SCALE, pos.y * World.INV_SCALE, pos.z * World.INV_SCALE);
        this.controlPos = true;
    }

    inline public function setQuaternion(q:Quat) { 
        this.newOrientation.init(q.s, q.x, q.y, q.z);
        this.controlRot = true;
    }

    inline public function setRotation(rot:Vec3) { 
        this.newOrientation = this.rotationVectToQuad(rot);
        this.controlRot = true;
    }

    //---------------------------------------------
    // RESET DYNAMIQUE POSITION AND ROTATION
    //---------------------------------------------

    inline public function resetPosition(x:Float, y:Float, z:Float) {	
        this.linearVelocity.init();
        this.angularVelocity.init();
        this.position.init(x * World.INV_SCALE, y * World.INV_SCALE, z * World.INV_SCALE);
        this.awake();
    }
	
    inline public function resetQuaternion(q:Quat) {
        this.angularVelocity.init();
        this.orientation = new Quat(q.s, q.x, q.y, q.z);
        this.awake();
    }
	
    inline public function resetRotation(x:Float, y:Float, z:Float) {	
        this.angularVelocity.init();
        this.orientation = this.rotationVectToQuad(new Vec3(x, y, z));
        this.awake();
    }

    //---------------------------------------------
    // GET POSITION AND ROTATION
    //---------------------------------------------

    inline public function getPosition():Vec3 {
        return new Vec3().scale(this.position, World.WORLD_SCALE);
    }
	
    /*inline public function getRotation():Euler {
        return new Euler().setFromRotationMatrix(this.rotation);
    }
	
    inline public function getQuaternion():Quat {
        return new Quat().setFromRotationMatrix(this.rotation);
    }*/
    inline public function getMatrix(): #if !html5 Array<Float> #else Float32Array #end {
        var m = this.matrix.elements;
		#if html5
		var r:Float32Array = new Float32Array(9);
		#else
        var r:Array<Float> = [];
		#end
		var p:Vec3 = null;
        if(!this.sleeping){
            // rotation matrix
            r = this.rotation.elements;
            m[0] = r[0]; m[1] = r[3]; m[2] = r[6];  m[3] = 0;
            m[4] = r[1]; m[5] = r[4]; m[6] = r[7];  m[7] = 0;
            m[8] = r[2]; m[9] = r[5]; m[10] = r[8]; m[11] = 0;
			
            // position matrix
            p = this.position;
            m[12] = p.x * World.WORLD_SCALE;
            m[13] = p.y * World.WORLD_SCALE;
            m[14] = p.z * World.WORLD_SCALE;
			
            // sleep or not ?
            m[15] = 0;
        } else {
            m[15] = 1;
        }
		
        return m;
    }
	
}
