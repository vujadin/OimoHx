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
package oimohx.physics.collision.broadphase;

import oimohx.physics.collision.broadphase.Pair;
import oimohx.physics.collision.broadphase.Proxy;

import oimohx.physics.collision.shape.Shape;
import oimohx.physics.constraint.joint.Joint;
import oimohx.physics.constraint.joint.JointLink;
import oimohx.physics.dynamics.RigidBody;
/**
	 * The broad-phase is used for collecting all possible pairs for collision.
	 */
class BroadPhase
{
    /**
		 * Brute force broad-phase algorithm.
		 */
    public static inline var BROAD_PHASE_BRUTE_FORCE : Int = 1;
    
    /**
		 * Sweep and prune broad-phase algorithm.
		 */
    public static inline var BROAD_PHASE_SWEEP_AND_PRUNE : Int = 2;
    
    /**
		 * Dynamic bounding volume tree broad-phase algorithm.
		 */
    public static inline var BROAD_PHASE_DYNAMIC_BOUNDING_VOLUME_TREE : Int = 3;
    
    /**
		 * The pairs whose proxies are overlapping.
		 */
    public var pairs : Array<Pair>;
    
    /**
		 * The number of pairs.
		 */
    public var numPairs : Int = 0;
    
    /**
		 * The number of pair checks.
		 */
    public var numPairChecks : Int;
    
    private var bufferSize : Int;
    
    public function new()
    {
        bufferSize = 256;
        pairs = new Array<Pair>();
        for (i in 0...bufferSize){
            pairs[i] = new Pair();
        }
    }
    
    /**
		 * Create a new proxy.
		 * @param	shape
		 * @return
		 */
    public function createProxy(shape : Shape) : Proxy{
        return null;
        //throw new Error("Inheritance error.");
    }
    
    /**
		 * Add the proxy into the broad-phase.
		 * @param	proxy
		 */
    public function addProxy(proxy : Proxy) : Void{
        //throw new Error("Inheritance error.");
    }
    
    /**
		 * Remove the proxy from the broad-phase.
		 * @param	proxy
		 */
    public function removeProxy(proxy : Proxy) : Void{
        //throw new Error("Inheritance error.");
    }
    
    /**
		 * Returns whether the pair is available or not.
		 * @param	s1
		 * @param	s2
		 * @return
		 */
    public function isAvailablePair(s1 : Shape, s2 : Shape) : Bool{
        var b1 : RigidBody = s1.parent;
        var b2 : RigidBody = s2.parent;
        if (
            b1 == b2 ||  // same parents  
            (!b1.isDynamic && !b2.isDynamic) ||  // static or kinematic objects  
            (s1.belongsTo & s2.collidesWith) == 0 ||
            (s2.belongsTo & s1.collidesWith) ==0) { // collision filtering
            return false;
        }
        var js : JointLink;
        if (b1.numJoints < b2.numJoints)             js = b1.jointLink
        else js = b2.jointLink;
        while (js != null){
            var joint : Joint = js.joint;
            if (
                !joint.allowCollision &&
                (joint.body1 == b1 && joint.body2 == b2 ||
                joint.body1 == b2 && joint.body2 == b1)) {
                return false;
            }
            js = js.next;
        }
        return true;
    }
    
    /**
		 * Detect overlapping pairs.
		 */
    public function detectPairs() : Void{
        while (numPairs > 0){
            var pair : Pair = pairs[--numPairs];
            pair.shape1 = null;
            pair.shape2 = null;
        }
        numPairChecks = 0;
        collectPairs();
    }
    
    private function collectPairs() : Void{
        //throw new Error("Inheritance error.");
    }
    
    private function addPair(s1 : Shape, s2 : Shape) : Void{
        if (numPairs == bufferSize) {  // expand pair buffer  
            var newBufferSize : Int = bufferSize << 1;
            var newPairs : Array<Pair> = new Array<Pair>();
            for (i in 0...bufferSize){
                newPairs[i] = pairs[i];
            }
            for (i in bufferSize...newBufferSize){
                newPairs[i] = new Pair();
            }
            pairs = newPairs;
            bufferSize = newBufferSize;
        }
        var pair : Pair = pairs[numPairs++];
        pair.shape1 = s1;
        pair.shape2 = s2;
    }
}

