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

import oimohx.physics.collision.broadphase.Proxy;
import oimohx.physics.collision.shape.Shape;
import oimohx.physics.dynamics.RigidBody;
import oimohx.physics.dynamics.World;

/**
 * A broad-phase algorithm with brute-force search.
 * This always checks for all possible pairs.
 */
class BruteForceBroadPhase extends BroadPhase {
	
    private var proxies:Array<Proxy>;
    private var numProxies:Int;
    private var maxProxies:Int;
	
    
    public function new() {
        super();
        maxProxies = 256;
        proxies = [];
		collectPairs = _collectPairs;
    }
    
    /**
	 * @inheritDoc
	 */
    override public function createProxy(shape:Shape):Proxy {
        return new BasicProxy(shape);
    }
    
    /**
	 * @inheritDoc
	 */
    override public function addProxy(proxy:Proxy) {
        if (numProxies == maxProxies) {
            maxProxies <<= 1;
            var newProxies:Array<Proxy> = [];
            for (i in 0...numProxies){
                newProxies[i] = proxies[i];
            }
            proxies = newProxies;
        }
        proxies[numProxies++] = proxy;
    }
    
    /**
	 * @inheritDoc
	 */
    override public function removeProxy(proxy:Proxy) {
        for (i in 0...numProxies){
            if (proxies[i] == proxy) {
                proxies[i] = proxies[--numProxies];
                proxies[numProxies] = null;
                return;
            }
        }
    }
    
    inline private function _collectPairs() {
        numPairChecks = numProxies * (numProxies - 1) >> 1;
		
		var p1:Proxy = null;
		var b1:AABB = null;
		var s1:Shape = null;
		var p2:Proxy = null;
		var b2:AABB = null;
		var s2:Shape = null;
		
        for (i in 0...numProxies) {
            p1 = proxies[i];
            b1 = p1.aabb;
            s1 = p1.shape;			
            for (j in i + 1...numProxies){
                p2 = proxies[j];
                b2 = p2.aabb;
                s2 = p2.shape;
                if (
					b1.maxX < b2.minX || b1.minX > b2.maxX ||
					b1.maxY < b2.minY || b1.minY > b2.maxY ||
					b1.maxZ < b2.minZ || b1.minZ > b2.maxZ ||
					!isAvailablePair(s1, s2)
				) {
					continue;
				}
                addPair(s1, s2);
            }
        }
    }
	
}
