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
package oimohx.physics.collision.broadphase.dbvt;

import oimohx.physics.collision.broadphase.dbvt.DBVTNode;
import oimohx.physics.collision.broadphase.AABB;

/**
 * A dynamic bounding volume tree for the broad-phase algorithm.
 * @author saharan
 */
class DBVT {
    /**
	 * The root of the tree.
	 */
    public var root:DBVTNode;
    
    private var freeNodes:Array<DBVTNode>;
    private var numFreeNodes:Int;
    private var aabb:AABB;
	
    
    public function new() {
        freeNodes = [];
        numFreeNodes = 0;
        aabb = new AABB();
    }
    
    /**
	 * Move a leaf.
	 * @param	leaf
	 */
    public function moveLeaf(leaf:DBVTNode) {
        deleteLeaf(leaf);
        insertLeaf(leaf);
    }
    
    /**
	 * Insert a leaf to the tree.
	 * @param	node
	 */
    public function insertLeaf(leaf:DBVTNode) {
        if (root == null) {
            root = leaf;
            return;
        }
        var lb:AABB = leaf.aabb;
        var sibling:DBVTNode = root;
        var oldArea:Float;
        var newArea:Float;
        while (sibling.proxy == null) {  // descend the node to search the best pair  
            var c1:DBVTNode = sibling.child1;
            var c2:DBVTNode = sibling.child2;
            var b:AABB = sibling.aabb;
            var c1b:AABB = c1.aabb;
            var c2b:AABB = c2.aabb;
            
            oldArea = b.surfaceArea();
            aabb.combine(lb, b);
            newArea = aabb.surfaceArea();
            var creatingCost:Float = newArea * 2;  // cost of creating a new pair with the node  
            var incrementalCost:Float = (newArea - oldArea) * 2;
            
            var discendingCost1:Float = incrementalCost;
            aabb.combine(lb, c1b);
            if (c1.proxy != null) {
                // leaf cost = area(combined aabb)
                discendingCost1 += aabb.surfaceArea();
            }
            else {
                // node cost = area(combined aabb) - area(old aabb)
                discendingCost1 += aabb.surfaceArea() - c1b.surfaceArea();
            }
            
            var discendingCost2:Float = incrementalCost;
            aabb.combine(lb, c2b);
            if (c2.proxy != null) {
                // leaf cost = area(combined aabb)
                discendingCost2 += aabb.surfaceArea();
            }
            else {
                // node cost = area(combined aabb) - area(old aabb)
                discendingCost2 += aabb.surfaceArea() - c2b.surfaceArea();
            }
            
            if (discendingCost1 < discendingCost2) {
                if (creatingCost < discendingCost1) {
                    break;
                }
                else {
                    sibling = c1;
                }
            }
            else {
                if (creatingCost < discendingCost2) {
                    break;
                }
                else {
                    sibling = c2;
                }
            }
        }
        var oldParent:DBVTNode = sibling.parent;
        var newParent:DBVTNode;
        if (numFreeNodes > 0) {
            newParent = freeNodes[--numFreeNodes];
        }
        else {
            newParent = new DBVTNode();
        }
        newParent.parent = oldParent;
        newParent.child1 = leaf;
        newParent.child2 = sibling;
        newParent.aabb.combine(leaf.aabb, sibling.aabb);
        newParent.height = sibling.height + 1;
        sibling.parent = newParent;
        leaf.parent = newParent;
        if (sibling == root) {
            // replace root
            root = newParent;
        }
        else {
            // replace child
            if (oldParent.child1 == sibling) {
                oldParent.child1 = newParent;
            }
            else {
                oldParent.child2 = newParent;
            }
        }  // update whole tree  
        
        do{
            newParent = balance(newParent);
            fix(newParent);
            newParent = newParent.parent;
        }        while ((newParent != null));
    }
    
    public function getBalance(node:DBVTNode):Int {
        if (node.proxy != null) {
			return 0;
		}
        return node.child1.height - node.child2.height;
    }
    
    public function print(node:DBVTNode, indent:Int, text:String):String {
        var hasChild:Bool = node.proxy == null;
		
        if (hasChild) {
            text = print(node.child1, indent + 1, text);
		}
		
        var i:Int = indent * 2;
        while (i >= 0){
            text += " ";
            i--;
        }
		
        text += ((hasChild) ? getBalance(node) + "" : "[" + node.proxy.aabb.minX + "]") + "\n";
        if (hasChild) {
            text = print(node.child2, indent + 1, text);
		}
		
        return text;
    }
    
    /**
	 * Delete a leaf from the tree.
	 * @param	node
	 */
    public function deleteLeaf(leaf:DBVTNode) {
        if (leaf == root) {
            root = null;
            return;
        }
        var parent:DBVTNode = leaf.parent;
        var sibling:DBVTNode;
        if (parent.child1 == leaf) {
            sibling = parent.child2;
        }
        else {
            sibling = parent.child1;
        }
        if (parent == root) {
            root = sibling;
            sibling.parent = null;
            return;
        }
        var grandParent:DBVTNode = parent.parent;
        sibling.parent = grandParent;
        if (grandParent.child1 == parent) {
            grandParent.child1 = sibling;
        }
        else {
            grandParent.child2 = sibling;
        }
        if (numFreeNodes < 16384) {
            freeNodes[numFreeNodes++] = parent;
        }
        do {
            grandParent = balance(grandParent);
            fix(grandParent);
            grandParent = grandParent.parent;
        } while (grandParent != null);
    }
    
    private function balance(node:DBVTNode):DBVTNode {
        var nh:Int = node.height;
        if (nh < 2) {
            return node;
        }
		
        var p:DBVTNode = node.parent;
        var l:DBVTNode = node.child1;
        var r:DBVTNode = node.child2;
        var lh:Int = l.height;
        var rh:Int = r.height;
        var balance:Int = lh - rh;
        var t:Int;  // for bit operation  
        
        //          [ N ]
        //         /     \
        //    [ L ]       [ R ]
        //     / \         / \
        // [L-L] [L-R] [R-L] [R-R]
        
        // Is the tree balanced?
        if (balance > 1) {
            var ll:DBVTNode = l.child1;
            var lr:DBVTNode = l.child2;
            var llh:Int = ll.height;
            var lrh:Int = lr.height;
            
            // Is L-L higher than L-R?
            if (llh > lrh) {
                // set N to L-R
                l.child2 = node;
                node.parent = l;
                
                //          [ L ]
                //         /     \
                //    [L-L]       [ N ]
                //     / \         / \
                // [...] [...] [ L ] [ R ]
                
                // set L-R
                node.child1 = lr;
                lr.parent = node;
                
                //          [ L ]
                //         /     \
                //    [L-L]       [ N ]
                //     / \         / \
                // [...] [...] [L-R] [ R ]
                
                // fix bounds and heights
                node.aabb.combine(lr.aabb, r.aabb);
                t = lrh - rh;
                node.height = lrh - (t & t >> 31) + 1;
                
                l.aabb.combine(ll.aabb, node.aabb);
                t = llh - nh;
                l.height = llh - (t & t >> 31) + 1;
            }
            else {
                // set N to L-L
                l.child1 = node;
                node.parent = l;
                
                //          [ L ]
                //         /     \
                //    [ N ]       [L-R]
                //     / \         / \
                // [ L ] [ R ] [...] [...]
                
                // set L-L
                node.child1 = ll;
                ll.parent = node;
                
                //          [ L ]
                //         /     \
                //    [ N ]       [L-R]
                //     / \         / \
                // [L-L] [ R ] [...] [...]
                
                // fix bounds and heights
                node.aabb.combine(ll.aabb, r.aabb);
                t = llh - rh;
                node.height = llh - (t & t >> 31) + 1;
                
                l.aabb.combine(node.aabb, lr.aabb);
                t = nh - lrh;
                l.height = nh - (t & t >> 31) + 1;
            }  // set new parent of L  
            
            if (p != null) {
                if (p.child1 == node) {
                    p.child1 = l;
                }
                else {
                    p.child2 = l;
                }
            }
            else {
                root = l;
            }
            l.parent = p;
            return l;
        }
        else if (balance < -1) {
            var rl:DBVTNode = r.child1;
            var rr:DBVTNode = r.child2;
            var rlh:Int = rl.height;
            var rrh:Int = rr.height;
            
            // Is R-L higher than R-R?
            if (rlh > rrh) {
                // set N to R-R
                r.child2 = node;
                node.parent = r;
                
                //          [ R ]
                //         /     \
                //    [R-L]       [ N ]
                //     / \         / \
                // [...] [...] [ L ] [ R ]
                
                // set R-R
                node.child2 = rr;
                rr.parent = node;
                
                //          [ R ]
                //         /     \
                //    [R-L]       [ N ]
                //     / \         / \
                // [...] [...] [ L ] [R-R]
                
                // fix bounds and heights
                node.aabb.combine(l.aabb, rr.aabb);
                t = lh - rrh;
                node.height = lh - (t & t >> 31) + 1;
                r.aabb.combine(rl.aabb, node.aabb);
                t = rlh - nh;
                r.height = rlh - (t & t >> 31) + 1;
            }
            else {
                // set N to R-L
                r.child1 = node;
                node.parent = r;
                
                //          [ R ]
                //         /     \
                //    [ N ]       [R-R]
                //     / \         / \
                // [ L ] [ R ] [...] [...]
                
                // set R-L
                node.child2 = rl;
                rl.parent = node;
                
                //          [ R ]
                //         /     \
                //    [ N ]       [R-R]
                //     / \         / \
                // [ L ] [R-L] [...] [...]
                
                // fix bounds and heights
                node.aabb.combine(l.aabb, rl.aabb);
                t = lh - rlh;
                node.height = lh - (t & t >> 31) + 1;
                r.aabb.combine(node.aabb, rr.aabb);
                t = nh - rrh;
                r.height = nh - (t & t >> 31) + 1;
            }  // set new parent of R  
            
            if (p != null) {
                if (p.child1 == node) {
                    p.child1 = r;
                }
                else {
                    p.child2 = r;
                }
            }
            else {
                root = r;
            }
            r.parent = p;
            return r;
        }
		
        return node;
    }
    
    inline private function fix(node:DBVTNode) {
        var c1:DBVTNode = node.child1;
        var c2:DBVTNode = node.child2;
        node.aabb.combine(c1.aabb, c2.aabb);
        var h1:Int = c1.height;
        var h2:Int = c2.height;
        if (h1 < h2) {
            node.height = h2 + 1;
        }
        else {
            node.height = h1 + 1;
        }
    }
	
}
