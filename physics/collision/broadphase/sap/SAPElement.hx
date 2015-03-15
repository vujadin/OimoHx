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
package oimohx.physics.collision.broadphase.sap;

import oimohx.physics.collision.broadphase.sap.SAPProxy;

/**
	 * An element of proxies.
	 * @author saharan
	 */
class SAPElement
{
    /**
		 * The parent proxy.
		 */
    public var proxy : SAPProxy;
    
    /**
		 * The pair element.
		 */
    public var pair : SAPElement;
    
    /**
		 * The value of the element.
		 */
    public var value : Float;
    
    /**
		 * Whether the element has maximum value or not.
		 */
    public var max : Bool;
    
    /**
		 * The minimum element on other axis.
		 */
    public var min1 : SAPElement;
    
    /**
		 * The maximum element on other axis.
		 */
    public var max1 : SAPElement;
    
    /**
		 * The minimum element on other axis.
		 */
    public var min2 : SAPElement;
    
    /**
		 * The maximum element on other axis.
		 */
    public var max2 : SAPElement;
    
    public function new(proxy : SAPProxy, max : Bool)
    {
        this.proxy = proxy;
        this.max = max;
        value = 0;
    }
}

