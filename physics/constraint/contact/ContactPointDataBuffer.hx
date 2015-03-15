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
package oimohx.physics.constraint.contact;


class ContactPointDataBuffer {
	
    public var norX:Float;
    public var norY:Float;
    public var norZ:Float;
    public var tanX:Float;
    public var tanY:Float;
    public var tanZ:Float;
    public var binX:Float;
    public var binY:Float;
    public var binZ:Float;
    
    public var rp1X:Float;
    public var rp1Y:Float;
    public var rp1Z:Float;
    public var rp2X:Float;
    public var rp2Y:Float;
    public var rp2Z:Float;
    
    public var norU1X:Float;
    public var norU1Y:Float;
    public var norU1Z:Float;
    public var norU2X:Float;
    public var norU2Y:Float;
    public var norU2Z:Float;
    public var tanU1X:Float;
    public var tanU1Y:Float;
    public var tanU1Z:Float;
    public var tanU2X:Float;
    public var tanU2Y:Float;
    public var tanU2Z:Float;
    public var binU1X:Float;
    public var binU1Y:Float;
    public var binU1Z:Float;
    public var binU2X:Float;
    public var binU2Y:Float;
    public var binU2Z:Float;
    
    public var norT1X:Float;
    public var norT1Y:Float;
    public var norT1Z:Float;
    public var norT2X:Float;
    public var norT2Y:Float;
    public var norT2Z:Float;
    public var tanT1X:Float;
    public var tanT1Y:Float;
    public var tanT1Z:Float;
    public var tanT2X:Float;
    public var tanT2Y:Float;
    public var tanT2Z:Float;
    public var binT1X:Float;
    public var binT1Y:Float;
    public var binT1Z:Float;
    public var binT2X:Float;
    public var binT2Y:Float;
    public var binT2Z:Float;
    
    public var norTU1X:Float;
    public var norTU1Y:Float;
    public var norTU1Z:Float;
    public var norTU2X:Float;
    public var norTU2Y:Float;
    public var norTU2Z:Float;
    public var tanTU1X:Float;
    public var tanTU1Y:Float;
    public var tanTU1Z:Float;
    public var tanTU2X:Float;
    public var tanTU2Y:Float;
    public var tanTU2Z:Float;
    public var binTU1X:Float;
    public var binTU1Y:Float;
    public var binTU1Z:Float;
    public var binTU2X:Float;
    public var binTU2Y:Float;
    public var binTU2Z:Float;
    
    public var norImp:Float;
    public var tanImp:Float;
    public var binImp:Float;
    
    public var norDen:Float;
    public var tanDen:Float;
    public var binDen:Float;
    
    public var norTar:Float;
    
    public var next:ContactPointDataBuffer;
    public var last:Bool;
	
    
    public function new() {
        
    }
	
}

