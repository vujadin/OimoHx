package oimohx.ds;


/**
	Created by Alexander "fzzr" Kozlovskij
 **/
typedef Float32Array =
#if (babylonhx && (js || purejs || html5 || web))
com.babylonhx.utils.typedarray.Float32Array;
#else
haxe.ds.Vector<Float>;
#end
