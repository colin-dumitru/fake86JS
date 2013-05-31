mergeInto(LibraryManager.library, {
  registerTimeout: function (funcPointer, delay){

		function runPeriodicFunction(func, delay) {
			func();
			window.setTimeout(function() {
				runPeriodicFunction(func, delay);
			}, delay);
		}

		var functionName = Pointer_stringify(funcPointer);
		console.log("Starting periodic function: " + functionName + " delay: " + delay);
		runPeriodicFunction(Module["_" + functionName], delay);
	}
});