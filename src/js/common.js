mergeInto(LibraryManager.library, {
  registerTimeout: function (funcPointer, delay){

		function runPeriodicFunction(func, delay) {
			window.setInterval(function() {
				func();
			}, delay);
		}

		var functionName = Pointer_stringify(funcPointer);
		console.log("Starting periodic function: " + functionName + " delay: " + delay);
		runPeriodicFunction(Module["_" + functionName], delay);
	}
});