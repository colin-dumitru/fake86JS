mergeInto(LibraryManager.library, {
  registerTimeout: function (func, delay){

		function runPeriodicFunction(func, delay) {
			func();
			window.setTimeout(function() {
				runPeriodicFunction(func, delay);
			}, delay);
		}

		console.log("Starting periodic function: " + func + " delay: " + delay);
		runPeriodicFunction(Module["_" + func], delay);
	}
});