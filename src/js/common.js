mergeInto(LibraryManager.library, {
  registerTimeout: function (func, delay){

		function runPeriodicFunction(func, delay) {
			func();
			window.setTimeout(function() {
				runPeriodicFunction(func, delay);
			}, 10);
		}

		console.log("Starting periodic function: " + func + " delay: " + delay);
		runPeriodicFunction(FUNCTION_TABLE[func], delay);
	}
});