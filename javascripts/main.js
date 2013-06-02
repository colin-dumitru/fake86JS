function onLoad() {
    $("#loadButton").click(function () {
		$("#loadingOverlay").addClass("shown");

        $("#mainContent")
            .text("Loading...")
            .removeClass("wrapper")
            .addClass("mainContentExpanded")
            .load("fake86.html", function() {
            	$("#loadingOverlay").removeClass("shown");
            });
    });
}