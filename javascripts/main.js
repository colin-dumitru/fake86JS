function onLoad() {
    $("#loadButton").click(function () {
        $("#mainContent")
            .text("Loading...")
            .removeClass("wrapper")
            .addClass("mainContentExpanded")
            .load("fake86.html");
    });
}