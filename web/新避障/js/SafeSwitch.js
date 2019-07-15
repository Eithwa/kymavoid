$(function($) {
    $("#SafeSwitchDiv").click(function() {
        obj = document.getElementById("SafeSwitchDiv");
        if (this.value) {
            safe_weitch_checked = false;
            $("#SafeSwitch").remove();
            var SafeSwitch_i = $(document.createElement('i'))
                .attr("class", "fa fa-lock fa-3x SafeIcon")
                .attr("aria-hidden", "true")
                .attr("id", "SafeSwitch")
                .attr("value", "0");
            SafeSwitch_i.appendTo("#SafeSwitchDiv");
            SafeSwitchState(0);
        } else {
            safe_weitch_checked = true;
            $("#SafeSwitch").remove();
            var SafeSwitch_i = $(document.createElement('i'))
                .attr("class", "fa fa-unlock fa-3x SafeIcon")
                .attr("aria-hidden", "true")
                .attr("id", "SafeSwitch")
                .attr("value", "1");
            SafeSwitch_i.appendTo("#SafeSwitchDiv");
            SafeSwitchState(1);
        }
    });
});

function SafeSwitchState(state) {
    obj = document.getElementById("SafeSwitchDiv");
    obj.value = state;
    SafeSwitchStart = state;
    RemoteSwitch(state);
}
