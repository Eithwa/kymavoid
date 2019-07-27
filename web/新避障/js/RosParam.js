//SPlanning_Velocity
var ParamSPlanningVelocityBox = new ROSLIB.Param({
    ros: ros,
    name: '/AvoidChallenge/SPlanning_Velocity'
});

function RosParamSP() {
    var box = []
    obj = document.getElementsByName('ParameterElement');
    box.push(100); //Vdis_max
    box.push(0.3); //Vdis_min

    box.push(parseFloat(obj[0].value)); //VTdis_max
    box.push(parseFloat(obj[1].value)); //VTdis_min
    box.push(1); //Tangle_max
    box.push(0.5); //Tangle_min
    box.push(144); //Angle_max
    box.push(5); //Angle_min

    console.log("SPlanning_Velocity", box);
    ParamSPlanningVelocityBox.set(box);
}

//getParam
ParamSPlanningVelocityBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
        for (var i = 0; i < 2; i++) {
            obj[i].value = value[i + 2];
        }
    }
});
//=======================================================================
//Distant
var ParamDistantBox = new ROSLIB.Param({
    ros: ros,
    name: '/AvoidChallenge/Distant'
});

function RosParamDistant() {
    var box = []
    obj = document.getElementsByName('ParameterElement');

    box.push(parseFloat(obj[2].value)); //close_dis
    box.push(parseFloat(obj[3].value)); //halfclose_dis
    box.push(parseFloat(obj[4].value)); //far_dis

    console.log("Distant", box);
    ParamDistantBox.set(box);
}
//getParam
ParamDistantBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
        for (var i = 2; i < 5; i++) {
            obj[i].value = value[i - 2];
        }
    }
});
//=======================================================================
//GraySet
var ParamGraySetBox = new ROSLIB.Param({
    ros: ros,
    name: '/AvoidChallenge/GraySet'
});

function ParamGraySet() {
    //GraySet
    console.log("GraySet", parseFloat(obj[5].value));
    ParamGraySetBox.set(parseFloat(obj[5].value));
}
//getParam
ParamGraySetBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
        obj[5].value = value;
    }
});
//=======================================================================
//obj_filter_size
var Paramfilter = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/blackItem/obj_filter_size'
});

function ParamFilterSet() {
    //GraySet
    console.log("obj_filter_size", parseFloat(obj[6].value));
    Paramfilter.set(parseFloat(obj[6].value));
}
//getParam
Paramfilter.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
        obj[6].value = value;
    }
});
//=======================================================================
//Save

function RosSaveParam() {
    RosParamSP();
    RosParamDistant();
    ParamGraySet();
    ParamFilterSet();
    setTimeout(topicROSSaveButton(),1000);
    setTimeout(savecall(),1000);
}

//=======================================================================
//Start

var AvoidGo = new ROSLIB.Param({
    ros: ros,
    name: '/AvoidChallenge/avoid_go'
});

function onloadAvoidGO() {
    setTimeout(ParamAvoidGo, 0, 0);
    setTimeout(ParamAvoidGo, 100, 0);
    setTimeout(ParamAvoidGo, 200, 0);
    setTimeout(ParamAvoidGo, 300, 0);
    setTimeout(ParamAvoidGo, 400, 0);
    setTimeout(ParamAvoidGo, 500, 0);
    $('#StartInput').prop('checked',false);
    $('#StartInput').change();
    $('#StopInput').prop('checked',true);
    $('#StopInput').change();
}

function ParamAvoidGo(state) {
    console.log(state);
    if(safe_weitch_checked==false&&state==1){
        alert("尚未解鎖");
        $('#StartInput').prop('checked',false);
        $('#StartInput').change();
        $('#StopInput').prop('checked',true);
        $('#StopInput').change();
    }
    if (CheckIP[0] == 1){
        if(state == 0){
            AvoidGo.set(state);
        }
        if(state == 1 && safe_weitch_checked){
            AvoidGo.set(state);
        }
    }
}
function strategy_connection(){
    onloadAvoidGO();
    SimulatorFalse();
    StrategyStop();
}
