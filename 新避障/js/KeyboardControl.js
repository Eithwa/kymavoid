window.addEventListener("keydown", keysdown, false);
window.addEventListener("keyup", keyuped, false);

$(function($) {
    $('#KeyboardControl').famultibutton();
    $("#KeyboardControl").click(function() {
        if (this.value)
            KeyboardState(0);
        else
            KeyboardState(1);
    });
});

function KeyboardState(state) {
    obj = document.getElementById("KeyboardControl");
    obj.value = state;
    KeyboardStart = state;
}

function keysdown(e) {
    if (KeyboardStart == true) {
        var vec3;
        keys[e.keyCode] = true;

        //RobotControl
        if (keys[87] && keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed / Math.pow(2, 0.5)),
                y: parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[87] && keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed / Math.pow(2, 0.5)),
                y: parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[83] && keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed / Math.pow(2, 0.5)),
                y: -parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[83] && keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed / Math.pow(2, 0.5)),
                y: -parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[87]) {
            vec3 = new ROSLIB.Message({
                x: 0,
                y: parseFloat(speed),
                z: 0
            });
            PublishTopicCmdVel(vec3);
           // PublishTopicCmdVel(vec3);
        } else if (keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed),
                y: 0,
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[83]) {
            vec3 = new ROSLIB.Message({
                x: 0,
                y: -parseFloat(speed),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed),
                y: 0,
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[69]) {
            var speed_;
            if (Math.abs(parseFloat(speed)) > 15){
              speed_ = parseFloat(speed) * 0.5;
            }else{
              speed_ = speed;
            }
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: -parseFloat(speed_)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[81]) {
            var speed_;
            if (Math.abs(parseFloat(speed)) > 15){
              speed_ = parseFloat(speed) * 0.5;
            }else{
              speed_ = speed;
            }
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: parseFloat(speed_)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        }

        //SwitchRobot
        if (keys[80]) {//key=p stop
            ParamAvoidGo(0);
            StrategyStop();
            $('#StartInput').prop('checked',false);
            $('#StartInput').change();
            $('#StopInput').prop('checked',true);
            $('#StopInput').change();
        } else if (keys[79]) {//key=o go
            if(safe_weitch_checked==true){
                ParamAvoidGo(1);
                $('#StartInput').prop('checked',true);
                $('#StartInput').change();
                $('#StopInput').prop('checked',false);
                $('#StopInput').change();
            }else{
                $('#StartInput').prop('checked',false);
                $('#StartInput').change();
                $('#StopInput').prop('checked',true);
                $('#StopInput').change();
            }
        }
    }
}


/* old */
// function keyuped(e) {
//     if (KeyboardStart) {
//         if (keys[81] == true) releasebutton();
//         else if (keys[69] == true) releasebutton();
//         else if (keys[87] == true) releasebutton();
//         else if (keys[65] == true) releasebutton();
//         else if (keys[83] == true) releasebutton();
//         else if (keys[68] == true) releasebutton();
//         keys[e.keyCode] = false;
//     }
// }

function keyuped(e) {
    //if(safe_weitch_checked==false){
    //    alert("尚未解鎖");
    //}
    if (KeyboardStart&&safe_weitch_checked) {
        console.log("start moving speed: %d", speed);
        if (keys[e.keyCode] == true) releasebutton(e.keyCode);
        keys[e.keyCode] = false;
    }
}

function releasebutton(state) {
    let vec3 = new ROSLIB.Message({
        x: 0,
        y: 0,
        z: 0
    });
    switch(state){
      case 81:
        vec3.z = 0;
        break;
      case 69:
        vec3.z = 0;
        break;
      case 87:
        vec3.y = 0;
        break;
      case 65:
        vec3.x = 0;
        break;
      case 83:
        vec3.y = 0;
        break;
      case 68:
        vec3.x = 0;
        break;
      default:
        vec3.x = 0;
        vec3.y = 0;
        vec3.Z = 0;
    }
    //if(state==81||state==69||state==87||state==65||state==83||state==68){
    //    console.log("stop");
    //    PublishTopicCmdVel(vec3);
        //PublishTopicCmdVel(vec3);
    //}
    
    console.log("stop");
    PublishTopicCmdVel(vec3);
}
