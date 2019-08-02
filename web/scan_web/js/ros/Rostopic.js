/* ============================================ */
/* scan enable */
var TScanEnable = new ROSLIB.Topic({
    ros: ros,
    name: '/scan/scan_enable',
    messageType: '/std_msgs/Int32MultiArray'
});

function PubScanEnable() {
    let LaserCheck = document.getElementsByName("LaserCheckElement");
    let Box = [];
    for(let i=0;i<LaserCheck.length;i++){
        if(LaserCheck[i].checked){
            Box.push(parseInt(1));
        }else{
            Box.push(parseInt(0));
        }
        
    }
    console.log(`scan enable = ${Box}`);
    let msg = new ROSLIB.Message({
        data: Box
    });
    TScanEnable.publish(msg);
}

/* ============================================ */
/* scan parameter */
var TScanParameter = new ROSLIB.Topic({
    ros: ros,
    name: '/scan/scan_parameter',
    messageType: '/std_msgs/Int32MultiArray'
});

function PubScanParam() {
    let LaserParam = document.getElementsByName("LaserBarElement");
    let Box = [];
    for(let i=0;i<LaserParam.length;i++){
       Box.push(parseInt(LaserParam[i].value));
    }
    console.log(`scan param = ${Box}`);
    let msg = new ROSLIB.Message({
        data: Box
    });
    TScanParameter.publish(msg);
}
/* ============================================ */
/* save parameter*/
var TSaveButton = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/bin_save',
    messageType: '/vision/bin'
});
function PubSaveButton(value) {
    SetScanEnable();
    SetScanParam();
    var SaveButton = new ROSLIB.Message({
        bin: 64
    });
    TSaveButton.publish(SaveButton);
    console.log("scan param save",64);
}
