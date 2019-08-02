/* ============================================ */
/* scan enable */
var PScanEnable = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/laser/scan_enable',
});

PScanEnable.get(function(value) {
    if (value != null) {
        $('#Laser_1').prop('checked',value[0]);
        $('#Laser_1').change();
        $('#Laser_2').prop('checked',value[1]);
        $('#Laser_2').change();
        $('#Laser_3').prop('checked',value[2]);
        $('#Laser_3').change();
    }
});

function SetScanEnable() {
    let LaserCheck = document.getElementsByName("LaserCheckElement");
    let Box = [];
    for(let i=0;i<LaserCheck.length;i++){
        if(LaserCheck[i].checked){
            Box.push(parseInt(1));
        }else{
            Box.push(parseInt(0));
        }
        
    }
    PScanEnable.set(Box);
}
/* ============================================ */
/* scan parameter */
var PScanParam = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/vision/laser/scan_parameter',
});

PScanParam.get(function(value) {
    if (value != null) {
        let barParam  = document.getElementsByName("LaserBarElement");
        let textParam = document.getElementsByName("LaserTextElement");

        for(let i=0;i<value.length;i++){
            barParam[i].value = value[i];
            textParam[i].value = value[i];
        }
    }
});

function SetScanParam() {
    let LaserParam = document.getElementsByName("LaserBarElement");
    let Box = [];
    for(let i=0;i<LaserParam.length;i++){
        Box.push(parseInt(LaserParam[i].value));
    }
    PScanParam.set(Box);
}