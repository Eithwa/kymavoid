function init_img(){
    var video = document.getElementById("canvasMap");
    //video.src = "img/AvGround.jpg";
    video.src = "img/AvGround.jpg";
    video.style.width = "100%";
    //setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/black";},100);
}
function vision_monitor(value){
    var video = document.getElementById("canvasMap");
    if(value==0){
        video.src = "img/AvGround.jpg";
        video.style.width = "100%";
    }
    if(value==1){
        video.src = "img/black.png";
        video.style.width = "74.3%";
        setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/black";},100);
    }
    if(value==2){
        video.src = "img/black.png";
        video.style.width = "74.3%";
        setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/red";},100);
    }
    if(value==3){
        video.src = "img/black.png";
        video.style.width = "60%";
        setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/avoid";},100);
    }
    
}
function MonitorSwitch(checked) {
    document.getElementById('canvasMap').style.zIndex = "0";
    var video = document.getElementById("canvasMap");
    let ground_reverse = document.getElementById("GroundButton").checked;
   
    if (checked == true) {
        video.src = "img/black.png";
        if(ground_reverse==true){
            document.getElementById('canvasMap').style.webkitTransform = "rotate(180deg)";
        }
        setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/mcl/image";},100);
        //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/mcl/image";
        console.log("localization map");
    } else {
        document.getElementById('canvasMap').style.webkitTransform = "rotate(0deg)";
        video.src = "img/LcGround.png";
        if (ground_reverse == false) {
            video.src = "img/LcGround.png";
        }else{
            video.src = "img/LcGround2.png";
        }
        console.log("ground map");
    }
}

