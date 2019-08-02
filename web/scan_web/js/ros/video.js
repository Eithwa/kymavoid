//===========================================================================
//new

function Camera_Switch(value) {
    var view = document.getElementById("Camera-View");

    if (view.checked) {
        viewflag = value;
    } else {
        viewflag = 'off';
    }
}

function Open_Camera() {

    var video = document.getElementById("player");
    var view = document.getElementById("View").checked;
    
    if(view){
        video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/black"
    }else{
        video.src = "img/offline.png";
    }
            
}

