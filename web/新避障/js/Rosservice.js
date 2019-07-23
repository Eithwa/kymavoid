
var vison_better_Client = new ROSLIB.Service({
  ros : ros,
  name : 'StrategyParam',
  serviceType : 'param_convey/strategy_param'
});

var vision_better_request = new ROSLIB.ServiceRequest({
    receive: 1
});
function savecall(){
  vison_better_Client.callService(vision_better_request, 
    function(vision_better_request) {
      if (vision_better_request.update == 2) {
        console.log('Parameter is saved');
      }
    },
    function(message){
      console.log("SAVE FAILED");
      //SendMsgs('儲存參數失敗',"red");
    }
  );
}
