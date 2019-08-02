var paramClient = new ROSLIB.Service({
  ros : ros,
  name : 'interface/save_srv',
  serviceType : 'std_srvs/Empty'
});

var param_request = new ROSLIB.ServiceRequest({
});

function savecall(){
  paramClient.callService(param_request,
    function(param_request) {
      console.log('成功儲存參數');
      //SendMsgs('成功儲存參數',"blue");
      //callback(result.action_servers);
    },
    function(message){
      console.log(message);
      //SendMsgs('儲存參數失敗',"red");
    }
  );
}
