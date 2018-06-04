Serial2.setup(115200,{rx:16, tx:17});
Serial2.on('data',function(data){handleSerial(data);});

function handleSerial(data) {
  console.log(data);
  var rcv={};
  if (data.charAt(0)=="=") {
    rcv.version=2;
  } else {
    rcv.version=1;
  }
  v=parseInt(data.substr(1,2),16)>>6;
  l=(v?(v==3?31:v==2?15:7):4);
  rcv.data=new Uint8Array(l);
  for (i=0;i<l;i++){
    rcv.data[i]=parseInt(data.substr(2*i+1,2),16);
  }
  console.log(JSON.stringify(rcv));
}