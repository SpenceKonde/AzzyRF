Serial2.setup(115200,{rx:16, tx:17});
Serial2.on('data',function(data){handleSerial(data);});
var RFAddresses=[31];

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
  rcv.length=l;
  rcv.destaddress=rcv.data[0]&0x3F;
  if (RFAddresses[0]==0 || RFAddresses.indexOf(rcv.destaddress)!=-1) { //is the packet for this device, or is this device universal listener?
    processPacket(rcv);
  }
}

function processPacket(rcvpkt) {
  if (rcvpkt.version==1) {
    processPacket_V1(rcvpkt);
  } else {
    processPacket_V2(rcvpkt);
  }
}

function processPacket_V1(rcvpkt) {
}

function processPacket_V2(rcvpkt) {
  
}
