Serial1.setup(9600, {tx:B6,rx:B7});
Serial1.on('data', function (data) { 
  cmd+=data;
  var idx = cmd.indexOf("\r");
  var idx2 = cmd.indexOf("+");
  if (idx2==0) {
    if (cmd.length >=3) {
      if (cmd.getCharAt(2)=="," && cmd.getCharCodeAt(1)==cmd.length+3) {
        processData(E.toUint8Array(cmd.substr(2)));
        cmd="";
        return;
      }
    } else {
      return; //return because we don't know what kind of data it is yet; 
    } 
  }
  while (idx>=0) { 
    var line = cmd.substr(0,idx);
    cmd = cmd.substr(idx+1);
    processLine(line);
    idx = cmd.indexOf("\r");
  }
  }
});
var cmd="";

function processLine(l) {
  console.log("Received line:")
  console.log(l)
  if (l.indexOf("+"==0){
    l=l.substr(1);
    len=l.length/2
    dat=new Uint8Array(len);
    for (var i=0;i<=len;i++) {
      dat[i]=parseInt(l.substr(i,2),16);
    }
    processData(dat)
  }
}

function processData(dat) {
  console.log("Received data:")
  console.log(dat)
}
function sendCmd(dat) {
  var len=dat.length;
  var CmdTX="";
  if (len==4) {
    CmdTX="AT+SEND\r";
  } else if (len==7){
    CmdTX="AT+SENDE\r";
  } else if (len==15){
    CmdTX="AT+SENDL\r";
  } else if (len==31){
    CmdTX="AT+SENDE\r";
  } else {
    throw "Invalid packet length"
  }
  Serial1.print(CmdTX)
  //Serial1.print(E.toString(dat));
  for (i=0;i < len;i++) {
    Serial1.print(dat[i].toString(16));
  }
  Serial1.print("\r")
}
