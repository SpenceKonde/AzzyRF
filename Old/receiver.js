var n="";
var t;
var b;
var d;
var wr;
var wf;
var rxing=0;
var lt;

function sigOff(e) {
  var d=e.time-e.lastTime;
  if (d>0.0004 && d<0.0013) n+=(d>0.0008)?1:0;
  else{
    if (n.length==32)  parseRx(n);
    n="";
  }
}

function parseRx(rcv) {
  console.log(rcv);
  var ext=parseInt(rcv.substr(24,4),2);
  var b2=parseInt(rcv.substr(16,8),2);
  var b1=parseInt(rcv.substr(8,8),2);
  var b0=parseInt(rcv.substr(0,8),2);
  var csc=parseInt(rcv.substr(28,4),2);
  ncsc=b0^b1^b2;
  ncsc=(ncsc&0x0F)^((ncsc>>4)&0x0F)^ext;
  console.log("B0:"+b0+" B1:"+b1+" B2:"+b2+" ext:"+ext+" csc:"+csc+" calccsc:"+ncsc);
  if (ncsc==csc) {
    clearWatch(wf);
    //processRx(b0,b1,b2,ext);
  } else {
    console.log("Bad checksum!");
  }
}

function startListen() {
  wf=setWatch(sigOff,C6,{repeat:true,edge:"falling"});
  console.log("Listening started");
}

function stopListen() {
  clearWatch(wf);
  n="";
}

