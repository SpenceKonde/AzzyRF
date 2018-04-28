
var txpin=C7;
var rxpin = C6;
var n="";
var wf;

var z=32; //This is the length of packet to receive, in bits. 

function sendRaw(dat,rep) {
  rep=rep?rep:5;
  var l=dat.length;
    var d = digitalPulse;
    var p=txpin;
    var h;
  for (var v=0;v<rep;v++) {
    for (i = 0; i < 25; i++) {
          d(p,1,0.4); 
          d(p,0,0.4); 
      }
      d(p,1,0.5); //end training burst
      d(p,0,2);
      for (var j=0;j<l;j++) {
        h=dat[j];
        for (k=7;k>=0;k--) {
        d(p,1,((h>>k)&1)?1.1:0.6); 
        d(p,0,0.65);
        }
      }
        d(p,0,0);
  }
    digitalWrite(txpin,0);
}

function sigOff(e) {
  var d=e.time-e.lastTime;
  if (d>0.0005 && d<0.0013) n+=d>0.0008?1:0;
  else{
    n="";
  }
  if (n.length==z)  parseRx(n);
}

function parseRx(rcv) {
  
  console.log(rcv);
  stopListen();
  var b3=parseInt(rcv.substr(24,8),2);
  var b2=parseInt(rcv.substr(16,8),2);
  var b1=parseInt(rcv.substr(8,8),2);
  var b0=parseInt(rcv.substr(0,8),2);
  var b4=parseInt(rcv.substr(32,8),2);
  var b5=parseInt(rcv.substr(40,8),2);
  var b6=parseInt(rcv.substr(48,8),2);
  var b7=parseInt(rcv.substr(56,8),2);
  var b8=parseInt(rcv.substr(64,8),2);
  var b9=parseInt(rcv.substr(72,8),2);
  var b10=parseInt(rcv.substr(80,8),2);
  var b11=parseInt(rcv.substr(88,8),2);
  var b12=parseInt(rcv.substr(96,8),2);
  var b13=parseInt(rcv.substr(104,8),2);
  var b14=parseInt(rcv.substr(112,8),2);
  var b15=parseInt(rcv.substr(120,8),2);
  var b16=parseInt(rcv.substr(128,8),2);
  var b17=parseInt(rcv.substr(136,8),2);
  var b18=parseInt(rcv.substr(144,8),2);
  var b19=parseInt(rcv.substr(152,8),2);
  var b20=parseInt(rcv.substr(160,8),2);
  var b21=parseInt(rcv.substr(168,8),2);
  var b22=parseInt(rcv.substr(176,8),2);
  var b23=parseInt(rcv.substr(184,8),2);
  var b24=parseInt(rcv.substr(192,8),2);
  var b25=parseInt(rcv.substr(200,8),2);
  var b26=parseInt(rcv.substr(208,8),2);
  var b27=parseInt(rcv.substr(216,8),2);
  var b28=parseInt(rcv.substr(224,8),2);
  var b29=parseInt(rcv.substr(232,8),2);
  var b30=parseInt(rcv.substr(240,8),2);
  var b31=parseInt(rcv.substr(248,8),2);
  console.log("B0:"+b0+" B1:"+b1+" B2:"+b2+" B3:"+b3+" B4: "+b4+" B5: "+b5+" B6: "+b6+" B7: "+b7);
  console.log("B8:"+b8+" B9:"+b9+" B10:"+b10+" B11:"+b11+" B12: "+b12+" B13: "+b13+" B14: "+b14+" B15: "+b15);
  console.log("B16:"+b16+" B17:"+b17+" B18:"+b18+" B19:"+b19+" B20: "+b20+" B21: "+b21+" B22: "+b22+" B23: "+b23);
  console.log("B24:"+b24+" B25:"+b25+" B26:"+b26+" B27:"+b27+" B28: "+b28+" B29: "+b29+" B30: "+b30+" B31: "+b31);
}

function startListen() {
  wf=setWatch(sigOff,C6,{repeat:true,debounce:0.35,edge:"falling"});
  console.log("Listening started");
}
function stopListen() {
  if (wf) {clearWatch(wf);}
  wf=0;
  n="";
}




function runTest(add,inc) {
    z=32;
    var csc=31^0xF2^add;
    csc=(csc&15)^(csc>>4)^inc;
    sendRaw([31,0xF2,add,((inc<<4)+csc)],5);
    setTimeout("startListen()",400);
    setTimeout("stopListen()",2500);
}

function runTestAV(add) {
    z=32;
    var csc=31^101^0;
    csc=(csc&15)^(csc>>4)^0;
    sendRaw([31,101,0,csc],5);
    setTimeout("startListen()",400);
    setTimeout("stopListen()",2500);
}


function runTest4(datalength,replyrepeat,sub) {
    z=datalength*8-sub;
    var csc=31^0xF4^datalength;
    csc=(csc&15)^(csc>>4)^replyrepeat;
    sendRaw([31,0xF4,datalength,((replyrepeat<<4)+csc)],5);
    setTimeout("startListen()",400);
    setTimeout("stopListen()",2500);
}


function sendLong4() {
    sendRaw([0x40+31,100,101,102],5);
}

function sendLong8() {
    sendRaw([0x40+31,100,101,102,103,104,105,94],5);
}

function sendLong16() {
    sendRaw([0x80+31,100,101,102,103,104,105,106,107,108,109,110,111,112,113,158],5);
}

function sendLong32() {
    sendRaw([0xC0+31,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,222],5);
}

function pixeltest() {
    sendRaw([0xC0+31,0x39,4,100,255,32,255,255,255,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,2,125,126,127,128,129,9,188],5);
}

function pixeltest2() {
    sendRaw([0x80+31,0x39,0,0,0,0,255,0,0,0,255,0,0,0,255,89],5);
}

function pixeltest3() {
    sendRaw([0x80+31,0x39,4,0,0,0,0,0,0,0,0,0,0,0,0,162],5);
}



function sendCmd(add,cmd,param,extparam,reply) {
  var csc=add^cmd^param;
  csc=(csc&15)^(csc>>4)^extparam;
  sendRaw([add,cmd,param,((extparam<<4)+csc)],5);
  if (reply) {
    z=reply;
    setTimeout("startListen()",400);
    setTimeout("stopListen()",2500);
  }
}


