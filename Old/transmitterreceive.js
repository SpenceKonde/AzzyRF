
var txpin=C7;
var rxpin = C6;


function sendCmd(add,cmd,param,ext) {
    ext=ext?ext&15:0;
    var csc=add^cmd^param;
    csc=(csc&15)^(csc>>4)^ext;
    console.log(csc); 
    var b=(add<<20)+(cmd<<12)+(param<<4)+ext;
    console.log(b);
  for (r=0;r<5;r++) {
    var d = digitalPulse;
    var p=txpin;
    for (i = 0; i < 25; i++) {
        d(p,1,0.4); 
        d(p,0,0.4); 
    } 
    d(p,1,0.5); //end training burst
    d(p,0,2);
    //send 1s or 0s, and pause
    for (j=27;j>-1;j--) {
        d(p,1,((b>>j)&1)?1.1:0.6); 
        d(p,0,0.65);
    }
    for (j=3;j>-1;j--) {
        d(p,1,((csc>>j)&1)?1.1:0.6); 
        d(p,0,0.65);
    }
    // wait for finish
    d(p,0,0);
  }
  digitalWrite(txpin,0);
}

function sendRaw(dat,rep) {
	rep=rep?rep:5;
	var l=dat.length;
    var d = digitalPulse;
    var p=txpin;
    var h;
	for (var i=0;i<rep;i++) {
		for (i = 0; i < 25; i++) {
	        d(p,1,0.4); 
	        d(p,0,0.4); 
	    }
	    d(p,1,0.5); //end training burst
	    d(p,0,2);
	    for (var j=0;j<l;j++) {
	    	h=dat[l]
	    	for (k=7;k>=0;k--) {
				d(p,1,((h>>k)&1)?1.1:0.6); 
				d(p,0,0.65);
	    	}
    		d(p,0,0);
	    }

	}
  	digitalWrite(txpin,0);
}



var n="";
var wf;

function sigOff(e) {
  var d=e.time-e.lastTime;
  if (d>0.0005 && d<0.0013) n+=(d>0.0008)?1:0;
  else{
    if (n.length==z)  parseRx(n);
    n="";
  }
}
function parseRx(rcv) {
  console.log(rcv);
  var b3=parseInt(rcv.substr(24,8),2);
  var b2=parseInt(rcv.substr(16,8),2);
  var b1=parseInt(rcv.substr(8,8),2);
  var b0=parseInt(rcv.substr(0,8),2);
  var b4=parseInt(rcv.substr(32,8),2);
  var b5=parseInt(rcv.substr(40,8),2);
  var b6=parseInt(rcv.substr(48,8),2);
  var b7=parseInt(rcv.substr(56,8),2);
  ncsc=b0^b1^b2;
  ncsc=(ncsc&0x0F)^((ncsc>>4)&0x0F)^ext;
  console.log("B0:"+b0+" B1:"+b1+" B2:"+b2+" b3:"+b3+" b4: "+b4+" b5: "+b5+" b6: "+b6+" b7: "+b7);
  //if (ncsc==csc) {
    stopListen();
    //processRx(b0,b1,b2,ext);
  //} else {
    //console.log("Bad checksum!");
    //n="";
  //}
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

function runTest() {
    sendCmd(31,0xF2,0x10,1);
    setTimeout("startListen()",1000);
    setTimeout("stopListen()",2000);
}

function runTest2(pl,st) {
    z=pl*16;
    sendCmd(31,0xF3,st,pl);
    setTimeout("startListen()",400);
    setTimeout("stopListen()",3000);
}