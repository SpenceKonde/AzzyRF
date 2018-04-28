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

function parseRx(rxdat) {
  stopListen();
  console.log(rxdat);
  rcvdata=new Uint8Array(z/8);
  for (i=0;i<(z/8);i++) {
  	rcvdata[i]=parseInt(rxdat.substr(i*8,8),2);
    
  }
  if (z==32) {
  	if (calcscsc(rcvdata)==(rcvdata[3]&0x0F)) {
        console.log("good csc");
        if (rxcallback) {
        	rxcallback(rcvdata);
        	delete rxcallback;
        }
  	} else {
  		console.log("bad csc");
        console.log("Expected "+calcscsc(rcvdata)+" got "+(rcvdata[3]&0x0F)+"");
  		startListen();
  		setTimeout("stopListen()",2500);
  	}
  } else {
  	if (calccsc(rcvdata)==rcvdata[(z/8)-1]) {
      if (rxcallback) {
        	rxcallback(rcvdata);
        	delete rxcallback;
        }
  	}
  }
}


    
function calcscsc(dat) {
	var retval=dat[0]^dat[1]^dat[2];
	retval=(retval&15)^(retval>>4)^((dat[3]&0xF0)>>4);
	return retval;
} 

function calccsc(dat) {
	var retval=0;
	for (var i=0;i<(dat.length-1);i++) {
		retval=retval^dat[i];
	}
	return retval;
}

function sendCmd(add,cmd,param,extparam,reply) {
	if (wf&&reply) {
		throw "Already waiting for transmission";
	} else {
		var csc=add^cmd^param;
		csc=(csc&15)^(csc>>4)^extparam;
		sendRaw([add,cmd,param,((extparam<<4)+csc)],10);
		if (reply) {
			z=reply;
			setTimeout("startListen()",300);
			setTimeout("stopListen()",2000);
		}
	}
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

function setPixel(address,px,r,g,b,latch) {
	var data=new Uint8Array(8);
	data[0]=address+64;
	data[1]=0x3A;
	data[2]=0x01+(latch?0x10:0);
	data[3]=px;
	data[4]=r;
	data[5]=g;
	data[6]=b;
	data[7]=calccsc(data);
	sendRaw(data);
}

function setAllPWM(address,r,g,b,w,latch) {
	var data=new Uint8Array(8);
	data[0]=address+64;
	data[1]=0x35;
	data[2]=(latch?1:0);
	data[3]=r;
	data[4]=g;
	data[5]=b;
	data[6]=w;
	data[7]=calccsc(data);
	sendRaw(data);
}


function setPixels(address,pxdat,latch) {
	var l=32;
	if (pxdat.length%4!=0 || pxdat.length > 28) {
		throw "Invalid data length";
	} else {
		if (pxdat.length==4) {
			return setPixel(address,pxdat[0],pxdat[1],pxdat[2],pxdat[3],latch);
		} else if (pxdat.length < 16) {
			l=16;
		}
		var data=new Uint8Array(l);
		data[0]=address+(l==16?128:196);
		data[1]=0x3A;
		data[2]=(pxdat.length/4)+(latch?0x10:0);
      console.log(pxdat.length/4+(latch?0x10:0));
		for (i=0;i<pxdat.length;i++) {
			data[3+i]=pxdat[i];
		}
		data[l-1]=calccsc(data);
		sendRaw(data);
	}
}

function latchPixels(address,delay,latch,clear) {
	var d=0;
	var ext=0;
	if (delay){
		if (delay < 256) {
			ext=0x04;
			d=delay;
		} else if (delay <256*60) {
			ext=0x08;
			d=delay/60;
		} else {
			ext=0x0C;
			d=delay/600;
		}
	} else {
		ext=(clear<<1)+latch;
	}
	sendCmd(address,0x3B,delay,ext);
}
function pCol(red,green,blue,white) {return (red==1)+((green==1)<<1)+((blue==1)<<2)+((white==1)<<3);}

function setPWM(address,colors,brightness,latch) {
	sendCmd(address,0x30+latch,brightness,colors);
}
function latchPWM(address,delay) {
	var ext=0;
	delay*=1000;
	while (delay > 255) {
		delay=delay>>1;
		ext++;
	}
	if (ext>15) {
		throw "Invalid delay";
	} else {
		sendCmd(address,0x32,delay,ext);
	}
}

function suspendPWM(address,colors,brightness) {
	sendCmd(address,0x32,brightness,colors);
}

function runTestAV(add) {
    z=32;
    var csc=add^101^0;
    csc=(csc&15)^(csc>>4)^0;
    sendRaw([add,101,0,csc],5);
    rxcallback=function (dat) {console.log("light level: "+((dat[1]<<2)+((dat[3]&0xB0)>>6))+" out of 1023. Temp is "+(((dat[2]<<2)+((dat[3]&0x30)>>4))-80)/4+" degrees C");}
    setTimeout("startListen()",400);
    setTimeout("stopListen()",2500);
}

function getSensor(add,sensor) {

    z=32;
    var csc=add^0x20^0;
    csc=(csc&15)^(csc>>4)^0;
    if (sensor==1) {
    	rxcallback=function (dat) {console.log("light level: "+((dat[1]<<2)+dat[2])+" out of 1023.");};
    } else {
    	rxcallback=function (dat) {console.log("Temp is "+((dat[1]<<2)+(dat[2]*0.25))" degrees C");};
    }
    sendCmd(add,0x20,sensor,0,32);
}