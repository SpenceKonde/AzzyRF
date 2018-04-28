var txpin=B9;
var rxpin = B8;
var n="";
var wf;

txOneLength=1.100; //length of a 1
txZeroLength=0.600; //length of a 0
txLowTime=0.650; //length of the gap between bits
txTrainRep=30; //number of pulses in training burst
txSyncTime=2; //length of sync
txTrainOneLen=0.400;
txTrainZeroLen=0.400;


var z=32; //This is the length of packet to receive, in bits. 

/*
txOneLength=0.500;
txZeroLength=0.210;
txLowTime=0.350;
txTrainOneLen=0.200;
txTrainZeroLen=0.200;

*/

///*
txOneLength=0.55;
txZeroLength=0.3;
txLowTime=0.42;
txTrainOneLen=0.200;
txTrainZeroLen=0.200;

//*/

var ol=txOneLength;
var zl=txZeroLength;
var lt=txLowTime;
var ttr=txTrainTimeRep;
var st=txSyncTime;



function sendRaw(dat,rep) {
  console.log(dat);
	rep=rep?rep:5;
	var l=dat.length;
	var d = digitalPulse;
	var p=txpin;
	var h;
    var tr=[txTrainOneLen,txTrainZeroLen];
	for (var v=0;v<rep;v++) {
	for (i = 0; i < ttr; i++) {
			d(p,1,tr);
		}
		d(p,1,0.5); //end training burst
		d(p,0,st);
		for (var j=0;j<l;j++) {
		h=dat[j];
		for (k=7;k>=0;k--) {
		d(p,1,((h>>k)&1)?ol:zl); 
		d(p,0,lt);
		}
		}
		d(p,0,0);
	}
	digitalWrite(txpin,0);
}

function sendRaw2(dat,rep) {
	rep=rep?rep:5;
	var l=dat.length;
	var d = digitalPulse;
	var p=txpin;
	var h;
	var tb=[];
	digitalWrite(txpin,0);
	for (i = 0; i < txTrainRep; i++) {
		tb[i*2]=txTrainOneLen; 
		tb[i*2+1]=txTrainZeroLen;
	}
	tb[txTrainRep]=txTrainOneLen;
	tb[txTrainRep+1]=txSyncTime;
	var td=[];
	//console.log(tb);
	for (var j=0;j<l;j++) 
		{
			h=dat[j];
			for (k=7;k>=0;k--) 
			{
				td[(j*8+(7-k))*2]=(((h>>k)&1)?txOneLength:txZeroLength);
				td[(j*8+(7-k))*2+1]=txLowTime;
			}
		}
  
	for (var v=0;v<rep;v++) 
	{
      //console.log(td);
      //console.log(tb);
		digitalPulse(p,1,tb);
		//d(p,0,0);
		digitalPulse(p,1,td);
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
	if (n.length==z)	parseRx(n);
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
		console.log("Add "+ add+" cmd: "+cmd+" param: "+param+" extparam: "+extparam);
		extparam=extparam&15;
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



function sendTestPacket(address) {
	sendCmd(address,0xFE,randInt(256),randInt(16));
}

function startTestRun(addr,qty_,delay){
  trcount=0;
  qty=qty_;
  inter=setInterval("trcount++;console.log(trcount);sendTestPacket("+addr+");if(trcount>=qty){clearInterval(inter);console.log('done');}",delay);
}

function randInt(i) {
	return parseInt(Math.random()*i);
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
	rxcallback=function (dat) {console.log("light level: "+((dat[1]<<2)+((dat[3]&0xB0)>>6))+" out of 1023. Temp is " (((dat[2]<<2)+((dat[3]&0x30)>>4))-80)/4+" degrees C");};
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
		rxcallback=function (dat) {console.log("Temp is "+((dat[1]<<2)+(dat[2]*0.25))+" degrees C");};
	}
	sendCmd(add,0x20,sensor,0,32);
}