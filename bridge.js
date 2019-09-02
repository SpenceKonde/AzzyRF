//for Espruino on ESP8266
//Talks over serial to an Arduino running ICPDemo
//to send things over 433MHz RF


function startup() {
  setBusyIndicator(2);
  require("ESP8266").logDebug(0);
  require("ESP8266").setLog(0);
  require("http").createServer(onPageRequest).listen(80);
  Telnet.setConsole(true);
  Serial1.setup(115200);
}



var http = require("http");



var CORS={'Access-Control-Allow-Origin':'*'};
// Network

function onPageRequest(req, res) {
  var a = url.parse(req.url, true);
  var resu = handleCmd(a.pathname,a.query,res);
  if (resu == -1) {;}
  	else if (resu) {
  	res.writeHead(resu,CORS);
  	if (resu==200) {res.write("OK");}
  	else if (resu==404) {res.write("Not Found");}
  } else {
  	res.writeHead(500,CORS);
  	res.write("ERROR");
  }
  res.end();
}

function handleCmd(pn,q,r) {
	try {
		if (q.hasOwnProperty("index")){
			if( q.index>memmap.statMax || q.index < 0) {
				r.write("MAX INDEX: "+memmap.statMax);
				return 400;
			}
		}
		if (pn=="/send4.cmd") {
		    return RFSend(q.message,4)?200:500;
	  	} else if (pn=="/send8.cmd") {
		    return RFSend(q.message,8)?200:500;
  		}else if (pn=="/send16.cmd") {
		    return RFSend(q.message,16)?200:500;
		} else if (pn=="/send32.cmd") {
		    return RFSend(q.message,32)?200:500;
  		} else {
  			return 404;
		}
	} catch (err) {
		console.log(err);
		//r.write(err);
		return 0;
	}
}

function RFSend(message,len) {
  if (message==undefined) {
    return 0;
  }
  msglen=len==4?8:(2*(len-1));
	if (message.length != msglen) {
		return 0;
	}
    if (len==4){
	  Serial1.println("AT+SEND");
    } else if (len==8){
      Serial1.println("AT+SENDM");
    } else if (len==16){
      Serial1.println("AT+SENDL");
    } else if (len==32){
      Serial1.println("AT+SENDE");
    } else {
      return 0;
    }
	Serial1.println(message);
	return 1;
}

function onInit() {
	setTimeout(startup,15000);
}
