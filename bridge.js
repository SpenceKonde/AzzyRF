//for Espruino on ESP8266
//Talks over serial to an Arduino running SerialBridge (formerly ICPDemo)
//to send things over 433MHz RF

var bridgeversion="AzzyRF WiFi Bridge v1.1";
var Clock = require("clock").Clock;
var heartbeatst=1;

function startup() {
  setBusyIndicator(2);
  require("ESP8266").logDebug(0);
  require("ESP8266").setLog(0);
  setTimeout("getDate();INTERVALS.date=setInterval(getDate,1800000);",2000);
  setTimeout('LastRxTimes=[getDStr()+" "+getTStr(":")];',4000);
  setTimeout('require("http").createServer(onPageRequest).listen(80);',5000);
  Telnet.setConsole(true);
  Serial1.setup(115200);
  Serial1.on('data', onSerial);
}

dateurl="http://raspi/date.php";
var INTERVALS={"date":""};
var clk;

var ResetTimer=-1; //This stores the ID of the timeout that resets the receiving code.
var KickTimer=-1;

function getDate() {
  var date="";
  require("http").get(dateurl, function(res) {
    res.on('data',function (data) {date+=data;});
    res.on('close',function() {clk=new Clock(date);});
  });
}


var http = require("http");
var rhead={'Access-Control-Allow-Origin':'*', 'Connection':'close'};
var khead={'Access-Control-Allow-Origin':'*', 'Connection':'close', 'Retry-After':'10'};

// Network

function onPageRequest(req, res) {
  if (KickTimer!=-1 ) {
    res.writeHead(503,khead);
    res.write("Transciever reset in progress");
    res.end();
    return;
  }
  var a = url.parse(req.url, true);
  var resu = handleCmd(a.pathname,a.query,res);
  if (resu == -1) {
    ; //do nothing
  } else if (resu) {
    res.writeHead(resu,rhead);
    if (resu==200) {res.write("OK");}
    else if (resu==404) {res.write("Not Found");}
  } else {
    res.writeHead(500,rhead);
    res.write("ERROR");
  }
  res.end();
}

function handleCmd(pn,q,r) {
  try {
    if (pn=="/send4.cmd") {
      return RFSend(q.message,4)?200:400;
    } else if (pn=="/send8.cmd") {
      return RFSend(q.message,8)?200:400;
    } else if (pn=="/send16.cmd") {
      return RFSend(q.message,16)?200:400;
    } else if (pn=="/send32.cmd") {
      return RFSend(q.message,32)?200:400;
    } else if (pn=="/kick.cmd") {
      r.writeHead(200,rhead);
      r.write("Resetting transciever, wait 5 seconds");
      kickTiny();
      return -1;
    } else if (pn=="/version.qry") {
      r.writeHead(200,rhead);
      r.write(bridgeversion);
      return -1;
    } else if (pn=="/lastrx.qry"){
      r.writeHead(200,rhead);
      r.write('[');
      for (var i=0;i<LastRxValues.length;i++){
        r.write('\r\n["');
        r.write(LastRxTimes[i]);
        r.write('","');
        r.write(LastRxValues[i]);
        r.write('"]');
        if (LastRxValues.length < i+1) {
          r.write(',');
        }
      }
      r.write('\r\n]');
      return -1;
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

function kickTiny(){
  resetReceive();
  digitalWrite(12,0);
  setTimeout("digitalWrite(12,1);",50);
  KickTimer=setTimeout("digitalWrite(12,1);resetReceive();KickTimer=-1;",5000);
}

//Receive processing

rxdata="";
rxing=0;
rcvvers=1;
rxlen=4;
rxraw=new Uint8Array(32);
rxrawidx=0;
LastRxValues=["REBOOT"];
//LastRxTimes=[""]; //set in onInit();
incomingraw=new Uint8Array(32);

function onSerial(data) {
  for (var x=0;x<data.length;x++) {
    var c=data.charAt(x);
    if (rxing) {
      if (c=='\n' || c=='\r') {
        //end of message
        if (rxdata.length==2*rxlen) {
          processReceive();
          resetReceive();
        } else {
          resetReceive();
        }
      } else {
        rxdata+=c;
        if (rxdata.length%2==0){
          rxraw[rxrawidx++]=parseInt(rxdata.substr(rxdata.length-2,2),16);
          if(rxrawidx==1){
            var pl=rxraw[0]>>6;
            if (pl==0){
              rxlen=4;
            } else if (pl==1){
              rxlen=7;
            } else if (pl==2){
              rxlen=15;
            } else {
              rxlen=31;
            }
          }
        }
      }
    } else { //not currently receiving
      if (c=='='){
        rcvvers=2;
        rxing=1;
        if (ResetTimer!=-1) {
          clearTimeout(ResetTimer);
          ResetTimer=-1;
        }
        resetTimer=setTimeout(resetReceive,1000);
      } else if (c=='+') {
        rcvvers=1;
        rxing=1;
        if (ResetTimer!=-1) {
          clearTimeout(ResetTimer);
          ResetTimer=-1;
        }
        resetTimer=setTimeout(resetReceive,1000);
      }
    }
  }
}

function processReceive() {
  if(LastRxValues.unshift((rcvvers==2?'=':'+')+rxdata)>=10){
    LastRxValues.pop();
  }
  if(LastRxTimes.unshift(getDStr()+" "+getTStr(":"))>=10){
    LastRxValues.pop();
  }
  //console.log(rxdata);
  var cmd="";
  if (rxlen ==4 ) {
    if (RFCm[(rcvvers==2?'=':'+')+rxdata]!=undefined){
      cmd=RFCm[(rcvvers==2?'=':'+')+rxdata];
    } else if (RFCm[rxdata]!=undefined){
      cmd=RFCm[rxdata];
    } else {
      //console.log("unknown");
      return; //no action for that packet
    }
    //console.log("cmd: "+cmd);
    setTimeout("eval("+cmd+")",500);
  } else {
    var key=rxdata.substr(0,14);
    for (var i=1;i<8;i++){
      if (RFCl[(rcvvers==2?'=':'+')+key]!=undefined) {
        cmd=RFCl[(rcvvers==2?'=':'+')+key];
        //console.log("key: "+key);
        break;
      } else if (RFCl[key]!=undefined) {
        cmd=RFCl[key];
        //console.log("key: "+key);
        break;
      }
      key=key.substr(2,14-(i*2));
    }
  if (cmd=="") {
    //console.log("unknown long");
    return;
  } else {
    //console.log("longcmd: "+cmd);
    var incomingraw=new Uint8Array(rxraw);
    setTimeout("eval("+cmd+")",500);
  }
  }
}

function resetReceive() {
  if (ResetTimer!=-1) {
    clearTimeout(ResetTimer);
    ResetTimer=-1;
  }
  rxdata="";
  rxing=0;
  rcvvers=1;
  rxlen=4;
  rxrawidx=0;
  rxraw.fill(0);
}



RFCm = {
};

/* example // each of these is a string of js passed to eval()
RFCm = {
  "+1F1E0200":"console.log('received 1F1E0200, CRC version 1')",
  "=1F1E0201":"console.log('received 1F1E0201, CRC version 2')",
  "1F1E0200":"console.log('received 1F1E0202, either version')"
};
*/

RFCl = {
};

function getTStr(sep){
  var hrs=clk.getDate().getHours();
  var mins=clk.getDate().getMinutes();
  var secs=clk.getDate().getSeconds();
  return (hrs>9?hrs:" "+hrs)+sep+(mins>9?mins:"0"+mins)+sep+(secs>9?secs:"0"+secs);
}

function getDStr(){
  var mon=clk.getDate().getMonth()+1;
  var day=clk.getDate().getDate();
  var year=clk.getDate().getFullYear();
  return (mon+"/"+day+"/"+year);
}
//onInit

function onInit() {
  digitalWrite(12,0);
  setTimeout("digitalWrite(12,1);",50);
  digitalWrite(13,1);
  setTimeout(startup,15000);
  setInterval("digitalWrite(13,(heartbeatst?0:1));heartbeatst=heartbeatst^1;",2500);
}
