
exports.connect = function(serial) {
    return new AzzyRF(serial);
};

function AzzyRF(serial) {
  this.Serial=serial;
  this.timeout=0;
  this.inString="";
  this.datastring="";
  this.Serial.on('data',this.onData.bind(this));
}

AzzyRF.prototype.onData = function(data) {
	if (data=="#" && this.datastring) {
		if (this.datastring) {
			this.Serial.print(this.datastring);
			this.datastring="";
		}
	} else if (data==">" && this.datastring) {
		//TODO
	} else if (data=="\n" || data=="\r"){
		if (this.inString!="") {
			this.outputFormat(this.inString);
			this.inString="";
		}
	} else {
		this.inString+=data;
    if (this.timeout > 0) {
      clearTimeout(this.timeout);
		}
  }
    this.timeout=setTimeout(function() {this.outputFormat(this.inString);this.inString='';this.timeout=0;}.bind(this),1000);
};

AzzyRF.prototype.writeA24 = function(addr,data) {
	if (data.length > 16) {
		throw "Data too long";
	} else {
      var tstr="";
		//tstr=E.toString([(addr>>8)&255,addr&255,data.length]);
        tstr+=String.fromCharCode((addr>>8)&255,addr&255,data.length);
		//tstr+=E.toString(data);
        tstr+=data;
		this.datastring=tstr;
		this.Serial.print("AT+24WL\r");
	}
};

AzzyRF.prototype.outputFormat = function(text) {
  if (text=="") return;
  var outstr="";
  if (text.charAt(0)=='+' || text.charAt(0)=='=') {
	  var len=text.length;
	  for (var i=1;i<len;i+=2) {
	    //console.log(i);
	    //console.log(text.substr(i,2));
	    var tnum=parseInt(text.substr(i,2),16);
	    if (!isNaN(tnum)) {
	      outstr+=String.fromCharCode(tnum);
	    }
	  }
	  if (outstr!="") {
	  	//console.log(outstr);
	  	if ((text.charAt(0)=='+')) {
	  		if (typeof this.onRcvOut == 'function') this.onRcvOut(E.toUint8Array(outstr));
	  	} else {
	  		if (typeof this.onDataOut == 'function') this.onDataOut(E.toUint8Array(outstr));
	  	}
		}
	} else {
		text=text.trim();
	  if (text!="") {
	  //console.log(outstr);
	  	if (typeof this.onTextOut == 'function'){
	  	  this.onTextOut(text);
	  	} else {
	  		console.log("other message: "+text);
	  	}
	  }
	}
	if (typeof this.onMsgOut == 'function') {
		this.onMsgOut(text);
	} else {
		console.log(text);
	}
};


AzzyRF.prototype.readA24 = function(addr,len) {
	if (len > 16) {
		throw "Data too long";
	} else {
		tstr=E.toString([(addr>>8)&255,addr&255,len]);
		this.datastring=tstr;
		this.Serial.print("AT+24RL\r");
	}
};

AzzyRF.prototype.send = function (addr,cmd,data) {
	if (data.length==2) {
		this.datastring=E.toString([addr&0x3F,cmd,data[0],data[1]]);
		this.Serial.print("AT+SEND\r");
	} else if (data.length==5) {
		this.datastring=E.toString([addr&0x3F,cmd])+E.toString(data);
		this.Serial.print("AT+SENDM\r");
	} else if (data.length==13) {
		this.datastring=E.toString([addr&0x3F,cmd])+E.toString(data);
		this.Serial.print("AT+SENDL\r");
	} else if (data.length==29) {
		this.datastring=E.toString([addr&0x3F,cmd])+E.toString(data);
		this.Serial.print("AT+SENDE\r");
	} else {
		throw "Invalid Length";
	}
};

AzzyRF.prototype.setRFConfig = function(set) {
	tarr=new Uint8Array(28);
	tarr[0]=set.txSyncTime;
	tarr[1]=set.txSyncTime>>8;
	tarr[2]=set.txTrainRep;
	tarr[3]=set.txTrainTime;
	tarr[4]=set.txTrainTime>>8;
	tarr[5]=set.txOneLength;
	tarr[6]=set.txOneLength>>8;
	tarr[7]=set.txZeroLength;
	tarr[8]=set.txZeroLength>>8;
	tarr[9]=set.txLowTime;
	tarr[10]=set.txLowTime>>8;
	tarr[11]=set.rxSyncMin;
	tarr[12]=set.rxSyncMin>>8;
	tarr[13]=set.rxSyncMax;
	tarr[14]=set.rxSyncMax>>8;
	tarr[15]=set.rxZeroMin;
	tarr[16]=set.rxZeroMin>>8;
	tarr[17]=set.rxZeroMax;
	tarr[18]=set.rxZeroMax>>8;
	tarr[19]=set.rxOneMin;
	tarr[20]=set.rxOneMin>>8;
	tarr[21]=set.rxOneMax;
	tarr[22]=set.rxOneMax>>8;
	tarr[23]=set.rxLowMax;
	tarr[24]=set.rxLowMax>>8;
	tarr[25]=set.txRepDelay;
	tarr[26]=set.txRepDelay>>8;
	tarr[27]=set.txRepCount;
	this.datastring=E.toString(tarr);
	console.log("Writing to config EEPROM on AzzyRF");
	this.Serial.print("AT+CONF\r");
};
