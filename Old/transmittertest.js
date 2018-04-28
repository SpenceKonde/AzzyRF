    var txpin=A1;


function sendTest3(data) {
    var d = digitalPulse, p=txpin;
    for (i = 0; i < 20; i++) {
        d(p,1,0.5); 
        d(p,0,0.5); 
    } 
    d(p,1,0.5); //end training burst
    d(p,0,2);
    //send 1s or 0s, and pause
    data.forEach(function(v) {
        d(p,1,v?1.1:0.6); 
        d(p,0,0.65);
    });
    // wait for finish
    d(p,0,0);
  digitalWrite(p,0);
}

var data = [0,1,0,1,1,0,1,1,
            1,0,0,0,1,1,0,0,
            1,1,1,0,1,0,0,1,
            1,1,0,1,0,0,0,0];


function sendTest4(b1,b2,b3,b4) {
    var d = digitalPulse, p=txpin;
    for (i = 0; i < 20; i++) {
        d(p,1,0.5); 
        d(p,0,0.5); 
    } 
    d(p,1,0.5); //end training burst
    d(p,0,2);
    //send 1s or 0s, and pause
    for (j=7;j>-1;j--) {
        d(p,1,((b1>>j)&1)?1.1:0.6); 
        d(p,0,0.65);
    }
    for (j=7;j>-1;j--) {
        d(p,1,((b2>>j)&1)?1.1:0.6); 
        d(p,0,0.65);
    }
    for (j=7;j>-1;j--) {
        d(p,1,((b3>>j)&1)?1.1:0.6); 
        d(p,0,0.65);
    }
    for (j=7;j>-1;j--) {
        d(p,1,((b4>>j)&1)?1.1:0.6); 
        d(p,0,0.65);
    }
    // wait for finish
    d(p,0,0);
  digitalWrite(p,0);
}


function sendCmd(add,cmd,param,ext) {
  ext=ext?ext&15:0;
    var csc=add^cmd^param;
    csc=(csc&15)^(csc>>4)^ext;
    var b=(add<<20)+(cmd<<12)+(param<<4)+ext;
    var d = digitalPulse;
    var p=txpin;
    for (i = 0; i < 40; i++) {
        d(p,1,0.3); 
        d(p,0,0.3); 
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
  digitalWrite(p,0);
}

function testRun(train,opp,zpp,number,interval) {
  tp=train;
  op=opp;
  zp=zpp;
  var ad=Math.round(255*Math.random());
  var pr=Math.round(255*Math.random());
  var cm=Math.round(198*Math.random());
  var ex=Math.round(16*Math.random());
  inter=setInterval("sendCmd(ad,cm,pr,ex);ad=pr=Math.round(255*Math.random());cm=Math.round(198*Math.random());ex=Math.round(16*Math.random());",interval);
  setTimeout("clearInterval(inter);",number*interval);
  setTimeout("sendCmd(31,200,150,10);console.log('done')",(number+5)*interval);
}