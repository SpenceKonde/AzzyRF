
var txpin=A1;


function sendCmd(add,cmd,param,ext) {
  for (r=0;r<5;r++) {
  ext=ext?ext&15:0;
    var csc=add^cmd^param;
    csc=(csc&15)^(csc>>4)^ext;
    var b=(add<<20)+(cmd<<12)+(param<<4)+ext;
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
