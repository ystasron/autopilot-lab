(()=>{
const $=i=>document.getElementById(i),cv=$('c'),g=cv.getContext('2d');
const X=[110,340,570,800],Y=[90,280,470],NC=4,NR=3,LN=12,STOP=52,SW=31;
const D=[{x:1,y:0},{x:0,y:1},{x:-1,y:0},{x:0,y:-1}],rt=d=>({x:-D[d].y,y:D[d].x});
const np=ni=>({x:X[ni%NC],y:Y[Math.floor(ni/NC)]});
const nb=(ni,d)=>{const c=ni%NC+D[d].x,r=Math.floor(ni/NC)+D[d].y;return c<0||c>=NC||r<0||r>=NR?-1:r*NC+c};
const stopPt=(ni,d,s=-1)=>{const p=np(ni),r=rt(d);return{x:p.x+s*D[d].x*STOP+r.x*LN,y:p.y+s*D[d].y*STOP+r.y*LN}};
const dd=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y),wrap=a=>Math.atan2(Math.sin(a),Math.cos(a)),clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),rnd=(a,b)=>a+Math.random()*(b-a);
const COLS=['#e6e9ee','#9aa4b2','#2b3038','#c0392b','#2f6fb5','#d9a441'];
const LIM=60,S={sn:{path:1,corr:1,lidar:1,cam:1,det:1},auto:true,sens:true,max:70,ped:[],hit:0,off:0,keys:{},cam:'free'},sh=Array(12).fill(0);
let clock=0,vdt=0,cars=[],rays=[],touching=false,outside=false;
/* camera: render-time only. 'follow' eases toward the ego car, clamped to the city box */
const cam={x:450,y:280,z:1,on:'free'},FOZ=1.9;
const camInv=(x,y)=>S.cam==='follow'?{x:(x-450)/cam.z+cam.x,y:(y-280)/cam.z+cam.y}:{x,y};
/* traffic lights: axis 0 = north-south, 1 = east-west; 21s cycle with all-red gaps */
const L=(ni,ax)=>{const a=(clock+((ni*7)%4)*5+sh[ni]+(ax?10.5:0))%21;return a<8?'green':a<9.5?'yellow':'red'};
/* pedestrian graph: 4 corners per intersection (NE,SE,SW,NW) */
const SWK=[[[0,3],[3,1]],[[0,2],[1,0]],[[2,1],[1,3]],[[2,0],[3,2]]];
const pnb=id=>{const ni=id>>2,k=id&3,o=[ni*4+(k+1)%4,ni*4+(k+3)%4];for(const[d,k2]of SWK[k]){const n=nb(ni,d);if(n>=0)o.push(n*4+k2)}return o};
const cp=id=>{const p=np(id>>2),k=id&3;return{x:p.x+[1,1,-1,-1][k]*SW,y:p.y+[-1,1,1,-1][k]*SW}};
const walkOK=(a,b)=>{const ni=a>>2,arm=((a&3)+1)%4===(b&3)?a&3:b&3;return nb(ni,arm)<0||L(ni,arm%2===0?1:0)==='red'};
function lin(p,a,b){const n=Math.max(1,Math.round(dd(a,b)/20));for(let k=1;k<=n;k++)p.push({x:a.x+(b.x-a.x)*k/n,y:a.y+(b.y-a.y)*k/n})}
function setPath(c,p){c.path=p;c.cum=[0];for(let i=1;i<p.length;i++)c.cum.push(c.cum[i-1]+dd(p[i],p[i-1]));c.pi=0}
const PC=['#f778ba','#58a6ff','#e3b341','#a5d6a7','#d2a8ff'];
const turnOf=c=>c.pend>=0?(c.pend-c.din+4)%4:0;
function pick(c){if(c.route.length)return c.route[0];const ni=c.node,din=c.din,o=[0,1,2,3].filter(d=>d!==(din+2)%4&&nb(ni,d)>=0),w=c.ego?{left:(din+3)%4,straight:din,right:(din+1)%4}[$('turn').value]:-1;
if(o.includes(w))return w;if(c.pend>=0&&o.includes(c.pend))return c.pend;return o[Math.floor(Math.random()*o.length)]}
function mk(ni,din,dist,ego){const q=Math.random(),ty=ego?1:q<.25?0:q<.8?1:2,PER=[{m:.85,hw:1.3,b:110,rt:1.2},{m:1,hw:.9,b:150,rt:.7},{m:1.12,hw:.6,b:220,rt:.4}][ty],E=stopPt(ni,din),s={x:E.x-D[din].x*dist,y:E.y-D[din].y*dist},p=[s];lin(p,s,E);
const c={x:s.x,y:s.y,a:Math.atan2(D[din].y,D[din].x),v:0,steer:0,node:ni,din,ego,r:10,w:9,len:14,max:ego?S.max:LIM*PER.m*rnd(.97,1.03),hw:PER.hw,b:PER.b,rt:ego?.2:PER.rt,ty,col:ego?'#7cffb2':COLS[Math.floor(Math.random()*6)],stuck:0,ghost:0,front:130,st:'–',pend:-1,sigT:0,blink:0,brake:false,cl:0,final:false,route:[],dest:null,wait:0,dl:0,dec:0};
c.pend=pick(c);setPath(c,p);return c}
function plan(c){const ni=c.node,din=c.din,rr=c.route.length>0,dout=pick(c);if(rr)c.route.shift();const fin=rr&&c.route.length===0&&c.dest;c.sigT=(dout-din+4)%4;
const P1=stopPt(ni,din),P2=stopPt(ni,dout,1),p=np(ni),ri=rt(din),ro=rt(dout);
const C=din===dout?{x:(P1.x+P2.x)/2,y:(P1.y+P2.y)/2}:{x:p.x+LN*(ri.x+ro.x),y:p.y+LN*(ri.y+ro.y)},pts=[];
for(let k=0;k<=10;k++){const t=k/10,u=1-t;pts.push({x:u*u*P1.x+2*u*t*C.x+t*t*P2.x,y:u*u*P1.y+2*u*t*C.y+t*t*P2.y})}
const nx=nb(ni,dout);const sd0=c.dest?(ni===c.dest.a?c.dest.s:c.dest.len-c.dest.s):0,dp={x:p.x+D[dout].x*sd0+rt(dout).x*LN,y:p.y+D[dout].y*sd0+rt(dout).y*LN};lin(pts,P2,fin?dp:stopPt(nx,dout));c.final=!!fin;c.cl=11;c.node=nx;c.dec=0;c.din=dout;c.pend=-1;c.pend=pick(c);setPath(c,pts)}
function addCars(k){for(let j=0;j<k&&cars.length<31;j++)for(let t=0;t<20;t++){const ni=Math.floor(Math.random()*12),din=Math.floor(Math.random()*4);if(nb(ni,(din+2)%4)<0)continue;
const c=mk(ni,din,rnd(10,60));c.v=c.max*.6;if(cars.every(o=>dd(o,c)>45)){randDest(c);cars.push(c);break}}}
function addPeople(k){for(let j=0;j<k&&S.ped.length<40;j++){const a=Math.floor(Math.random()*48),o=pnb(a);S.ped.push({a,b:o[Math.floor(Math.random()*o.length)],t:Math.random(),sp:rnd(18,30),r:5,w:5,len:5,x:0,y:0,col:PC[Math.floor(Math.random()*5)]})}}
function reset(){cars[0]=mk(5,0,60,true);S.trip='–';S.hit=S.off=0}
function resume(c){c.final=false;c.wait=0;const s={x:c.x,y:c.y},p=[s];lin(p,s,stopPt(c.node,c.din));setPath(c,p);c.cl=0;if(!c.ego)randDest(c)}
function randDest(c){for(let t=0;t<9;t++){const a=Math.floor(Math.random()*12),d=Math.random()<.5?0:1,b=nb(a,d);if(b<0)continue;setDest(c,a,b,rnd(72,dd(np(a),np(b))-72));return}}
const exitBlocked=c=>{const bp=np(c.node);if(cars.some(o=>o!==c&&o.v<3&&Math.abs(o.x-bp.x)<27&&Math.abs(o.y-bp.y)<27))return true;if(c.pend<0)return false;const E=stopPt(c.node,c.pend,1),ax=D[c.pend].x,ay=D[c.pend].y;
return cars.some(o=>{if(o===c||o.v>=10)return false;const dx=o.x-E.x,dy=o.y-E.y,al=dx*ax+dy*ay;return al>-15&&al<45&&Math.abs(-dx*ay+dy*ax)<12})};
function route(c,t){const EL=d=>d%2?190:230,dist=Array(48).fill(1e9),prv=Array(48).fill(-1),done=[];dist[c.node*4+c.din]=0;
for(let it=0;it<48;it++){let k=-1;for(let i=0;i<48;i++)if(!done[i]&&(k<0||dist[i]<dist[k]))k=i;if(k<0||dist[k]>=1e9)break;done[k]=1;const n=k>>2,d=k&3;
for(let o=0;o<4;o++){if(o===(d+2)%4)continue;const m=nb(n,o);if(m<0)continue;const j=m*4+o,w=dist[k]+EL(o);if(w<dist[j]){dist[j]=w;prv[j]=k}}}
let best=1e9,bk=0,bg=0;[[t.a,t.b,t.s],[t.b,t.a,t.len-t.s]].forEach(([u,v,su])=>{const gd=[0,1,2,3].find(d=>nb(u,d)===v);
for(let d=0;d<4;d++){if(d===(gd+2)%4)continue;const w=dist[u*4+d]+su;if(w<best){best=w;bk=u*4+d;bg=gd}}});
const r=[bg];for(let k=bk;prv[k]>=0;k=prv[k])r.unshift(k&3);return r}
function setDest(e,a,b,s){const len=dd(np(a),np(b));s=clamp(s,72,len-72);e.dest={a,b,s,len};if(e.ego)S.trip='en route';if(e.final)resume(e);e.route=[];
const pv=nb(e.node,(e.din+2)%4);
if(pv>=0&&e.pi>=e.cl&&((a===pv&&b===e.node)||(b===pv&&a===e.node))){const sp=a===pv?s:len-s,q=np(pv),P={x:q.x+D[e.din].x*sp+rt(e.din).x*LN,y:q.y+D[e.din].y*sp+rt(e.din).y*LN};
if((P.x-e.x)*D[e.din].x+(P.y-e.y)*D[e.din].y>30){const p=[{x:e.x,y:e.y}];lin(p,p[0],P);setPath(e,p);e.cl=0;e.final=true;return}}
e.route=route(e,e.dest)}
const dcen=e=>{const A=np(e.dest.a),B=np(e.dest.b),t=e.dest.s/e.dest.len;return{x:A.x+(B.x-A.x)*t,y:A.y+(B.y-A.y)*t}};
function routeLine(){const e=cars[0];if(!e.dest)return;const pts=e.path.slice(e.pi);
if(!e.final){let n=e.node;for(const d of e.route){pts.push(np(n));n=nb(n,d)}pts.push(dcen(e))}
g.save();g.strokeStyle='rgba(88,166,255,.8)';g.lineWidth=3;g.setLineDash([8,7]);g.beginPath();g.moveTo(e.x,e.y);pts.forEach(p=>g.lineTo(p.x,p.y));g.stroke();g.restore()}
function pin(){const e=cars[0];if(!e.dest)return;const p=dcen(e),k=(clock*14)%16;
g.save();g.strokeStyle='rgba(255,77,109,'+(1-k/16)*.8+')';g.lineWidth=2;g.beginPath();g.arc(p.x,p.y,4+k,0,7);g.stroke();
g.fillStyle='#ff4d6d';g.beginPath();g.moveTo(p.x,p.y);g.lineTo(p.x-6,p.y-12);g.lineTo(p.x+6,p.y-12);g.fill();g.beginPath();g.arc(p.x,p.y-15,8,0,7);g.fill();
g.fillStyle='#fff';g.beginPath();g.arc(p.x,p.y-15,3,0,7);g.fill();g.restore()}
const oncoming=c=>{const p=np(c.node);return cars.some(o=>o!==c&&dd(o,p)<100&&o.v>3&&Math.cos(o.a-c.a)<-.7)};
function stepCar(c,dt,all){
let P=c.path,last=P.length-1,steer=0,lim=1e9,rem=0,st='–',hold=false,err=0,ux=Math.cos(c.a),uy=Math.sin(c.a);
const auto=!(c.ego&&!S.auto),v0=c.v;if(c.ego)c.pend=pick(c);
if(auto){
while(c.pi<last&&dd(c,P[c.pi])<14)c.pi++;
rem=dd(c,P[c.pi])+c.cum[last]-c.cum[c.pi];st=L(c.node,c.din%2===0?1:0);
if(c.final){st='–';if(rem<4&&c.v<3&&!c.wait){c.wait=c.ego?1e12:clock+1.5+Math.random();if(c.ego){S.trip='arrived';c.dest=null;c.route=[]}}
if(c.wait&&clock>c.wait){resume(c);P=c.path;last=P.length-1;rem=c.cum[last];st=L(c.node,c.din%2===0?1:0)}}
else{hold=(rem<70&&turnOf(c)===3&&oncoming(c))||(rem<40&&st==='green'&&exitBlocked(c));
if(rem<10&&(st==='green'||c.dec===2)&&!hold){plan(c);P=c.path;last=P.length-1;rem=c.cum[last];st=c.final?'–':L(c.node,c.din%2===0?1:0);hold=false}}
const tg=P[Math.min(c.pi+2+Math.floor(c.v/30),last)],d=dd(tg,c)||1;err=wrap(Math.atan2(tg.y-c.y,tg.x-c.x)-c.a);ux=(tg.x-c.x)/d;uy=(tg.y-c.y)/d}
let F=130;for(const o of all){if(o===c||(c.ghost>0&&o.len===14))continue;const dx=o.x-c.x,dy=o.y-c.y,f=dx*ux+dy*uy,l=-dx*uy+dy*ux;
if(f>0&&f<130&&Math.abs(l)<o.w+10)F=Math.min(F,Math.max(0,f-o.len-14))}c.front=F;
if(auto){c.st=st;
const need=c.v*c.v/(2*Math.max(rem-3,.1));if(st==='green'||st==='–')c.dec=0;else if(!c.dec)c.dec=need<(st==='red'?300:c.b)?1:2;const sl=c.dec===1;if(c.final)lim=Math.sqrt(200*Math.max(rem-2,0));else if(sl||(hold&&!(rem-2<c.v*c.v/600)))lim=Math.sqrt(200*Math.max(rem-3,0));
const tn=c.final?0:turnOf(c);if(tn===1||tn===3)lim=Math.min(lim,Math.sqrt((tn===1?22:30)**2+200*Math.max(rem-10,0)));
if(c.pi<c.cl&&c.sigT)lim=Math.min(lim,c.sigT===1?22:30);
let tgt=Math.max(0,Math.min(c.max,lim,(F-12)/c.hw))*(1-Math.min(.5,Math.abs(err)*.6));if(c.v<1&&tgt>3){c.dl+=dt;if(c.dl<c.rt)tgt=0}else c.dl=0;
c.v=Math.max(0,c.v+clamp(tgt-c.v,-250*dt,45*dt));steer=clamp(err*1.1,-.7,.7);
if(c.v<2&&F<25&&!(rem<45&&(c.dec===1||hold)))c.stuck+=dt;else c.stuck=0;if(c.stuck>(c.ego?6:10)){c.ghost=2;c.stuck=0}c.ghost-=dt;
c.blink=c.pi<c.cl&&c.sigT?c.sigT:!c.final&&rem<120?turnOf(c):0}
else{const k=S.keys;steer=((k.ArrowRight?1:0)-(k.ArrowLeft?1:0))*.5;c.v=clamp(c.v+(k.ArrowUp?70:k.ArrowDown?-160:-25)*dt,0,S.max*1.3);c.st='–';c.blink=0}
c.steer+=(steer-c.steer)*Math.min(1,10*dt);c.a+=c.v/18*Math.tan(c.steer)*dt;c.x+=Math.cos(c.a)*c.v*dt;c.y+=Math.sin(c.a)*c.v*dt;
c.brake=c.v<v0-.05||(c.v<3&&c.st!=='green'&&c.st!=='–')}
const onRoad=(x,y)=>X.some(v=>Math.abs(x-v)<38&&y>Y[0]-38&&y<Y[NR-1]+38)||Y.some(v=>Math.abs(y-v)<38&&x>X[0]-38&&x<X[NC-1]+38);
function step(dt){
clock+=dt;
S.ped.forEach(p=>{const A=cp(p.a),B=cp(p.b),len=dd(A,B);
if(p.t===0&&(p.a>>2)===(p.b>>2)&&!walkOK(p.a,p.b)){p.x=A.x;p.y=A.y;return}
p.t=Math.min(1,p.t+p.sp*dt/len);p.x=A.x+(B.x-A.x)*p.t;p.y=A.y+(B.y-A.y)*p.t;
if(p.t>=1){const o=pnb(p.b).filter(id=>id!==p.a),n=o.length?o[Math.floor(Math.random()*o.length)]:p.a;p.a=p.b;p.b=n;p.t=0}});
const all=cars.concat(S.ped);cars.forEach(c=>stepCar(c,dt,all));
const e=cars[0];rays=[];
for(let k=-4;k<=4;k++){const an=e.a+k*.28,dx=Math.cos(an),dy=Math.sin(an);let d=100;
for(const o of all){if(o===e)continue;const fx=o.x-e.x,fy=o.y-e.y,pr=fx*dx+fy*dy;if(pr<0)continue;const pd=Math.abs(fx*dy-fy*dx),R=o.r+2;if(pd<R){const h=pr-Math.sqrt(R*R-pd*pd);if(h<d)d=Math.max(h,0)}}rays.push({an,d})}
const hit=all.some(o=>o!==e&&dd(o,e)<o.r+10);if(hit&&!touching)S.hit++;touching=hit;
const out=!onRoad(e.x,e.y);if(out&&!outside)S.off++;outside=out}
/* static city layer */
const bg=document.createElement('canvas');bg.width=900;bg.height=560;
let sd=7;const rn=()=>(sd=sd*16807%2147483647)/2147483647;
(()=>{const b=bg.getContext('2d'),x0=X[0],x1=X[NC-1],y0=Y[0],y1=Y[NR-1],B=['#1c2430','#222c3a','#2a2f3b','#182029'];
b.fillStyle='#0e1319';b.fillRect(0,0,900,560);
for(let i=0;i<NC-1;i++)for(let j=0;j<NR-1;j++){const bx=X[i]+44,by=Y[j]+44,bw=X[i+1]-X[i]-88,bh=Y[j+1]-Y[j]-88;
if((i+j)%3===1){b.fillStyle='#13261a';b.fillRect(bx,by,bw,bh);for(let k=0;k<14;k++){const tx=bx+12+rn()*(bw-24),ty=by+12+rn()*(bh-24),r=8+rn()*7;
b.fillStyle='rgba(0,0,0,.3)';b.beginPath();b.arc(tx+3,ty+4,r,0,7);b.fill();b.fillStyle=k%2?'#2a6b3a':'#23593a';b.beginPath();b.arc(tx,ty,r,0,7);b.fill();b.fillStyle='rgba(255,255,255,.08)';b.beginPath();b.arc(tx-2,ty-2,r*.5,0,7);b.fill()}continue}
for(let a=0;a<2;a++)for(let e=0;e<2;e++){const w=bw/2-6,h=bh/2-6,x=bx+a*bw/2+3,y=by+e*bh/2+3;
b.fillStyle='rgba(0,0,0,.4)';b.fillRect(x+6,y+7,w,h);b.fillStyle=B[Math.floor(rn()*4)];b.fillRect(x,y,w,h);b.fillStyle='rgba(255,255,255,.06)';b.fillRect(x+5,y+5,w-10,h-10);b.fillStyle='#39424f';b.fillRect(x+8+rn()*(w-30),y+8+rn()*(h-26),10,8)}}
for(const[w,c]of[[76,'#2b323c'],[50,'#556070'],[48,'#1b2027']]){b.fillStyle=c;Y.forEach(y=>b.fillRect(x0-w/2,y-w/2,x1-x0+w,w));X.forEach(x=>b.fillRect(x-w/2,y0-w/2,w,y1-y0+w))}
for(let k=0;k<700;k++){const x=rn()*900,y=rn()*560;if(onRoad(x,y)){b.fillStyle='rgba(255,255,255,'+rn()*.035+')';b.fillRect(x,y,2,2)}}
b.setLineDash([12,10]);b.strokeStyle='#c9a227';b.lineWidth=2;
for(let i=0;i<NC;i++)for(let j=0;j<NR;j++){if(i<NC-1){b.beginPath();b.moveTo(X[i]+40,Y[j]);b.lineTo(X[i+1]-40,Y[j]);b.stroke()}if(j<NR-1){b.beginPath();b.moveTo(X[i],Y[j]+40);b.lineTo(X[i],Y[j+1]-40);b.stroke()}}
for(let ni=0;ni<12;ni++)for(let d=0;d<4;d++){const p=np(ni),r=rt(d);
if(nb(ni,d)>=0){b.fillStyle='rgba(230,237,243,.75)';for(let j=0;j<6;j++){const t=-20+j*8,x=p.x+D[d].x*SW+r.x*t,y=p.y+D[d].y*SW+r.y*t;d%2?b.fillRect(x-2,y-5,4,10):b.fillRect(x-5,y-2,10,4)}}
if(nb(ni,(d+2)%4)>=0){b.fillStyle='#dfe6ee';const cx=p.x-D[d].x*37+r.x*12,cy=p.y-D[d].y*37+r.y*12;d%2?b.fillRect(cx-11,cy-1.5,22,3):b.fillRect(cx-1.5,cy-11,3,22)}}
for(let id=0;id<48;id++){const q=cp(id),k=id&3;b.fillStyle='#8b93a1';b.beginPath();b.arc(q.x+[1,1,-1,-1][k]*7,q.y+[-1,1,1,-1][k]*7,2,0,7);b.fill()}})();
const lampG=document.createElement('canvas');lampG.width=lampG.height=110;
(()=>{const b=lampG.getContext('2d'),gr=b.createRadialGradient(55,55,0,55,55,55);gr.addColorStop(0,'rgba(255,220,150,.45)');gr.addColorStop(1,'rgba(255,220,150,0)');b.fillStyle=gr;b.fillRect(0,0,110,110)})();
const hl=document.createElement('canvas');hl.width=110;hl.height=70;
(()=>{const b=hl.getContext('2d'),gr=b.createRadialGradient(0,35,0,0,35,110);gr.addColorStop(0,'rgba(255,240,200,.6)');gr.addColorStop(1,'rgba(255,240,200,0)');b.fillStyle=gr;b.beginPath();b.moveTo(0,35);b.lineTo(110,0);b.lineTo(110,70);b.closePath();b.fill()})();
const LC={green:'#3fb950',yellow:'#e3b341',red:'#f85149'};
function body(c){g.save();g.translate(c.x,c.y);g.rotate(c.a);g.fillStyle='rgba(0,0,0,.35)';g.beginPath();g.roundRect(-13,-5,28,16,4);g.fill();
g.fillStyle=c.col;g.beginPath();g.roundRect(-14,-8,28,16,4);g.fill();g.fillStyle='rgba(0,0,0,.55)';
g.beginPath();g.roundRect(3,-6.5,6,13,2);g.fill();g.beginPath();g.roundRect(-11,-6,4,12,1.5);g.fill();g.fillStyle='rgba(255,255,255,.14)';g.fillRect(-6,-6.5,9,13);g.restore()}
function lights(c,n){g.save();g.translate(c.x,c.y);g.rotate(c.a);if(n){g.globalCompositeOperation='lighter';g.drawImage(hl,12,-35)}
g.fillStyle='#fff6d8';g.fillRect(12,-7,2,3);g.fillRect(12,4,2,3);g.fillStyle=c.brake?'#ff3b30':'#7a1712';g.fillRect(-14,-7,2,3);g.fillRect(-14,4,2,3);
if(c.blink&&clock%.8<.4){g.fillStyle='#ffb000';const y=c.blink===3?-8:6;g.fillRect(12,y,2,2);g.fillRect(-14,y,2,2)}g.restore()}
function signals(n){for(let ni=0;ni<12;ni++)for(let d=0;d<4;d++){if(nb(ni,(d+2)%4)<0)continue;const p=np(ni),r=rt(d),x=p.x-D[d].x*(STOP+3)+r.x*32,y=p.y-D[d].y*(STOP+3)+r.y*32,s=L(ni,d%2===0?1:0);
g.fillStyle='#0a0d12';g.beginPath();g.roundRect(x-3.5,y-9,7,18,2);g.fill();
['red','yellow','green'].forEach((m,k)=>{const on=s===m;g.fillStyle=on?LC[m]:'#2a2f38';g.beginPath();g.arc(x,y-5+k*5,1.9,0,7);g.fill();
if(on&&n){g.save();g.globalCompositeOperation='lighter';g.globalAlpha=.3;g.beginPath();g.arc(x,y-5+k*5,9,0,7);g.fill();g.restore()}})}}
const M=.17;
function sensors(){const e=cars[0],n=S.sn,all=cars.concat(S.ped),P=e.path,up=[{x:e.x,y:e.y}].concat(P.slice(e.pi,e.pi+14));let cnt=0;
const poly=a=>{g.beginPath();a.forEach((p,i)=>i?g.lineTo(p.x,p.y):g.moveTo(p.x,p.y))};
const lab=(t,x,y,c)=>{g.font='10px system-ui';const w=g.measureText(t).width;g.fillStyle='rgba(13,17,23,.85)';g.fillRect(x,y-10,w+6,13);g.fillStyle=c;g.fillText(t,x+3,y)};
g.save();g.lineJoin='round';
if(S.auto&&up.length>1){
if(n.corr){const Lp=[],Rp=[];up.forEach((p,i)=>{const a=up[Math.max(0,i-1)],b=up[Math.min(up.length-1,i+1)],d=Math.hypot(b.x-a.x,b.y-a.y)||1,nx=-(b.y-a.y)/d*12,ny=(b.x-a.x)/d*12;Lp.push({x:p.x+nx,y:p.y+ny});Rp.push({x:p.x-nx,y:p.y-ny})});
poly(Lp.concat(Rp.slice().reverse()));g.closePath();g.fillStyle='rgba(63,185,80,.16)';g.fill();g.strokeStyle='rgba(63,185,80,.8)';g.lineWidth=1.5;poly(Lp);g.stroke();poly(Rp);g.stroke()}
if(n.path){g.strokeStyle='#58d5ff';g.lineWidth=2.5;poly(up);g.stroke();g.fillStyle='#58d5ff';up.slice(1).forEach(p=>{g.beginPath();g.arc(p.x,p.y,2,0,7);g.fill()});
const t=P[Math.min(e.pi+2+Math.floor(e.v/30),P.length-1)];g.strokeStyle='#fff';g.lineWidth=1.5;g.beginPath();g.arc(t.x,t.y,5,0,7);g.stroke()}}
if(n.cam){g.fillStyle='rgba(227,177,65,.13)';g.strokeStyle='rgba(227,177,65,.65)';g.lineWidth=1.5;g.beginPath();g.moveTo(e.x,e.y);g.arc(e.x,e.y,170,e.a-.61,e.a+.61);g.closePath();g.fill();g.stroke()}
if(n.lidar){const R=130;g.strokeStyle='rgba(255,123,114,.15)';g.lineWidth=1;g.beginPath();g.arc(e.x,e.y,R,0,7);g.stroke();
const sw=clock*6;g.strokeStyle='rgba(255,123,114,.45)';g.beginPath();g.moveTo(e.x,e.y);g.lineTo(e.x+Math.cos(sw)*R,e.y+Math.sin(sw)*R);g.stroke();
for(let k=0;k<90;k++){const an=k/90*6.2832,dx=Math.cos(an),dy=Math.sin(an);let ds=R,dy2=R;
for(let s=6;s<R;s+=4)if(!onRoad(e.x+dx*s,e.y+dy*s)){ds=s;break}
for(const o of all){if(o===e)continue;const fx=o.x-e.x,fy=o.y-e.y,pr=fx*dx+fy*dy;if(pr<0||pr>R)continue;const pd=Math.abs(fx*dy-fy*dx),rr=o.r+2;if(pd<rr)dy2=Math.min(dy2,Math.max(0,pr-Math.sqrt(rr*rr-pd*pd)))}
const dyn=dy2<ds,d=Math.min(ds,dy2);if(d>=R)continue;g.fillStyle=dyn?'#ff7b72':'rgba(121,192,255,.85)';g.fillRect(e.x+dx*d-1.5,e.y+dy*d-1.5,3,3)}}
if(n.det){all.forEach((o,i)=>{if(o===e)return;const dx=o.x-e.x,dy=o.y-e.y,dist=Math.hypot(dx,dy);if(dist>170||Math.abs(wrap(Math.atan2(dy,dx)-e.a))>.61)return;
const ped=o.len===5,c=ped?'#f778ba':'#d2a8ff';cnt++;g.save();g.translate(o.x,o.y);if(!ped)g.rotate(o.a);g.strokeStyle=c;g.lineWidth=1.5;ped?g.strokeRect(-7,-7,14,14):g.strokeRect(-17,-11,34,22);g.restore();
lab((ped?'person ':'car ')+(dist*M).toFixed(0)+'m '+(.9+(i*37%9)/100).toFixed(2),o.x-16,o.y-(ped?10:14),c)});
if(LC[e.st]&&!e.final){const p=np(e.node),r=rt(e.din),lx=p.x-D[e.din].x*(STOP+3)+r.x*32,ly=p.y-D[e.din].y*(STOP+3)+r.y*32,dist=Math.hypot(lx-e.x,ly-e.y);
if(dist<260&&(Math.abs(wrap(Math.atan2(ly-e.y,lx-e.x)-e.a))<.8||dist<60)){cnt++;g.strokeStyle=LC[e.st];g.lineWidth=1.5;g.strokeRect(lx-6,ly-11,12,22);lab('signal '+e.st+' '+(dist*M).toFixed(0)+'m',lx-20,ly-15,LC[e.st])}}}
g.restore();$('dt').textContent=n.det?cnt:'–'}
function draw(){
const ce=cars[0];g.save();
if(S.cam==='follow'){const hx=450/FOZ,hy=280/FOZ,tx=clamp(ce.x,hx,900-hx),ty=clamp(ce.y,hy,560-hy),k=Math.min(1,vdt*6);
if(cam.on!=='follow'){cam.x=tx;cam.y=ty;cam.z=1;cam.on='follow'}
cam.x+=(tx-cam.x)*k;cam.y+=(ty-cam.y)*k;cam.z+=(FOZ-cam.z)*k;
g.translate(450,280);g.scale(cam.z,cam.z);g.translate(-cam.x,-cam.y)}
else{cam.on='free';cam.z=1;cam.x=450;cam.y=280}
g.drawImage(bg,0,0);routeLine();
S.ped.forEach(p=>{const A=cp(p.a),B=cp(p.b);g.save();g.translate(p.x,p.y);g.rotate(Math.atan2(B.y-A.y,B.x-A.x));g.fillStyle='rgba(0,0,0,.3)';g.beginPath();g.ellipse(1,1,3,5,0,0,7);g.fill();
g.fillStyle=p.col;g.beginPath();g.ellipse(0,0,2.5,5,0,0,7);g.fill();g.fillStyle='#e0b48f';g.beginPath();g.arc(.5,0,2.4,0,7);g.fill();g.restore()});
const e=cars[0];cars.slice(1).forEach(body);e.col=touching?'#ff5a5a':'#7cffb2';body(e);
if(S.night){g.fillStyle='rgba(4,8,24,.58)';g.fillRect(0,0,900,560);g.save();g.globalCompositeOperation='lighter';for(let id=0;id<48;id++){const q=cp(id);g.drawImage(lampG,q.x-55,q.y-55)}g.restore()}
cars.forEach(c=>lights(c,S.night));signals(S.night);pin();
sensors();
g.restore();
$('sp').textContent=Math.round(e.v*.6);$('fr').textContent=e.front>=130?'–':Math.round(e.front);
$('sg').textContent=e.st;$('sg').style.color=LC[e.st]||'';$('hit').textContent=S.hit;$('off').textContent=S.off;$('tr').textContent=cars[0].dest||S.trip==='arrived'?S.trip:'–';$('nc').textContent=cars.length;$('np').textContent=S.ped.length}
let last=0;function loop(ts){const dt=Math.min(.05,(ts-last)/1000||0);last=ts;vdt=dt;step(dt);draw();requestAnimationFrame(loop)}
$('auto').onclick=ev=>{S.auto=!S.auto;ev.target.textContent=S.auto?'Autopilot: on':'Manual: arrow keys';if(S.auto){const e=cars[0];let b=0,m=1e9;e.path.forEach((p,i)=>{const d=dd(p,e);if(d<m){m=d;b=i}});e.pi=b}};
document.querySelectorAll('#sensors input').forEach(i=>{S.sn[i.dataset.k]=i.checked?1:0;i.onchange=()=>{S.sn[i.dataset.k]=i.checked?1:0}});
document.querySelectorAll('#camMode input').forEach(i=>{S.cam=i.value;i.onchange=()=>{if(i.checked)S.cam=i.value}});
$('traf').onclick=()=>addCars(5);$('ped').onclick=()=>addPeople(10);
$('clr').onclick=()=>{cars.length=1;S.ped=[]};$('rst').onclick=reset;
$('spd').oninput=ev=>{S.max=+ev.target.value;$('spv').textContent=S.max;cars[0].max=S.max};
cv.onclick=ev=>{const r=cv.getBoundingClientRect(),p=camInv((ev.clientX-r.left)*900/r.width,(ev.clientY-r.top)*560/r.height);
if($('click').value==='light'){for(let ni=0;ni<12;ni++)if(dd(np(ni),p)<44){sh[ni]+=10.5;break}return}
let bd=1e9,be=null;for(let ni=0;ni<12;ni++)for(const d of[0,1]){const m=nb(ni,d);if(m<0)continue;const A=np(ni),B=np(m),len=dd(A,B),t=clamp(((p.x-A.x)*(B.x-A.x)+(p.y-A.y)*(B.y-A.y))/(len*len),0,1),q={x:A.x+(B.x-A.x)*t,y:A.y+(B.y-A.y)*t},dist=dd(p,q);if(dist<bd){bd=dist;be={a:ni,b:m,s:t*len}}}
setDest(cars[0],be.a,be.b,be.s)};
$('cancel').onclick=()=>{const e=cars[0];e.dest=null;e.route=[];if(e.final)resume(e);S.trip='–'};
addEventListener('keydown',ev=>{if(ev.key.startsWith('Arrow')){S.keys[ev.key]=1;if(!S.auto)ev.preventDefault()}});
addEventListener('keyup',ev=>{delete S.keys[ev.key]});
$('night').onclick=ev=>{S.night=!S.night;ev.target.textContent='Night: '+(S.night?'on':'off')};
reset();addCars(8);addPeople(16);requestAnimationFrame(loop);
})();
