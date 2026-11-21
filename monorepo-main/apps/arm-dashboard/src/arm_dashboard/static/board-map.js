/* Plane geometry only: no robot connections or movement commands. */
(function(root){
'use strict';
function validateReference(points){
 if(points.length!==4||points.some(p=>p.length!==2||p.some(v=>!Number.isFinite(v))))throw Error('Select four valid corners.');
 const cross=points.map((p,i)=>{const q=points[(i+1)%4],r=points[(i+2)%4];return(q[0]-p[0])*(r[1]-q[1])-(q[1]-p[1])*(r[0]-q[0]);});
 if(cross.some(v=>Math.abs(v)<1e-6)||!cross.every(v=>Math.sign(v)===Math.sign(cross[0])))throw Error('Corners must form a convex rectangle outline in perimeter order.');
 const area=Math.abs(points.reduce((s,p,i)=>{const q=points[(i+1)%4];return s+p[0]*q[1]-q[0]*p[1]},0))/2;
 if(area<100)throw Error('Reference is too small; choose a larger visible rectangle.');
}
function solveHomography(points,width,height){
 validateReference(points);
 if(!Number.isFinite(width)||!Number.isFinite(height)||width<=0||height<=0)throw Error('Enter measured width and height in millimetres.');
 const dst=[[0,0],[width,0],[width,height],[0,height]],a=[];
 for(let i=0;i<4;i++){const[x,y]=points[i],[u,v]=dst[i];a.push([x,y,1,0,0,0,-u*x,-u*y,u],[0,0,0,x,y,1,-v*x,-v*y,v]);}
 for(let c=0;c<8;c++){
  let best=c;for(let r=c+1;r<8;r++)if(Math.abs(a[r][c])>Math.abs(a[best][c]))best=r;
  [a[c],a[best]]=[a[best],a[c]];const scale=a[c][c];if(Math.abs(scale)<1e-10)throw Error('Unstable reference geometry.');
  for(let k=c;k<=8;k++)a[c][k]/=scale;
  for(let r=0;r<8;r++){if(r===c)continue;const f=a[r][c];for(let k=c;k<=8;k++)a[r][k]-=f*a[c][k];}
 }
 return [...a.map(row=>row[8]),1];
}
function projectPoint(h,p){const[x,y]=p,d=h[6]*x+h[7]*y+h[8];if(Math.abs(d)<1e-9)throw Error('Plane horizon crosses the image.');return[(h[0]*x+h[1]*y+h[2])/d,(h[3]*x+h[4]*y+h[5])/d];}
function invertHomography(h){
 const[a,b,c,d,e,f,g,i,j]=h;
 const v=[e*j-f*i,c*i-b*j,b*f-c*e,f*g-d*j,a*j-c*g,c*d-a*f,d*i-e*g,b*g-a*i,a*e-b*d];
 const det=a*v[0]+b*v[3]+c*v[6];if(Math.abs(det)<1e-12)throw Error('Singular plane transform.');return v.map(x=>x/det);
}
function boardBounds(h,width,height){
 const corners=[[0,0],[width-1,0],[width-1,height-1],[0,height-1]];
 const signs=corners.map(([x,y])=>Math.sign(h[6]*x+h[7]*y+h[8]));
 if(signs.some(s=>!s||s!==signs[0]))throw Error('Plane horizon crosses the image; choose a better reference.');
 const xy=corners.map(p=>projectPoint(h,p)),xs=xy.map(p=>p[0]),ys=xy.map(p=>p[1]);
 const result={minX:Math.min(...xs),maxX:Math.max(...xs),minY:Math.min(...ys),maxY:Math.max(...ys)};
 if(Object.values(result).some(v=>!Number.isFinite(v))||Math.max(result.maxX-result.minX,result.maxY-result.minY)>20000)throw Error('Implausibly large mapped area; check the corner order and dimensions.');
 return result;
}
const api={validateReference,solveHomography,projectPoint,invertHomography,boardBounds};
if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.BoardGeometry=api;
})(typeof window!=='undefined'?window:globalThis);
