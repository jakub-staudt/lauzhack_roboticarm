const {test}=require('node:test');
const assert=require('node:assert/strict');
const G=require('../apps/arm-dashboard/src/arm_dashboard/static/board-map.js');
function near(a,b){assert.ok(Math.abs(a-b)<1e-6,`${a} != ${b}`)}
test('projective reference recovers millimetres inside and outside reference',()=>{
 const worldToImage=[.5,.1,130,.02,.45,70,.0003,.0002,1];
 const corners=[[0,0],[500,0],[500,300],[0,300]].map(p=>G.projectPoint(worldToImage,p));
 const h=G.solveHomography(corners,500,300);
 for(const world of [[0,0],[500,300],[210,120],[-100,80],[700,400]]){
  const actual=G.projectPoint(h,G.projectPoint(worldToImage,world));near(actual[0],world[0]);near(actual[1],world[1]);
 }
 const inv=G.invertHomography(h),pixel=[250,180];const roundtrip=G.projectPoint(inv,G.projectPoint(h,pixel));near(roundtrip[0],pixel[0]);near(roundtrip[1],pixel[1]);
});
test('rejects crossed, collinear, tiny and dimensionless references',()=>{
 for(const points of [[[0,0],[100,100],[100,0],[0,100]],[[0,0],[10,0],[20,0],[30,0]],[[0,0],[1,0],[1,1],[0,1]]])assert.throws(()=>G.solveHomography(points,100,100));
 assert.throws(()=>G.solveHomography([[0,0],[100,0],[100,100],[0,100]],0,100));
});
test('whole visible plane bounds extend beyond calibration rectangle',()=>{
 const h=G.solveHomography([[100,100],[200,100],[200,200],[100,200]],100,100);
 const bounds=G.boardBounds(h,640,480);
 near(bounds.minX,-100);near(bounds.maxX,539);near(bounds.minY,-100);near(bounds.maxY,379);
});
test('rejects a projective horizon crossing the image',()=>{
 assert.throws(()=>G.boardBounds([1,0,0,0,1,0,1,0,-100],640,480));
});
