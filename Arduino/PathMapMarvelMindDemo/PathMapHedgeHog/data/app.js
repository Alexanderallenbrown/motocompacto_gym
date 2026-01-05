const c=document.getElementById("plot"),x=c.getContext("2d");
let m=[],r={x:0,y:0,yaw:0,preview:1.5},s=20;

// // Load default map from ESP32 on startup
// fetch("/map")
// .then(r => r.text())
// .then(t => {
//     m = t.trim().split("\n")
//               .slice(1)
//               .map(l=>{
//                   let [p,q] = l.split(",");
//                   return {x:+p, y:+q};
//               });
//     d();
// });

// preview slider
preview.oninput=e=>{
    r.preview=+e.target.value;
    fetch("/preview?d="+r.preview);
};

// gas slider
gas.oninput=e=>{
    fetch("/servo?gas="+e.target.value);
};

// steer slider
steer.oninput=e=>{
    fetch("/servo?steer="+e.target.value);
};

// upload CSV
csvInput.onchange=e=>{
 const f=e.target.files[0];
 const fd=new FormData();
 fd.append("file",f);
 fetch("/upload",{method:"POST",body:fd});
 const rd=new FileReader();
 rd.onload=()=>{m=rd.result.trim().split("\n").slice(1).map(l=>{let[p,q]=l.split(",");return{x:+p,y:+q}});d();};
 rd.readAsText(f);
};

// poll pose + errors
function poll(){
 fetch("/data")
 .then(j=>j.json())
 .then(o=>{
    r.x=o.x;
    r.y=o.y;
    r.yaw=o.yaw;
    station.innerText=o.station.toFixed(2);
    lateral.innerText=o.ePrev.toFixed(2);
    d();
 });
}
setInterval(poll,100);

function d(){
 x.clearRect(0,0,600,600);x.save();x.translate(300,300);x.scale(s,-s);
 x.beginPath();m.forEach((p,i)=>i?x.lineTo(p.x,p.y):x.moveTo(p.x,p.y));x.stroke();
 x.fillStyle="red";x.beginPath();x.arc(r.x,r.y,.2,0,6.28);x.fill();
 let px=r.x+r.preview*Math.cos(r.yaw),py=r.y+r.preview*Math.sin(r.yaw);
 x.fillStyle="blue";x.beginPath();x.arc(px,py,.15,0,6.28);x.fill();
 x.restore();
}
