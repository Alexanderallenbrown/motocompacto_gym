
  let textDataArray = []; // Array to store text field values
  let JTLogging = false;
  let lastTime = Date.now();
  let socket;
  let mapRunning = false;
  let mapScale = 20;      // pixels per map unit
  let posX = 0;
  let posY = 0;
  let yaw = 0;





const MAX_POINTS = 500;   // scrolling window width (~5 seconds @100Hz)

let rollBuf = new Array(MAX_POINTS).fill(0);
let goalBuf = new Array(MAX_POINTS).fill(0);
let timeBuf = new Array(MAX_POINTS).fill(0);

let writeIndex = 0;
let sampleCount = 0;




let plotcanvas, ctx, mapCanvas, mapCtx;
let WIDTH,HEIGHT;
let MAP_W,MAP_H;

// your loaded map:
let m = [];  // filled by loadMap()

window.onload = () => {
  mapCanvas = document.getElementById("mapCanvas");
  mapCtx = mapCanvas.getContext("2d");
  plotcanvas = document.getElementById("plot");
  ctx = plotcanvas.getContext("2d");
  MAP_W = mapCanvas.width;
  MAP_H = mapCanvas.height;
  WIDTH = plotcanvas.width;
  HEIGHT = plotcanvas.height;
    connectWebSocket();   // start data stream
    drawPlot();           // start render loop
};

// pushes data to buffered arrays
function pushSample(roll, goal) {
    const now = performance.now();

    rollBuf[writeIndex] = roll;
    goalBuf[writeIndex] = goal;
    timeBuf[writeIndex] = now;

    writeIndex = (writeIndex + 1) % MAX_POINTS;
    sampleCount = Math.min(sampleCount + 1, MAX_POINTS);
}

//plot renderer
function drawPlot() {
  ctx.fillStyle = "#0e0e0e";
  ctx.fillRect(0, 0, WIDTH, HEIGHT);

    // draw grid
    drawGrid();
    drawYAxisTicks();

    // find value range
    let min = -20;
    let max = 20;

    drawLine(rollBuf,  "#00c8ff", min, max);
    drawLine(goalBuf, "#ffb000", min, max);

    requestAnimationFrame(drawPlot);
}

function drawYAxisTicks() {
    ctx.fillStyle = "#888";
    ctx.font = "12px Arial";
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";

    const min = -20;
    const max = 20;
    const numTicks = 5;  // -45, -22.5, 0, 22.5, 45

    for (let i = 0; i <= numTicks; i++) {
        const val = min + (i / numTicks) * (max - min);
        const y = HEIGHT - ((val - min) / (max - min)) * HEIGHT;
        ctx.fillText(val.toFixed(0), 35, y);  // add 35px padding for y-axis
        ctx.beginPath();
        ctx.strokeStyle = "#444";
        ctx.moveTo(40, y);
        ctx.lineTo(WIDTH, y);
        ctx.stroke();
    }
}

//grid draw Function
function drawGrid() {
    ctx.strokeStyle = "#222";
    ctx.lineWidth = 1;

    // horizontal lines
    for (let i = 0; i <= 6; i++) {
        let y = i * HEIGHT / 6;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(WIDTH, y);
        ctx.stroke();
    }

    // vertical lines
    for (let i = 0; i <= 10; i++) {
        let x = i * WIDTH / 10;
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, HEIGHT);
        ctx.stroke();
    }
}

//scrolling line draw
function drawLine(buffer, color, minVal, maxVal) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();

    for (let i = 0; i < sampleCount; i++) {
        let index = (writeIndex + i) % MAX_POINTS;

        let v = buffer[index];

        let x = i * WIDTH / (MAX_POINTS - 1);
        let y = HEIGHT - ((v - minVal) / (maxVal - minVal)) * HEIGHT;

        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }

    ctx.stroke();
}

//map drawing functions:
function worldToScreen(wx, wy) {
    const sx = MAP_W / 2 + (wx - posX) * mapScale;
    const sy = MAP_H / 2 - (wy - posY) * mapScale;
    return {x: sx, y: sy};
}

function drawMap() {

    // background
    mapCtx.fillStyle = "#0e0e0e";
    mapCtx.fillRect(0, 0, MAP_W, MAP_H);

    drawMapGrid();
    drawMapPoints();
    drawVehicle();

    requestAnimationFrame(drawMap);
}

function drawMapGrid() {
    mapCtx.strokeStyle = "#1f1f1f";
    mapCtx.lineWidth = 1;

    const step = mapScale;

    const ox = MAP_W / 2;
    const oy = MAP_H / 2;

    for (let x = ox % step; x < MAP_W; x += step) {
        mapCtx.beginPath();
        mapCtx.moveTo(x, 0);
        mapCtx.lineTo(x, MAP_H);
        mapCtx.stroke();
    }

    for (let y = oy % step; y < MAP_H; y += step) {
        mapCtx.beginPath();
        mapCtx.moveTo(0, y);
        mapCtx.lineTo(MAP_W, y);
        mapCtx.stroke();
    }
}

function drawMapPoints() {
    mapCtx.fillStyle = "#00b7ff";

    for (let p of m) {
        const s = worldToScreen(p.x, p.y);

        mapCtx.beginPath();
        mapCtx.arc(s.x, s.y, 3, 0, Math.PI * 2);
        mapCtx.fill();
    }
}

function drawVehicle() {
    const s = worldToScreen(posX, posY);

    // body
    mapCtx.fillStyle = "#ffffff";
    mapCtx.beginPath();
    mapCtx.arc(s.x, s.y, 5, 0, Math.PI * 2);
    mapCtx.fill();

    // yaw arrow
    const len = 28;
    const ex = s.x + Math.cos(yaw) * len;
    const ey = s.y - Math.sin(yaw) * len;

    mapCtx.strokeStyle = "#ff4444";
    mapCtx.lineWidth = 2;

    mapCtx.beginPath();
    mapCtx.moveTo(s.x, s.y);
    mapCtx.lineTo(ex, ey);
    mapCtx.stroke();

    drawArrowHead(ex, ey, yaw);
}

function drawArrowHead(x, y, a) {
    const size = 7;

    mapCtx.fillStyle = "#ff4444";
    mapCtx.beginPath();
    mapCtx.moveTo(x, y);
    mapCtx.lineTo(x - Math.cos(a - 0.5) * size, y + Math.sin(a - 0.5) * size);
    mapCtx.lineTo(x - Math.cos(a + 0.5) * size, y + Math.sin(a + 0.5) * size);
    mapCtx.closePath();
    mapCtx.fill();
}






  function connectWebSocket() {
    socket = new WebSocket(`ws://${location.host}/ws`);

    socket.onopen = () => {
        console.log("WebSocket connected");
    };

    socket.onclose = () => {
        console.log("WebSocket disconnected");
        setTimeout(connectWebSocket, 1000); // auto-reconnect
    };

    socket.onmessage = (event) => {
        const data = JSON.parse(event.data);

        if (JTLogging) {
            logJTData(data);
        }
        posX = data.x;
        posY = data.y;
        yaw = data.yaw;
        // optional: update UI here
        pushSample(data.rollFinal,data.goalRoll_filt);
    };
}

function logJTData(d) {
    const timestamp = Date.now();

    const row =
        timestamp + "," +
        d.x + "," +
        d.y + "," +
        d.yaw + "," +
        d.station + "," +
        d.ePrev + "," +
        d.rollFinal + "," +
        d.goalRoll_filt + "," +
        d.rollRate + "," +
        d.steer + "\r\n";

    textDataArray.push(row);
}


function loadMap(){
  if (mapRunning) return;
    mapRunning = true;
  console.log("attempting to load map...")
  fetch("/map")
  .then(r => r.text())
  .then(t => {
      m = t.trim().split("\n")
                .slice(1)
                .map(l=>{
                    let [p,q] = l.split(",");
                    return {x:+p, y:+q};
                });
      drawMap();
      console.log(m)
  });
}

  function toggleGoState(){
    fetch('/toggle_go')
    .then(response => response.json())
    .then(data => {
      document.getElementById('goButton').innerText = data.buttonText;
    });
  }

  function zeroRollAngle(){
    fetch('/zero_roll').then(response => response.json()).then(data => {document.getElementById('zeroRollButton').innerText = data.buttonText;});
  }

  function zeroYaw(){
    fetch('/zero_yaw');//.then(response => response.json()).then(data => {document.getElementById('zeroYawButton').innerText = data.buttonText;});
  }

  function pollData(){
    fetch("/data")
    .then(data => data.json())


  }

  // Function to download the text data as a file
  function downloadDataFile() {
    if(JTLogging){

      let pref = "pathFollowing"//document.getElementById('prefixField').value;
      let comment = document.getElementById('commentField').value;
      textDataArray.unshift('#'+comment);

      const blob = new Blob([textDataArray.join('\n')], { type: 'text/plain' });
      const url = window.URL.createObjectURL(blob);
      const a = document.createElement('a');

      a.href = url;
      const now = new Date();
      const year = now.getFullYear();
      const month = String(now.getMonth() + 1).padStart(2, '0'); // Months are 0-indexed
      const day = String(now.getDate()).padStart(2, '0');
      const hours = String(now.getHours()).padStart(2, '0');
      const minutes = String(now.getMinutes()).padStart(2, '0');
      const seconds = String(now.getSeconds()).padStart(2, '0');

      const dateString = `${year}_${month}_${day}_${hours}_${minutes}_${seconds}`;
      // Using toString()
      a.download = dateString+'_'+pref+'_'+'compactoData.txt'; // File name
      a.click();
      window.URL.revokeObjectURL(url);
    }
      JTLogging = !JTLogging;
    }



    //        setInterval(fetchSpeedField,50);
