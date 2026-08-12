from flask import Flask, render_template_string, jsonify, Response
import serial
import cv2
import cv2.aruco as aruco
import threading
import time

app = Flask(__name__)

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD = 115200
esp = None
serial_lock = threading.Lock()
serial_connected = False
last_reconnect_attempt = 0
RECONNECT_INTERVAL = 2.0   # 秒, 断线后多久重试一次

def open_serial():
    """尝试打开串口, 成功返回True。失败不抛异常, 只记录状态。"""
    global esp, serial_connected
    try:
        if esp is not None:
            try: esp.close()
            except Exception: pass
        esp = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
        serial_connected = True
        print(f"[串口] 已连接 {SERIAL_PORT}")
        return True
    except Exception as e:
        serial_connected = False
        print(f"[串口] 连接失败: {e}")
        return False

def ensure_serial():
    """确保串口可用, 断线时按节流频率自动重试, 不阻塞太久"""
    global last_reconnect_attempt
    if serial_connected:
        return True
    now = time.time()
    if now - last_reconnect_attempt < RECONNECT_INTERVAL:
        return False
    last_reconnect_attempt = now
    return open_serial()

open_serial()   # 启动时先尝试一次

camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
camera.set(cv2.CAP_PROP_FPS, 15)

# ===== ArUco 追踪配置 =====
TARGET_ID       = 0
CENTER_DEADZONE = 0.15
STOP_SIZE_RATIO = 0.85
CMD_INTERVAL    = 0.5
LOST_TIMEOUT    = 1.0

DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(DICT, aruco.DetectorParameters())

# ===== 共享状态 =====
auto_mode = False
auto_status = "手动"
gait = "wave"
frame_lock = threading.Lock()
latest_frame = None

last_cmd = None
last_cmd_time = 0
last_seen_time = 0

GAIT_MAP = {
    "wave": {"FWD":"BACK","BACK":"FWD","LEFT":"LEFT","RIGHT":"RIGHT",
             "STRAFE_LEFT":"STRAFE_LEFT","STRAFE_RIGHT":"STRAFE_RIGHT"},
    "trot": {"FWD":"TROT","BACK":"TBACK","LEFT":"TLEFT","RIGHT":"TRIGHT",
             "STRAFE_LEFT":"TSTRAFE_LEFT","STRAFE_RIGHT":"TSTRAFE_RIGHT"},
}

def raw_send(s):
    """线程安全的底层串口写(ADJ高频指令与普通指令共用)
       断线时自动尝试重连; 写入失败不抛异常, 只标记断线, 由下次调用触发重连"""
    global serial_connected
    if not ensure_serial():
        return False   # 仍未连上, 静默放弃这条指令(避免刷屏报错)
    with serial_lock:
        try:
            esp.write((s + '\n').encode())
            return True
        except Exception as e:
            serial_connected = False
            print(f"[串口] 写入失败, 标记断线待重连: {e}")
            return False

def send(cmd, force=False):
    global last_cmd, last_cmd_time
    real = GAIT_MAP[gait].get(cmd, cmd)
    now = time.time()
    if not force and real == last_cmd and now - last_cmd_time < CMD_INTERVAL:
        return True   # 去重跳过, 不算失败
    ok = raw_send(real)
    last_cmd = real
    last_cmd_time = now
    return ok

# ===== 姿态平衡控制器 (pid_ctrl.py) =====
balance_ctrl = None
balance_err = ""
try:
    from pid_ctrl import AttitudeController
    balance_ctrl = AttitudeController(raw_send, kp=1.5, ki=0.0, kd=0.08)
except Exception as e:
    balance_err = str(e)   # IMU没接好等情况, 网页按钮会提示

def camera_loop():
    global latest_frame, auto_status, last_seen_time
    while True:
        ok, frame = camera.read()
        if not ok:
            time.sleep(0.05)
            continue
        h, w = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        if auto_mode:
            decision = "SEARCH"
            found = False
            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    if mid == TARGET_ID:
                        found = True
                        c = corners[i][0]
                        cx = c[:, 0].mean(); cy = c[:, 1].mean()
                        size = max(c[:,0].max()-c[:,0].min(),
                                   c[:,1].max()-c[:,1].min())
                        offset = (cx - w/2) / (w/2)
                        size_ratio = size / w

                        if size_ratio > STOP_SIZE_RATIO:
                            decision = "ARRIVED"; send("STAND")
                        elif offset < -CENTER_DEADZONE:
                            decision = "LEFT"; send("LEFT")
                        elif offset > CENTER_DEADZONE:
                            decision = "RIGHT"; send("RIGHT")
                        else:
                            decision = "FWD"; send("FWD")

                        cv2.drawMarker(frame,(int(cx),int(cy)),(0,0,255),
                                       cv2.MARKER_CROSS,24,2)
                        last_seen_time = time.time()
                        break
            if not found and time.time()-last_seen_time > LOST_TIMEOUT:
                decision = "LOST"; send("STAND")

            auto_status = decision
            cv2.putText(frame, "AUTO: "+decision, (10,34),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
        else:
            cv2.putText(frame, "MANUAL / "+gait.upper(), (10,34),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200,200,200), 2)

        # 平衡状态叠加
        if balance_ctrl and balance_ctrl.running:
            s = balance_ctrl.status
            cv2.putText(frame, f"BAL r={s['roll']:+.1f} p={s['pitch']:+.1f}",
                        (10, h-14), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,255), 2)

        with frame_lock:
            latest_frame = frame
        time.sleep(0.03)

threading.Thread(target=camera_loop, daemon=True).start()

def generate_frames():
    while True:
        with frame_lock:
            f = latest_frame
        if f is None:
            time.sleep(0.05); continue
        _, buf = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 60])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
               + buf.tobytes() + b'\r\n')
        time.sleep(0.05)

HTML = '''
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>TarantulaBot</title>
    <style>
        * { box-sizing:border-box; }
        html, body { height:100%; overflow:hidden; }
        body { background:#14161a; color:#e8e8e8; font-family:'Segoe UI',Arial,sans-serif;
               margin:0; display:flex; flex-direction:column; }

        /* ---------- Header ---------- */
        .header { display:flex; align-items:center; justify-content:center; gap:14px;
                   padding:10px 16px; background:linear-gradient(180deg,#1c1f26,#161820);
                   border-bottom:1px solid #2a2d35; flex-shrink:0; }
        h1 { color:#ff7a1a; font-size:20px; margin:0; letter-spacing:0.5px;
             text-shadow:0 0 12px rgba(255,122,26,0.35); }
        #serial-status { font-size:12px; padding:4px 12px; border-radius:14px; background:#555;
                          font-weight:600; transition:background 0.3s; }
        #lang-btn { background:#30343d; padding:6px 14px; font-size:13px; border-radius:8px; }

        /* ---------- Main grid ---------- */
        .layout { flex:1; display:grid; grid-template-columns:200px 1fr 250px;
                  gap:12px; padding:12px; min-height:0; }

        .panel { background:#1c1f26; border-radius:12px; padding:10px;
                 border:1px solid #262a33; box-shadow:0 2px 10px rgba(0,0,0,0.25);
                 display:flex; flex-direction:column; min-height:0; }
        .panel h3 { color:#9aa0ac; margin:2px 0 8px; text-align:center; font-size:13px;
                    text-transform:uppercase; letter-spacing:1px; font-weight:600; }

        /* ---------- Log ---------- */
        #log-panel { min-height:0; }
        #log { flex:1; overflow-y:auto; font-size:12px; font-family:'Consolas',monospace;
               color:#8fe38f; }
        #log p { margin:0; padding:5px 4px; border-bottom:1px solid #23262e; line-height:1.4; }
        #log::-webkit-scrollbar { width:5px; }
        #log::-webkit-scrollbar-thumb { background:#3a3e48; border-radius:3px; }

        /* ---------- Center ---------- */
        #center-panel { align-items:center; justify-content:flex-start; gap:8px; }
        #video-container { width:100%; text-align:center; }
        #video-container img { border-radius:10px; border:1px solid #333; width:100%;
                                max-height:55vh; object-fit:cover; display:block; margin:0 auto; }

        .dpad { display:grid; grid-template-columns:repeat(3, 74px); grid-gap:6px;
                justify-content:center; margin-top:4px; }
        .btn { padding:10px 0; border:none; border-radius:8px; font-size:14px; font-weight:600;
               cursor:pointer; color:#fff; transition:transform 0.08s, filter 0.15s; }
        .btn:hover { filter:brightness(1.12); }
        .btn:active { transform:scale(0.95); }
        .move  { background:#27ae60; }
        .stop  { background:#e74c3c; }
        .toolbar { display:flex; gap:6px; justify-content:center; margin-top:6px; flex-wrap:wrap; }
        #gait-btn { width:104px; background:#3498db; font-size:12px; padding:8px 0; }
        #auto-btn { width:150px; background:#f39c12; font-size:12px; padding:8px 0; }
        #auto-btn.on { background:#e74c3c; }
        #bal-btn { width:110px; background:#16a085; font-size:12px; padding:8px 0; }
        #bal-btn.on { background:#e74c3c; }

        /* ---------- Right panel ---------- */
        #right-panel { overflow-y:auto; gap:8px; padding:8px; }
        #right-panel::-webkit-scrollbar { width:5px; }
        #right-panel::-webkit-scrollbar-thumb { background:#3a3e48; border-radius:3px; }
        .card { background:#20242c; border-radius:10px; padding:8px 10px;
                border:1px solid #2a2e37; flex-shrink:0; }
        .card h3 { color:#9aa0ac; margin:0 0 7px; font-size:12px; text-align:center;
                   text-transform:uppercase; letter-spacing:0.8px; }
        .card .btn { width:47%; margin:1.5%; padding:8px 0; font-size:13px; }
        .blue{background:#3498db;} .purple{background:#9b59b6;}
        .green{background:#27ae60;} .red{background:#e74c3c;} .gray{background:#6b7280;}

        #magtest-grid { display:grid; grid-template-columns:1fr 1fr; gap:5px; }
        #magtest-grid .btn { width:100%; margin:0; padding:7px 0; font-size:12.5px; }

        #att-display { text-align:center; font-size:14px; color:#2ecffb; font-weight:600;
                        padding:4px 0; }

        @media (max-height:700px){
            #video-container img { max-height:42vh; }
            .btn { padding:7px 0; font-size:13px; }
        }
        @media (max-width:900px){
            .layout { grid-template-columns:1fr; grid-auto-rows:auto; overflow-y:auto; }
            html, body { overflow:auto; }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🕷 TarantulaBot</h1>
        <span id="serial-status">●串口检测中</span>
        <button id="lang-btn" class="btn" onclick="toggleLang()">EN</button>
    </div>
    <div class="layout">
        <div id="log-panel" class="panel">
            <h3 data-i18n="log">日志</h3>
            <div id="log"></div>
        </div>
        <div id="center-panel" class="panel">
            <div id="video-container"><img src="/video_feed"></div>
            <div class="dpad">
                <div></div><button class="btn move" data-i18n="fwd" onclick="send('FWD')">前进</button><div></div>
                <button class="btn move" data-i18n="left" onclick="send('LEFT')">原地左转</button>
                <button class="btn stop" data-i18n="stop" onclick="send('STOP')">停止</button>
                <button class="btn move" data-i18n="right" onclick="send('RIGHT')">原地右转</button>
                <div></div><button class="btn move" data-i18n="back" onclick="send('BACK')">后退</button><div></div>
            </div>
            <div class="dpad" style="grid-template-columns:repeat(2, 74px);">
                <button class="btn move" data-i18n="strafeL" onclick="send('STRAFE_LEFT')">⇤ 左移</button>
                <button class="btn move" data-i18n="strafeR" onclick="send('STRAFE_RIGHT')">右移 ⇥</button>
            </div>
            <div class="toolbar">
                <button id="gait-btn" class="btn" onclick="toggleGait()">步态: Wave</button>
                <button id="bal-btn" class="btn" onclick="toggleBalance()">⚖️ 平衡: 关</button>
                <button id="auto-btn" class="btn" onclick="toggleAuto()">🎯 自动追踪: 关</button>
            </div>
        </div>
        <div id="right-panel" class="panel">
            <div class="card">
                <h3 data-i18n="arm">机械臂</h3>
                <button class="btn purple" data-i18n="extend" onclick="send('ACT_EXTEND')">伸</button>
                <button class="btn purple" data-i18n="retract" onclick="send('ACT_RETRACT')">缩</button>
                <button class="btn gray" data-i18n="actstop" onclick="send('ACT_STOP')">推杆停</button>
                <button class="btn blue" data-i18n="suck" onclick="send('PUMP_ON')">提取</button>
                <button class="btn gray" data-i18n="release" onclick="send('PUMP_OFF')">放</button>
            </div>
            <div class="card">
                <h3 data-i18n="magnet">电磁铁</h3>
                <button class="btn green" data-i18n="magon" onclick="send('MAGNET_ON')">吸附</button>
                <button class="btn red" data-i18n="magoff" onclick="send('MAGNET_OFF')">释放</button>
            </div>
            <div class="card">
                <h3 data-i18n="magtest">单腿磁铁测试</h3>
                <div id="magtest-grid">
                    <button class="btn green" onclick="send('MAG_LF_ON')">LF吸</button>
                    <button class="btn red"   onclick="send('MAG_LF_OFF')">LF松</button>
                    <button class="btn green" onclick="send('MAG_RF_ON')">RF吸</button>
                    <button class="btn red"   onclick="send('MAG_RF_OFF')">RF松</button>
                    <button class="btn green" onclick="send('MAG_LB_ON')">LB吸</button>
                    <button class="btn red"   onclick="send('MAG_LB_OFF')">LB松</button>
                    <button class="btn green" onclick="send('MAG_RB_ON')">RB吸</button>
                    <button class="btn red"   onclick="send('MAG_RB_OFF')">RB松</button>
                </div>
            </div>
            <div class="card">
                <h3 data-i18n="att">姿态</h3>
                <div id="att-display">--</div>
            </div>
        </div>
    </div>
    <script>
        let autoOn = false, balOn = false, gaitNow = 'wave', lang = 'zh';

        const I18N = {
          zh: {log:'日志', fwd:'前进', back:'后退', left:'原地左转', right:'原地右转', stop:'停止',
               strafeL:'⇤ 左移', strafeR:'右移 ⇥',
               arm:'机械臂', extend:'伸', retract:'缩', actstop:'推杆停', suck:'提取', release:'放',
               magnet:'电磁铁', magon:'吸附', magoff:'释放', magtest:'单腿磁铁测试', att:'姿态',
               gait:'步态', auto:'自动追踪', bal:'平衡', on:'开', off:'关',
               autoBlock:'自动模式中, 方向键无效 (先关自动)',
               autoIn:'进入自动追踪模式', autoOut:'退出自动, 已停止',
               balIn:'平衡开启 (校准中保持静止)', balOut:'平衡关闭, 偏移清零',
               balFail:'平衡不可用: ', gaitTo:'步态切换为 '},
          en: {log:'Log', fwd:'FWD', back:'BACK', left:'Turn Left', right:'Turn Right', stop:'STOP',
               strafeL:'⇤ Strafe L', strafeR:'Strafe R ⇥',
               arm:'Arm', extend:'Extend', retract:'Retract', actstop:'Act Stop', suck:'Suck', release:'Release',
               magnet:'Magnet', magon:'Attach', magoff:'Detach', magtest:'Per-Leg Magnet Test', att:'Attitude',
               gait:'Gait', auto:'Auto Track', bal:'Balance', on:'ON', off:'OFF',
               autoBlock:'Auto mode active, D-pad disabled',
               autoIn:'Auto tracking ON', autoOut:'Auto OFF, robot stopped',
               balIn:'Balance ON (calibrating, keep still)', balOut:'Balance OFF, offsets cleared',
               balFail:'Balance unavailable: ', gaitTo:'Gait switched to '}
        };
        function t(k){ return I18N[lang][k]; }

        function applyLang(){
            document.querySelectorAll('[data-i18n]').forEach(el=>{
                el.textContent = t(el.getAttribute('data-i18n'));
            });
            document.getElementById('lang-btn').textContent = (lang==='zh') ? 'EN' : '中';
            refreshGaitBtn(); refreshAutoBtn(); refreshBalBtn();
        }
        function toggleLang(){ lang = (lang==='zh')?'en':'zh'; applyLang(); }

        function refreshGaitBtn(){
            document.getElementById('gait-btn').textContent =
                t('gait')+': '+(gaitNow==='wave'?'Wave':'Trot');
        }
        function refreshAutoBtn(st){
            var b=document.getElementById('auto-btn');
            b.textContent = '🎯 '+t('auto')+': '+(autoOn?t('on'):t('off'))+(st?' ['+st+']':'');
            b.classList.toggle('on', autoOn);
        }
        function refreshBalBtn(){
            var b=document.getElementById('bal-btn');
            b.textContent = '⚖️ '+t('bal')+': '+(balOn?t('on'):t('off'));
            b.classList.toggle('on', balOn);
        }

        function log(msg){
            var l=document.getElementById('log');
            l.innerHTML='<p>'+new Date().toLocaleTimeString()+' → '+msg+'</p>'+l.innerHTML;
        }
        function send(cmd){
            if(autoOn && ['FWD','BACK','LEFT','RIGHT','STRAFE_LEFT','STRAFE_RIGHT'].includes(cmd)){
                log(t('autoBlock')); return;
            }
            fetch('/cmd/'+cmd).then(r=>r.json()).then(d=>log(cmd));
        }
        function toggleGait(){
            fetch('/gait/toggle').then(r=>r.json()).then(d=>{
                gaitNow = d.gait; refreshGaitBtn();
                log(t('gaitTo')+gaitNow.toUpperCase());
            });
        }
        function toggleAuto(){
            fetch('/auto/toggle').then(r=>r.json()).then(d=>{
                autoOn = d.auto; refreshAutoBtn();
                log(autoOn ? t('autoIn') : t('autoOut'));
            });
        }
        function toggleBalance(){
            fetch('/balance/toggle').then(r=>r.json()).then(d=>{
                if(d.error){ log(t('balFail')+d.error); return; }
                balOn = d.balance; refreshBalBtn();
                log(balOn ? t('balIn') : t('balOut'));
            });
        }
        setInterval(()=>{
            if(autoOn){
                fetch('/auto/status').then(r=>r.json()).then(d=>refreshAutoBtn(d.status));
            }
            if(balOn){
                fetch('/balance/status').then(r=>r.json()).then(d=>{
                    document.getElementById('att-display').textContent =
                        'roll '+d.roll+'°  pitch '+d.pitch+'°';
                });
            }
        }, 1000);

        setInterval(()=>{
            fetch('/serial/status').then(r=>r.json()).then(d=>{
                var el = document.getElementById('serial-status');
                if (d.connected) {
                    el.textContent = '● 串口已连接';
                    el.style.background = '#2ecc71';
                } else {
                    el.textContent = '● 串口断开, 重连中...';
                    el.style.background = '#e74c3c';
                }
            });
        }, 1500);

        applyLang();
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cmd/<command>')
def send_cmd(command):
    if auto_mode and command in ('FWD','BACK','LEFT','RIGHT','STRAFE_LEFT','STRAFE_RIGHT'):
        return jsonify({'status':'blocked', 'reason':'auto mode'})
    ok = send(command, force=True)
    return jsonify({'status':'ok' if ok else 'serial_error', 'cmd':command,
                    'connected': serial_connected})

@app.route('/serial/status')
def serial_status():
    return jsonify({'connected': serial_connected})

@app.route('/gait/toggle')
def gait_toggle():
    global gait
    gait = 'trot' if gait == 'wave' else 'wave'
    send("STAND", force=True)
    return jsonify({'gait': gait})

@app.route('/auto/toggle')
def auto_toggle():
    global auto_mode
    auto_mode = not auto_mode
    if not auto_mode:
        send("STAND", force=True)
    return jsonify({'auto': auto_mode})

@app.route('/auto/status')
def auto_stat():
    return jsonify({'auto': auto_mode, 'status': auto_status})

@app.route('/balance/toggle')
def balance_toggle():
    if balance_ctrl is None:
        return jsonify({'error': balance_err or 'IMU init failed'})
    if balance_ctrl.running:
        balance_ctrl.stop()
    else:
        balance_ctrl.start()   # 含1.5s陀螺校准, 期间保持机器人静止
    return jsonify({'balance': balance_ctrl.running})

@app.route('/balance/status')
def balance_stat():
    if balance_ctrl is None or not balance_ctrl.running:
        return jsonify({'roll':0, 'pitch':0})
    s = balance_ctrl.status
    return jsonify({'roll': s['roll'], 'pitch': s['pitch']})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
