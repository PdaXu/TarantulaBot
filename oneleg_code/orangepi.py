from flask import Flask, render_template_string, jsonify
import serial
import threading

app = Flask(__name__)
esp = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>TarantulaBot控制台</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { background:#1a1a1a; color:white; font-family:Arial; text-align:center; padding:20px; }
        h1 { color:#27AE60; }
        .btn { 
            display:inline-block; margin:10px; padding:20px 40px;
            font-size:18px; border:none; border-radius:10px; cursor:pointer;
        }
        .green { background:#27AE60; color:white; }
        .red { background:#C0392B; color:white; }
        .blue { background:#2E6DA4; color:white; }
        #log { 
            margin-top:20px; padding:10px; background:#333;
            border-radius:10px; height:200px; overflow-y:auto; text-align:left;
        }
    </style>
</head>
<body>
    <h1>🕷 TarantulaBot</h1>
    <div>
        <button class="btn green" onclick="send('STEP')">迈步</button>
        <button class="btn green" onclick="send('MAGNET_ON')">磁铁开</button>
        <button class="btn red" onclick="send('MAGNET_OFF')">磁铁关</button>
    </div>
    <div id="log"></div>
    <script>
        function send(cmd) {
            fetch('/cmd/' + cmd)
            .then(r => r.json())
            .then(d => {
                var log = document.getElementById('log');
                log.innerHTML = '<p>' + new Date().toLocaleTimeString() + ' → ' + cmd + '</p>' + log.innerHTML;
            });
        }
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/cmd/<command>')
def send_cmd(command):
    esp.write((command + '\n').encode())
    return jsonify({'status': 'ok', 'cmd': command})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)