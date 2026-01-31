import subprocess
from flask import Flask, Response
from stick_to_line import LineFollower

app = Flask(__name__)

# settings
width = 640
height = 480
fps = 20
port = 9130

enable_processing = True

# line follower settings
line_follower = LineFollower(
    width=width,
    height=height,
    linePos_1=380,
    linePos_2=430,
    invert_for_black_line=True,
    erode_iterations=6,
    jpeg_quality=80
)


def start_camera():
    cmd = [
        "/usr/bin/rpicam-vid",
        "-t", "0",
        "--inline",
        "--codec", "mjpeg",
        "--width", str(width),
        "--height", str(height),
        "--framerate", str(fps),
        "-o", "-"
    ]

    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=0
    )


def read_jpeg_frames(process):
    buffer = b""

    while True:
        chunk = process.stdout.read(4096)
        if not chunk:
            break

        buffer += chunk

        a = buffer.find(b"\xff\xd8")  # jpeg start
        b = buffer.find(b"\xff\xd9")  # jpeg end

        if a != -1 and b != -1 and b > a:
            jpg = buffer[a:b + 2]
            buffer = buffer[b + 2:]
            yield jpg


def generate_frames(processed=True):
    process = start_camera()

    try:
        for jpg in read_jpeg_frames(process):
            out_jpg = jpg

            if processed and enable_processing:
                try:
                    out_jpg, _ = line_follower.process_jpeg(jpg)
                except Exception as e:
                    print("processing error:", e)
                    out_jpg = jpg

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + out_jpg + b"\r\n"
            )

    finally:
        try:
            process.terminate()
            process.kill()
        except:
            pass


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(processed=True),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/video_feed_raw")
def video_feed_raw():
    return Response(
        generate_frames(processed=False),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/")
def index():
    return """
    <html>
      <body style="background:#000; display:flex; flex-direction:column; align-items:center; justify-content:center; height:100vh; margin:0;">
        <div style="margin-bottom:10px; font-family:monospace;">
          <a href="/video_feed" style="color:#9f9; margin-right:15px;">processed</a>
          <a href="/video_feed_raw" style="color:#aaa;">raw</a>
        </div>

        <img src="/video_feed" style="max-width:100%; border:2px solid #333;">
      </body>
    </html>
    """


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=port, threaded=True)