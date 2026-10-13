#!/usr/bin/env python3
"""3D attitude viewer + serial monitor for the drone.

  python tools/gyro3d.py [COM3]

Owns the serial port (close other monitors first), switches the firmware to
`plot att` (50 Hz), and draws a wireframe quad that follows roll/pitch. Yaw is
integrated from the gyro for display only (the firmware has no heading).
The lower pane is a normal log/command console: type `selftest`, `orient`...
Needs only pyserial (pip install pyserial) and tkinter (ships with Python).
"""
import math, sys, threading, queue, time, signal
import tkinter as tk
from tkinter import ttk
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = 115200

# ---------------- geometry (body frame: X forward, Y right, Z down) -------------
ARM = 1.0
BODY = [  # wireframe segments as pairs of points
    ((ARM, -ARM, 0), (-ARM, ARM, 0)),   # FL-RR arm (M1-M3)
    ((ARM, ARM, 0), (-ARM, -ARM, 0)),   # FR-RL arm (M2-M4)
    ((0.5, -0.4, 0), (0.5, 0.4, 0)), ((0.5, 0.4, 0), (-0.5, 0.4, 0)),   # body plate
    ((-0.5, 0.4, 0), (-0.5, -0.4, 0)), ((-0.5, -0.4, 0), (0.5, -0.4, 0)),
    ((0.5, -0.4, 0), (0.5, -0.4, -0.25)), ((0.5, 0.4, 0), (0.5, 0.4, -0.25)),  # plate height
    ((-0.5, 0.4, 0), (-0.5, 0.4, -0.25)), ((-0.5, -0.4, 0), (-0.5, -0.4, -0.25)),
    ((0.5, -0.4, -0.25), (0.5, 0.4, -0.25)), ((0.5, 0.4, -0.25), (-0.5, 0.4, -0.25)),
    ((-0.5, 0.4, -0.25), (-0.5, -0.4, -0.25)), ((-0.5, -0.4, -0.25), (0.5, -0.4, -0.25)),
    ((0.5, 0, -0.25), (1.1, 0, -0.25)),  # nose arrow
    ((1.1, 0, -0.25), (0.95, 0.1, -0.25)), ((1.1, 0, -0.25), (0.95, -0.1, -0.25)),
]
MOTORS = {  # name: (position, colour)  M1 FL, M2 FR, M3 RR, M4 RL
    "M1": ((ARM, -ARM, 0), "#ff5555"), "M2": ((ARM, ARM, 0), "#ff5555"),
    "M3": ((-ARM, ARM, 0), "#55ff55"), "M4": ((-ARM, -ARM, 0), "#55ff55"),
}

def rot_body_to_world(roll, pitch, yaw):
    """ZYX Euler (aerospace): world = Rz(yaw) Ry(pitch) Rx(roll) body."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )

def apply(R, p):
    return tuple(R[i][0] * p[0] + R[i][1] * p[1] + R[i][2] * p[2] for i in range(3))

# Camera: looking at the drone from behind-right-above. World X fwd, Y right, Z down.
CAM_YAW, CAM_ELEV = math.radians(-35), math.radians(28)

def project(p, w, h, scale):
    x, y, z = p
    # rotate world so camera looks along +X, then tilt
    cx = x * math.cos(CAM_YAW) - y * math.sin(CAM_YAW)
    cy = x * math.sin(CAM_YAW) + y * math.cos(CAM_YAW)
    up = -z * math.cos(CAM_ELEV) - cx * math.sin(CAM_ELEV)
    depth = cx * math.cos(CAM_ELEV) - z * math.sin(CAM_ELEV)
    f = 1.0 / (1.0 + 0.18 * (2.5 - depth))  # mild perspective
    return w / 2 + cy * scale * f, h / 2 - up * scale * f

# ---------------- serial thread ----------------------------------------------
class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.05, dsrdtr=False, rtscts=False)
        self.ser.dtr = False; self.ser.rts = False
        self.att = queue.Queue(); self.log = queue.Queue()
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                buf += self.ser.read(256)
            except serial.SerialException:
                self.log.put("!! serial error, port gone?\n"); self.alive = False; return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode(errors="replace").rstrip("\r")
                if s.startswith("att "):
                    try:
                        v = s.split()
                        self.att.put(tuple(float(x) for x in v[1:10]))
                    except ValueError:
                        pass
                else:
                    self.log.put(s + "\n")
                    if "Ready. Type" in s:  # board (re)booted: telemetry mode is off again
                        self.send("plot att")

    def send(self, cmd):
        try: self.ser.write((cmd + "\n").encode())
        except serial.SerialException: pass

    def reset(self):
        self.ser.rts = True; time.sleep(0.1); self.ser.rts = False

    def close(self):
        self.alive = False
        try: self.send("plot off"); time.sleep(0.05); self.ser.close()
        except Exception: pass

# ---------------- UI ------------------------------------------------------------
class App:
    def __init__(self, root, link):
        self.root, self.link = root, link
        root.title(f"Drone attitude - {PORT}")
        root.configure(bg="#111")
        self.cv = tk.Canvas(root, width=640, height=420, bg="#111", highlightthickness=0)
        self.cv.pack(fill="both", expand=True)
        bar = tk.Frame(root, bg="#111"); bar.pack(fill="x")
        self.info = tk.Label(bar, text="waiting for data... (send `plot att`)", fg="#ccc", bg="#111",
                             font=("Consolas", 11), anchor="w")
        self.info.pack(side="left", padx=8)
        tk.Button(bar, text="Zero yaw", command=self.zero_yaw).pack(side="right", padx=4)
        tk.Button(bar, text="Reset board", command=self.link.reset).pack(side="right", padx=4)
        self.txt = tk.Text(root, height=12, bg="#000", fg="#9f9", font=("Consolas", 10), insertbackground="#fff")
        self.txt.pack(fill="both", expand=False)
        row = tk.Frame(root); row.pack(fill="x")
        self.ent = tk.Entry(row, font=("Consolas", 11)); self.ent.pack(side="left", fill="x", expand=True)
        self.ent.bind("<Return>", self.on_cmd); self.ent.focus_set()
        tk.Label(row, text="command + Enter (help, i2c, selftest, orient, cal...)").pack(side="left", padx=6)
        self.roll = self.pitch = self.yaw = 0.0
        self.motors = [0, 0, 0, 0]; self.armed = 0; self.acc = 1.0
        self.lastFrame = time.time(); self.stopping = False
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "stopping", True))  # Ctrl+C in the console
        self.last = time.time(); self.rate = 0.0; self.n = 0; self.t0 = time.time()
        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.after(500, lambda: self.link.send("plot att"))
        self.tick()

    def zero_yaw(self): self.yaw = 0.0

    def on_cmd(self, _):
        c = self.ent.get().strip(); self.ent.delete(0, "end")
        if c:
            self.link.send(c)
            if c.startswith("plot") and c != "plot att":
                self.root.after(3000, lambda: self.link.send("plot att"))  # keep the stream alive

    def quit(self):
        self.link.close(); self.root.destroy()

    def tick(self):
        if self.stopping or not self.link.alive:
            self.quit(); return
        got = None
        while True:
            try: got = self.link.att.get_nowait(); self.n += 1
            except queue.Empty: break
        if got:
            now = time.time(); dt = now - self.last; self.last = now
            self.roll, self.pitch, gz = got[0], got[1], got[2]
            self.motors = list(got[3:7]); self.armed = int(got[7])
            if len(got) > 8: self.acc = got[8]
            self.lastFrame = now
            if dt < 0.5: self.yaw += gz * dt  # display-only heading
            if now - self.t0 > 1: self.rate = self.n / (now - self.t0); self.n = 0; self.t0 = now
        while True:
            try: s = self.link.log.get_nowait()
            except queue.Empty: break
            self.txt.insert("end", s); self.txt.see("end")
            if int(self.txt.index("end-1c").split(".")[0]) > 400: self.txt.delete("1.0", "50.0")
        if time.time() - self.lastFrame > 2.0:  # stream died (reset, `plot off`...): ask again
            self.link.send("plot att"); self.lastFrame = time.time()
        self.draw()
        self.root.after(20, self.tick)

    def draw(self):
        cv = self.cv; cv.delete("all")
        w, h = cv.winfo_width(), cv.winfo_height(); scale = min(w, h) * 0.28
        R = rot_body_to_world(math.radians(self.roll), math.radians(self.pitch), math.radians(self.yaw))
        # ground grid (world frame) for reference
        for i in range(-2, 3):
            a = project((i, -2, 0.6), w, h, scale); b = project((i, 2, 0.6), w, h, scale)
            cv.create_line(*a, *b, fill="#2a2a2a")
            a = project((-2, i, 0.6), w, h, scale); b = project((2, i, 0.6), w, h, scale)
            cv.create_line(*a, *b, fill="#2a2a2a")
        for p, q in BODY:
            a = project(apply(R, p), w, h, scale); b = project(apply(R, q), w, h, scale)
            cv.create_line(*a, *b, fill="#8ab4f8", width=2)
        for i, (name, (pos, col)) in enumerate(MOTORS.items()):
            x, y = project(apply(R, pos), w, h, scale)
            r = 6 + 14 * self.motors[i]
            cv.create_oval(x - r, y - r, x + r, y + r, outline=col, width=2)
            cv.create_text(x, y - r - 9, text=name, fill=col, font=("Consolas", 9))
        # world axes legend
        cv.create_text(10, 10, anchor="nw", fill="#777", font=("Consolas", 9),
                       text="nose arrow = forward   red = front motors   green = rear")
        cv.create_text(w - 10, 10, anchor="ne", fill="#f55" if self.armed else "#5c5", font=("Consolas", 11, "bold"),
                       text="ARMED" if self.armed else "disarmed")
        accOk = 0.8 < self.acc < 1.2  # outside this the AHRS ignores the accel -> gyro-only drift
        self.info.config(fg="#ccc" if accOk else "#f66",
                         text=f"roll {self.roll:+7.2f}  pitch {self.pitch:+7.2f}  yaw(int) {self.yaw:+7.1f}  "
                              f"|a| {self.acc:.3f}g{'' if accOk else ' <-- accel ignored, expect drift'}  "
                              f"M {self.motors[0]:.2f} {self.motors[1]:.2f} {self.motors[2]:.2f} {self.motors[3]:.2f}  {self.rate:.0f} Hz")

def main():
    try:
        link = Link(PORT)
    except serial.SerialException as e:
        print(f"cannot open {PORT}: {e}\nClose the Arduino Serial Monitor / monitor.ps1 first."); sys.exit(1)
    root = tk.Tk(); root.geometry("760x720")
    App(root, link)
    root.mainloop()

if __name__ == "__main__":
    main()
