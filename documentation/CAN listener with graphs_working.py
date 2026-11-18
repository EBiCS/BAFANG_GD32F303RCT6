import time
import csv
import usb
import can
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog
from collections import deque
from datetime import datetime
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# -----------------------------
# CAN configuration
# -----------------------------
CAN_ID = 0x00010203
CAN_MASK = 0x1FFFFFFF

bus = None
running = False
start_time = 0.0

# -----------------------------
# Plot buffers
# -----------------------------
samples = 300
time_history = deque(maxlen=samples)
channel_history = [deque(maxlen=samples) for _ in range(4)]

# -----------------------------
# Log buffer
# -----------------------------
log_data = []

# -----------------------------
# Helpers
# -----------------------------
def int16_from_u16(value):
    return np.int16(np.uint16(value))

def decode_channels(data):
    if len(data) < 8:
        return [0, 0, 0, 0]

    ch1 = int16_from_u16((data[0] << 8) | data[1])
    ch2 = int16_from_u16((data[2] << 8) | data[3])
    ch3 = int16_from_u16((data[4] << 8) | data[5])
    ch4 = int16_from_u16((data[6] << 8) | data[7])
    return [ch1, ch2, ch3, ch4]

# -----------------------------
# GUI
# -----------------------------
root = tk.Tk()
root.title("Live CAN Data")
root.geometry("900x700")

status_var = tk.StringVar(value="Stopped")

top_bar = tk.Frame(root)
top_bar.pack(fill="x", padx=10, pady=10)

def save_log_csv():
    global log_data

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_name = f"can_log_{ts}.csv"

    filename = filedialog.asksaveasfilename(
        title="Save CAN log as CSV",
        defaultextension=".csv",
        initialfile=default_name,
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )

    if not filename:
        return

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "Ch1", "Ch2", "Ch3", "Ch4"])

        for row in log_data:
            t, ch1, ch2, ch3, ch4 = row
            writer.writerow([
                datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                ch1, ch2, ch3, ch4
            ])

    status_var.set(f"Saved: {filename}")

def start_live_data():
    global bus, running, start_time, log_data

    if running:
        return

    try:
        dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
        if dev is None:
            raise RuntimeError("USB CAN adapter not found")

        bus = can.Bus(interface="gs_usb", channel=dev.product, index=0, bitrate=250000)
        bus.set_filters([
            {"can_id": CAN_ID, "can_mask": CAN_MASK, "extended": True}
        ])

        # reset buffers
        time_history.clear()
        for dq in channel_history:
            dq.clear()

        for i in range(4):
            lines[i].set_data([], [])

        for ax in axes:
            ax.set_xlim(0, 1)
            ax.set_ylim(-32768, 32767)

        log_data = []
        running = True
        start_time = time.time()
        status_var.set("Running")

        start_button.config(state="disabled")
        stop_button.config(state="normal")

        root.after(20, poll_can)

    except Exception as e:
        running = False
        status_var.set(f"Error: {e}")
        print("Start error:", e)

def stop_live_data():
    global bus, running

    running = False
    status_var.set("Stopped")

    if bus is not None:
        try:
            bus.shutdown()
        except Exception:
            pass
        bus = None

    start_button.config(state="normal")
    stop_button.config(state="disabled")

    save_log_csv()

start_button = ttk.Button(top_bar, text="Start", command=start_live_data)
start_button.pack(side="left", padx=(0, 10))

stop_button = ttk.Button(top_bar, text="Stop", command=stop_live_data, state="disabled")
stop_button.pack(side="left")

status_label = ttk.Label(top_bar, textvariable=status_var)
status_label.pack(side="left", padx=(20, 0))

# Matplotlib figure
fig = Figure(figsize=(10, 8), dpi=100)

axes = [fig.add_subplot(4, 1, 1)]
for i in range(1, 4):
    axes.append(fig.add_subplot(4, 1, i + 1, sharex=axes[0]))

fig.suptitle("Live CAN data (4 channels)", fontsize=12)

lines = []
for i, ax in enumerate(axes):
    line, = ax.plot([], [], lw=1.5)
    lines.append(line)
    ax.set_ylabel(f"Ch{i+1}")
    ax.grid(True)
    ax.set_ylim(-32768, 32767)

axes[-1].set_xlabel("Time (s)")

canvas = FigureCanvasTkAgg(fig, master=root)
toolbar = NavigationToolbar2Tk(canvas, root)
toolbar.update()
toolbar.pack(side="top", fill="x")
canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=(0, 10))

def poll_can():
    global running

    if not running or bus is None:
        return

    try:
        msg = bus.recv(timeout=0.01)
    except Exception as e:
        status_var.set(f"Receive error: {e}")
        return

    if msg is not None and msg.arbitration_id == CAN_ID:
        values = decode_channels(msg.data)
        t = time.time()

        log_data.append((t, values[0], values[1], values[2], values[3]))
        time_history.append(t - start_time)

        for i in range(4):
            channel_history[i].append(values[i])

        x = list(time_history)

        for i in range(4):
            lines[i].set_data(x, list(channel_history[i]))

        if len(x) > 1:
            x_min = max(0, x[0])
            x_max = max(1, x[-1] + 0.5)
        else:
            x_min = 0
            x_max = 1

        for ax in axes:
            ax.set_xlim(x_min, x_max)

        for i in range(4):
            vals = list(channel_history[i])
            if vals:
                min_y = min(vals)
                max_y = max(vals)
                pad = max(10, (max_y - min_y) * 0.1)
                axes[i].set_ylim(max(-32768, min_y - pad), min(32767, max_y + pad))

        fig.canvas.draw_idle()

    if running:
        root.after(20, poll_can)

root.protocol("WM_DELETE_WINDOW", stop_live_data)
root.mainloop()
