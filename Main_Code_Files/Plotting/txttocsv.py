import re
import csv
import sys
import os

# ── Configure input/output paths here ───────────────────────────────────────
INPUT_FILE  = "C:\\Users\\dhyey\\OneDrive\\Desktop\\motorfeed.txt"          # <-- change to your .txt file path
OUTPUT_FILE = "C:\\Users\\dhyey\\OneDrive\\Desktop\\motor_feedback2.csv" # <-- output CSV path
# ────────────────────────────────────────────────────────────────────────────

if not os.path.exists(INPUT_FILE):
    print(f"Error: file '{INPUT_FILE}' not found.")
    sys.exit(1)

PATTERN = re.compile(
    r'\[MOTOR FEEDBACK\]\s+'
    r'CAN_ID=(\d+)\s+RX_ID=(\d+)\s+LEN=(\d+)\s*\|\s*'
    r'Motor_ID_byte=(\d+)\s*\|\s*'
    r'p_out=([\-\d.]+)\s*rad\s*\|\s*'
    r'v_out=([\-\d.]+)\s*rad/s\s*\|\s*'
    r't_out=([\-\d.]+)\s*Nm\s*\|\s*'
    r'p_cmd=([\-\d.]+)\s*rad\s*\|\s*'
    r'v_cmd=([\-\d.]+)\s*rad/s\s*\|\s*'
    r'kp=([\-\d.]+)\s*\|\s*'
    r'kd=([\-\d.]+)\s*\|\s*'
    r'raw=\[([0-9A-Fa-f\s]+)\]'
)

COLUMNS = [
    "sample",
    "CAN_ID", "RX_ID", "LEN", "Motor_ID_byte",
    "p_out_rad", "v_out_rad_s", "t_out_Nm",
    "p_cmd_rad", "v_cmd_rad_s",
    "kp", "kd",
    "raw_bytes"
]

with open(INPUT_FILE, "r", errors="replace") as f:
    raw = f.read()

rows = [
    [i, *m.group(1,2,3,4,5,6,7,8,9,10,11), m.group(12).strip()]
    for i, m in enumerate(PATTERN.finditer(raw), 1)
]

with open(OUTPUT_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(COLUMNS)
    writer.writerows(rows)

print(f"Done — {len(rows)} samples written to '{OUTPUT_FILE}'")
