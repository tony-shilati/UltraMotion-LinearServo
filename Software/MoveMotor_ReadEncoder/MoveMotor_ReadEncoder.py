#!/usr/bin/env python3
"""MoveMotor_ReadEncoder

Connect to a Teensy 4.1 over USB serial, read telemetry lines formatted as
  telemetry, encoder, time
collect samples until no message is received for a configurable inactivity
timeout (default 5 seconds). After the timeout the script saves a timestamped
CSV in ../outputs and creates a vector (SVG) plot of telemetry and encoder vs time.

Usage:
  python MoveMotor_ReadEncoder.py [serial_port]
If serial_port is not supplied the script will attempt to auto-detect a USB
port with 'teensy' or 'usb' in the description.
"""

import serial
import serial.tools.list_ports
import time
import csv
import os
import sys
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from math import ceil

# Configuration
BAUD_RATE = 115200
INACTIVITY_TIMEOUT = 5.0  # seconds
# Conversion factors
LOADCELL_CONVERSION = 1.0
SERVO_CONVERSION = 5.782369e-4  # mm/ticks


def find_teensy_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or '').lower()
        if 'teensy' in desc or 'usb' in desc:
            return p.device
    return None


def ensure_output_dir():
    # place outputs directory alongside this script
    out_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def read_from_serial(port=None, baud=BAUD_RATE, inactivity_timeout=INACTIVITY_TIMEOUT):
    """Read telemetry from a serial port (blocking) and return arrays.

    Returns:
      (cmd_times, cmd_vals, loadcell_times, loadcell_vals, encoder_times, encoder_vals, csv_path)
    """
    opened_here = False
    if port is None:
        port = find_teensy_port()
        if port is None:
            print("No Teensy auto-detected. Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}: {p.description}")
            port = input("Enter serial port (e.g. /dev/ttyACM0 or COM3): ")

    print(f"Opening port {port} @ {baud} baud")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1.0)
        opened_here = True
    except Exception as e:
        print(f"Failed to open port {port}: {e}")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        raise SystemExit(1)

    out_dir = ensure_output_dir()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f"MoveMotorTelemetry_{timestamp}.csv")

    # storage for G/S messages only
    g_times = []
    g_vals = []
    s1_vals = []
    s2_vals = []
    s_times = []

    last_recv = time.time()

    print(f"Reading lines (timeout after {inactivity_timeout}s of inactivity)...")
    try:
        while True:
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as e:
                print(f"\nSerial read error: {e}")
                break
            if not raw:
                if (time.time() - last_recv) >= inactivity_timeout:
                    print('\nInactivity timeout reached, stopping capture.')
                    break
                continue

            last_recv = time.time()
            try:
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception:
                line = str(raw)

            if not line:
                continue

            l = line.strip()
            if len(l) == 0:
                continue

            colon = l.find(':')
            if colon <= 0:
                # skip malformed
                continue
            prefix = l[:colon].upper()
            payload = l[colon+1:]

            if prefix == 'G':
                # G:data,time
                parts = [p.strip() for p in payload.replace('\t', ' ').replace(';', ',').split(',') if p.strip()]
                if len(parts) == 1:
                    parts = payload.split()
                if len(parts) < 2:
                    print(f"Skipping malformed G payload: '{payload}'")
                    continue
                try:
                    val = float(parts[0])
                    tstamp = float(parts[1])
                except ValueError:
                    print(f"Skipping unparsable G line: '{line}'")
                    continue
                # convert G (command) to servo units before storing
                g_vals.append(val * SERVO_CONVERSION)
                g_times.append(tstamp)

            elif prefix == 'S':
                # S:data1,data2,time
                parts = [p.strip() for p in payload.replace('\t', ' ').replace(';', ',').split(',') if p.strip()]
                if len(parts) == 1:
                    parts = payload.split()
                if len(parts) < 3:
                    print(f"Skipping malformed S payload: '{payload}'")
                    continue
                try:
                    s1 = float(parts[0])
                    s2 = float(parts[1])
                    tstamp = float(parts[2])
                except ValueError:
                    print(f"Skipping unparsable S line: '{line}'")
                    continue
                s1_vals.append(s1)
                s2_vals.append(s2)
                s_times.append(tstamp)

            # progress
            total = len(g_times) + len(s_times)
            if total % 50 == 0:
                print(f"Captured total G/S samples: G={len(g_times)}, S={len(s_times)}")

    except KeyboardInterrupt:
        print('\nInterrupted by user, stopping capture.')
    finally:
        if opened_here:
            try:
                ser.close()
            except Exception:
                pass

    # Write GS CSV with columns: [G, G_time, S1, S2, S_time]
    gs_csv_path = os.path.join(out_dir, f"MoveMotor_GS_{timestamp}.csv")
    try:
        with open(gs_csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['G', 'G_time', 'S1', 'S2', 'S_time'])
            max_len = max(len(g_vals), len(s1_vals), len(s2_vals), len(s_times))
            for i in range(max_len):
                row = []
                if i < len(g_vals):
                    row.append(f"{g_vals[i]:.6f}")
                    row.append(f"{g_times[i]:.6f}")
                else:
                    row.extend(['', ''])

                if i < len(s1_vals):
                    row.append(f"{s1_vals[i]:.6f}")
                else:
                    row.append('')

                if i < len(s2_vals):
                    row.append(f"{s2_vals[i]:.6f}")
                else:
                    row.append('')

                if i < len(s_times):
                    row.append(f"{s_times[i]:.6f}")
                else:
                    row.append('')

                writer.writerow(row)
    except Exception as e:
        print(f"Failed to write GS CSV: {e}")

    print(f"Saved GS CSV to: {gs_csv_path}")
    # Return legacy-shaped tuple with empty arrays and the GS csv path for compatibility
    return (np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), gs_csv_path)






def main():
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]

    cmd_times, cmd_vals, loadcell_times, loadcell_vals, encoder_times, encoder_vals, csv = read_from_serial(port)
    # plot_data expects (telemetry_times, telemetry_vals, encoder_times, encoder_vals, gnd_times, gnd_vals, csv)
    # we map load-cell -> telemetry slot and cmd -> gnd slot for plotting


if __name__ == '__main__':
    main()