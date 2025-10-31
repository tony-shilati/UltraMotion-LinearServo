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
INACTIVITY_TIMEOUT = 3.0  # seconds


def find_teensy_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or '').lower()
        if 'teensy' in desc or 'usb' in desc:
            return p.device
    return None


def ensure_output_dir():
    out_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'outputs')
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def read_from_serial(port=None, baud=BAUD_RATE, inactivity_timeout=INACTIVITY_TIMEOUT):
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
    except Exception as e:
        # serial.SerialException may be raised; catch broadly to be user-friendly
        print(f"Failed to open port {port}: {e}")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        raise SystemExit(1)

    out_dir = ensure_output_dir()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f"MoveMotorTelemetry_{timestamp}.csv")

    # Separate storage for telemetry and encoder messages
    telemetry_times = []
    telemetry_vals = []
    encoder_times = []
    encoder_vals = []

    last_recv = time.time()

    print(f"Reading lines (timeout after {inactivity_timeout}s of inactivity)...")
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # write raw records with a type column so we store the original stream
        writer.writerow(['type', 'value', 'time'])

        try:
            while True:
                try:
                    raw = ser.readline()
                except (serial.SerialException, OSError) as e:
                    # Happens when device is unplugged or OS-level read fails
                    print(f"\nSerial read error: {e}")
                    print("Stopping capture. If the device was unplugged, reconnect and re-run the script.")
                    break
                if not raw:
                    # no data this read
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

                # Accept lines starting with 'T:' or 'E:' (case-insensitive)
                l = line.strip()
                if len(l) == 0:
                    continue

                msg_type = None
                payload = None
                if l[0].upper() == 'T' and l[1:2] == ':':
                    msg_type = 'T'
                    payload = l[2:]
                elif l[0].upper() == 'E' and l[1:2] == ':':
                    msg_type = 'E'
                    payload = l[2:]
                else:
                    # also accept bare comma/whitespace triple as fallback (legacy)
                    # keep compatibility: try to split into three parts
                    parts = [p.strip() for p in l.replace('\t', ' ').replace(';', ',').split(',') if p.strip()]
                    if len(parts) == 1:
                        parts = l.split()
                    if len(parts) >= 3:
                        # legacy triple: telemetry, encoder, time
                        try:
                            tval = float(parts[0])
                            enc = float(parts[1])
                            tstamp = float(parts[2])
                        except ValueError:
                            print(f"Skipping unparsable legacy line: '{line}'")
                            continue
                        # write two raw records so user can trace original stream
                        writer.writerow(['T', f"{tval:.4f}", f"{tstamp:.4f}"])
                        writer.writerow(['E', f"{enc:.4f}", f"{tstamp:.4f}"])
                        telemetry_vals.append(tval); telemetry_times.append(tstamp)
                        encoder_vals.append(enc); encoder_times.append(tstamp)
                        if (len(telemetry_times) + len(encoder_times)) % 50 == 0:
                            print(f"Captured total records: T={len(telemetry_times)}, E={len(encoder_times)}")
                        continue
                    else:
                        print(f"Skipping malformed line: '{line}'")
                        continue

                # payload should be value,time (comma or whitespace separated)
                if payload is None:
                    print(f"Skipping unparsable line: '{line}'")
                    continue

                # normalize separators
                parts = [p.strip() for p in payload.replace('\t', ' ').replace(';', ',').split(',') if p.strip()]
                if len(parts) == 1:
                    parts = payload.split()

                if len(parts) < 2:
                    print(f"Skipping malformed payload: '{payload}'")
                    continue

                try:
                    val = float(parts[0])
                    tstamp = float(parts[1])
                except ValueError:
                    print(f"Skipping unparsable numbers in line: '{line}'")
                    continue

                # record raw and typed data
                writer.writerow([msg_type, f"{val:.4f}", f"{tstamp:.4f}"])
                if msg_type == 'T':
                    telemetry_vals.append(val)
                    telemetry_times.append(tstamp)
                else:
                    encoder_vals.append(val)
                    encoder_times.append(tstamp)

                if (len(telemetry_times) + len(encoder_times)) % 50 == 0:
                    print(f"Captured total records: T={len(telemetry_times)}, E={len(encoder_times)}")

        except KeyboardInterrupt:
            print('\nInterrupted by user, stopping capture.')
        finally:
            ser.close()

    print(f"Saved CSV to: {csv_path}")
    return (np.array(telemetry_times), np.array(telemetry_vals),
            np.array(encoder_times), np.array(encoder_vals), csv_path)


def _compute_uniform_fft(times, values):
    """Interpolate values to a uniform grid and compute FFT.

    This function is tolerant of small datasets, equal timestamps, and
    irregular sampling. It tries to compute a reasonable dt from the
    median positive time difference; if that fails it falls back to
    assuming unit spacing.

    Returns (freqs, magnitudes, nyquist) or None if not enough data.
    """
    if len(times) < 2:
        return None

    times = np.array(times, dtype=float)
    values = np.array(values, dtype=float)
    sort_idx = np.argsort(times)
    times = times[sort_idx]
    values = values[sort_idx]

    diffs = np.diff(times)
    positive_diffs = diffs[np.isfinite(diffs) & (diffs > 0)]

    if positive_diffs.size > 0:
        dt = float(np.median(positive_diffs))
    else:
        # fallback: use 1.0 as nominal dt between samples
        dt = 1.0

    if not np.isfinite(dt) or dt <= 0:
        dt = 1.0

    fs = 1.0 / dt

    t_start, t_end = times[0], times[-1]
    duration = t_end - t_start

    if duration <= 0:
        # timestamps identical or decreasing; treat samples as uniformly
        # spaced with spacing dt and construct a uniform grid
        t_uniform = np.arange(len(values), dtype=float) * dt
        y_uniform = values.copy()
    else:
        # Choose number of points for uniform interpolation. Use at least
        # the original sample count, and at least 256 for decent FFT
        # resolution. If original duration is short relative to dt, N will
        # be >= original count.
        original_count = len(times)
        est_n = int(ceil(duration / dt)) if duration > 0 else original_count
        N = max(256, original_count, est_n)
        t_uniform = np.linspace(t_start, t_end, N)
        y_uniform = np.interp(t_uniform, times, values)

    if len(y_uniform) < 4:
        # Not enough samples for a meaningful FFT
        return None

    # apply window to reduce leakage
    window = np.hanning(len(y_uniform))
    yw = y_uniform * window

    yf = np.fft.rfft(yw)
    xf = np.fft.rfftfreq(len(yw), d=(t_uniform[1] - t_uniform[0]))
    # normalize magnitude by window energy to keep scale consistent
    mag = np.abs(yf) / np.sum(window)
    nyquist = 0.5 * fs
    return xf, mag, nyquist


def plot_data(telemetry_times, telemetry_vals, encoder_times, encoder_vals, csv_path):
    if len(telemetry_times) == 0 and len(encoder_times) == 0:
        print("No data to plot.")
        return

    out_dir = os.path.dirname(csv_path)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    plot_path = os.path.join(out_dir, f"MoveMotorPlot_{timestamp}.svg")

    # two-row figure: time domain (top) and frequency domain (bottom)
    fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(10, 8),
                                          gridspec_kw={'height_ratios': [2, 1]})

    # Time domain: telemetry (blue) and encoder (red)
    if len(telemetry_times) > 0:
        ax_time.plot(telemetry_times, telemetry_vals, 'b-', label='Telemetry')
    ax_time.set_xlabel('Time (s)')
    ax_time.set_ylabel('Telemetry', color='b')
    ax_time.tick_params(axis='y', labelcolor='b')

    ax_time2 = ax_time.twinx()
    if len(encoder_times) > 0:
        ax_time2.plot(encoder_times, encoder_vals, 'r-', label='Encoder')
    ax_time2.set_ylabel('Encoder', color='r')
    ax_time2.tick_params(axis='y', labelcolor='r')

    ax_time.grid(True, which='both', alpha=0.3)
    ax_time.set_title('Telemetry and Encoder vs Time')

    # Frequency domain: compute FFTs for each series
    plotted_any = False
    nyq = None

    t_fft = _compute_uniform_fft(telemetry_times, telemetry_vals) if len(telemetry_times) > 1 else None
    if t_fft is not None:
        xf, mag, nyq_t = t_fft
        ax_fft.plot(xf, mag, color='b', label='Telemetry')
        plotted_any = True
        nyq = nyq_t if nyq is None else min(nyq, nyq_t)

    e_fft = _compute_uniform_fft(encoder_times, encoder_vals) if len(encoder_times) > 1 else None
    if e_fft is not None:
        xf_e, mag_e, nyq_e = e_fft
        ax_fft.plot(xf_e, mag_e, color='r', label='Encoder')
        plotted_any = True
        nyq = nyq_e if nyq is None else min(nyq, nyq_e)

    if plotted_any:
        ax_fft.set_xlabel('Frequency (Hz)')
        ax_fft.grid(True, which='both', alpha=0.3)

        # set x limit to 0-25 Hz if possible
        if nyq is not None and nyq > 0:
            xlim = min(25.0, nyq)
        else:
            xlim = 25.0
        ax_fft.set_xlim(0, xlim)

        xticks = list(range(0, int(ceil(xlim)) + 1))
        ax_fft.set_xticks(xticks)

        # If both telemetry and encoder FFTs exist, plot encoder magnitude
        # on a separate right-hand y-axis so scales do not mask each other.
        handles = []
        labels = []
        # telemetry was plotted on ax_fft (if present)
        if t_fft is not None:
            h1, l1 = ax_fft.get_legend_handles_labels()
            handles += h1; labels += l1

        if e_fft is not None:
            # create twin y-axis for encoder FFT
            ax_fft2 = ax_fft.twinx()
            ax_fft2.plot(xf_e, mag_e, color='r', label='Encoder')
            ax_fft2.set_ylabel('Encoder amplitude', color='r')
            ax_fft2.tick_params(axis='y', labelcolor='r')
            h2, l2 = ax_fft2.get_legend_handles_labels()
            handles += h2; labels += l2

        # combine legends from both axes
        if handles:
            ax_fft.legend(handles, labels)
    else:
        ax_fft.text(0.5, 0.5, 'Not enough data for FFT', ha='center', va='center')

    fig.tight_layout()
    plt.savefig(plot_path, format='svg')
    print(f"Saved plot to: {plot_path}")
    plt.show()


def main():
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]

    telemetry_times, telemetry_vals, encoder_times, encoder_vals, csv = read_from_serial(port)
    plot_data(telemetry_times, telemetry_vals, encoder_times, encoder_vals, csv)


if __name__ == '__main__':
    main()