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

    # storage
    cmd_times = []
    cmd_vals = []
    loadcell_times = []
    loadcell_vals = []
    encoder_times = []
    encoder_vals = []
    telemetry_times = []
    telemetry_vals = []

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

            # prefix parsing
            msg_type = None
            payload = None
            colon = l.find(':')
            if colon > 0:
                prefix = l[:colon].upper()
                payload = l[colon+1:]
                if prefix == 'LC':
                    msg_type = 'LC'
                elif prefix == 'T' or prefix == 'TELEMETRY':
                    msg_type = 'T'
                elif prefix == 'E' or prefix == 'ENCODER':
                    msg_type = 'E'
                elif prefix == 'GND':
                    msg_type = 'GND'
                else:
                    msg_type = None
                    payload = None

            # legacy triple: value, encoder, timestamp
            if payload is None:
                parts = [p.strip() for p in l.replace('\t', ' ').replace(';', ',').split(',') if p.strip()]
                if len(parts) == 1:
                    parts = l.split()
                if len(parts) >= 3:
                    try:
                        tval = float(parts[0])
                        enc = float(parts[1])
                        tstamp = float(parts[2])
                    except ValueError:
                        print(f"Skipping unparsable legacy line: '{line}'")
                        continue
                    loadcell_vals.append(tval * LOADCELL_CONVERSION)
                    loadcell_times.append(tstamp)
                    encoder_vals.append(enc); encoder_times.append(tstamp)
                    if (len(loadcell_times) + len(encoder_times)) % 50 == 0:
                        print(f"Captured total records: LC={len(loadcell_times)}, E={len(encoder_times)}")
                    continue
                else:
                    print(f"Skipping malformed line: '{line}'")
                    continue

            # now payload should be value,time
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

            if msg_type == 'LC':
                loadcell_vals.append(val * LOADCELL_CONVERSION)
                loadcell_times.append(tstamp)
            elif msg_type == 'T':
                telemetry_vals.append(val)
                telemetry_times.append(tstamp)
            elif msg_type == 'E':
                encoder_vals.append(val)
                encoder_times.append(tstamp)
            elif msg_type == 'GND':
                cmd_vals.append(val * SERVO_CONVERSION)
                cmd_times.append(tstamp)

            if (len(loadcell_times) + len(encoder_times) + len(cmd_times)) % 50 == 0:
                print(f"Captured total records: LC={len(loadcell_times)}, E={len(encoder_times)}, CMD={len(cmd_times)}")

    except KeyboardInterrupt:
        print('\nInterrupted by user, stopping capture.')
    finally:
        if opened_here:
            try:
                ser.close()
            except Exception:
                pass

    # Save consolidated CSV with columns [cmd, cmd_time, load_cell, load_cell_time, encoder, encoder_time]
    try:
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['cmd', 'cmd_time', 'load_cell', 'load_cell_time', 'encoder', 'encoder_time'])
            max_len = max(len(cmd_vals), len(loadcell_vals), len(encoder_vals))
            for i in range(max_len):
                row = []
                if i < len(cmd_vals):
                    row.append(f"{cmd_vals[i]:.6f}")
                    row.append(f"{cmd_times[i]:.6f}")
                else:
                    row.extend(['', ''])

                if i < len(loadcell_vals):
                    row.append(f"{loadcell_vals[i]:.6f}")
                    row.append(f"{loadcell_times[i]:.6f}")
                else:
                    row.extend(['', ''])

                if i < len(encoder_vals):
                    row.append(f"{encoder_vals[i]:.6f}")
                    row.append(f"{encoder_times[i]:.6f}")
                else:
                    row.extend(['', ''])

                writer.writerow(row)
    except Exception as e:
        print(f"Failed to write consolidated CSV: {e}")

    print(f"Saved CSV to: {csv_path}")
    return (np.array(cmd_times), np.array(cmd_vals), np.array(loadcell_times), np.array(loadcell_vals), np.array(encoder_times), np.array(encoder_vals), csv_path)


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

    # subtract DC / center value before windowing to remove large zero-frequency component
    y_uniform = y_uniform - np.mean(y_uniform)

    # apply window to reduce leakage
    window = np.hanning(len(y_uniform))
    yw = y_uniform * window

    # compute FFT on the windowed, uniformly sampled signal
    yf = np.fft.rfft(yw)
    d_uniform = float(t_uniform[1] - t_uniform[0])
    xf = np.fft.rfftfreq(len(yw), d=d_uniform)

    # Compute single-sided amplitude spectrum and compensate for the window
    # Standard single-sided amplitude (for real signals) is (2/N)*|Y|,
    # but because we applied a window we correct by dividing by the window sum
    # (equivalent to multiplying by N/sum(window)). Combining these gives
    # 2 * |Y| / sum(window)
    mag = 2.0 * np.abs(yf) / np.sum(window)

    # sampling frequency and nyquist based on the uniform grid spacing
    fs_uniform = 1.0 / d_uniform if d_uniform > 0 else fs
    nyquist = 0.5 * fs_uniform
    return xf, mag, nyquist


def _interpolate_to_uniform(times, values, min_N=256):
    """Interpolate times/values to a uniform grid.

    Returns (t_uniform, y_uniform, fs) or None if not enough data.
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
        dt = 1.0
    if not np.isfinite(dt) or dt <= 0:
        dt = 1.0

    t_start, t_end = times[0], times[-1]
    duration = t_end - t_start
    if duration <= 0:
        # fallback to using original spacing
        N = max(min_N, len(values))
        t_uniform = np.arange(len(values), dtype=float) * dt
        y_uniform = values.copy()
    else:
        original_count = len(times)
        est_n = int(ceil(duration / dt)) if duration > 0 else original_count
        N = max(min_N, original_count, est_n)
        t_uniform = np.linspace(t_start, t_end, N)
        y_uniform = np.interp(t_uniform, times, values)

    if len(y_uniform) < 4:
        return None

    fs_uniform = 1.0 / float(t_uniform[1] - t_uniform[0])
    return t_uniform, y_uniform, fs_uniform


def plot_data(telemetry_times, telemetry_vals, encoder_times, encoder_vals, gnd_times, gnd_vals, csv_path):
    if len(telemetry_times) == 0 and len(encoder_times) == 0 and len(gnd_times) == 0:
        print("No data to plot.")
        return

    out_dir = os.path.dirname(csv_path)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    plot_path = os.path.join(out_dir, f"MoveMotorPlot_{timestamp}.svg")

    # two-row figure: time domain (top) and frequency domain (bottom)
    fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(10, 8),
                                          gridspec_kw={'height_ratios': [2, 1]})

    # Time domain: plot telemetry, encoder and GND on a single axis (different colors)
    # Subtract center (mean) from T and GND for time-domain plotting
    telemetry_vals_plot = np.array(telemetry_vals) if len(telemetry_vals) > 0 else np.array([])
    gnd_vals_plot = np.array(gnd_vals) if len(gnd_vals) > 0 else np.array([])
    if telemetry_vals_plot.size > 0:
        telemetry_vals_plot = telemetry_vals_plot - np.mean(telemetry_vals_plot)
    if gnd_vals_plot.size > 0:
        gnd_vals_plot = gnd_vals_plot - np.mean(gnd_vals_plot)

    if len(telemetry_times) > 0:
        ax_time.plot(telemetry_times, telemetry_vals_plot, 'b-', label='Telemetry (T)')
    if len(encoder_times) > 0:
        ax_time.plot(encoder_times, encoder_vals, 'r-', label='Encoder')
    if len(gnd_times) > 0:
        ax_time.plot(gnd_times, gnd_vals_plot, 'g-', label='G-reading (G)')

    ax_time.set_xlabel('Time (s)')
    ax_time.set_ylabel('Signal')
    ax_time.grid(True, which='both', alpha=0.3)
    ax_time.set_title('Telemetry, Encoder and GND vs Time')

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
        # plot encoder FFT on the same axis as telemetry/GND so all magnitudes
        # share a common scale (user requested).
        ax_fft.plot(xf_e, mag_e, color='r', label='Encoder')
        plotted_any = True
        nyq = nyq_e if nyq is None else min(nyq, nyq_e)

    # GND FFT
    g_fft = _compute_uniform_fft(gnd_times, gnd_vals) if len(gnd_times) > 1 else None
    if g_fft is not None:
        xf_g, mag_g, nyq_g = g_fft
        ax_fft.plot(xf_g, mag_g, color='g', label='Command')
        plotted_any = True
        nyq = nyq_g if nyq is None else min(nyq, nyq_g)

    if plotted_any:
        ax_fft.set_xlabel('Frequency (Hz)')
        ax_fft.grid(True, which='both', alpha=0.3)

        # set x limit to 0-25 Hz if possible
        if nyq is not None and nyq > 0:
            xlim = min(160.0, nyq)
        else:
            xlim = 160.0
        ax_fft.set_xlim(0, xlim)

        xticks = list(range(0, int(ceil(xlim)) + 1))
        ax_fft.set_xticks(xticks)

        # single legend for ax_fft (includes telemetry, GND, encoder if plotted)
        ax_fft.legend()

    # --- Spectrogram for encoder (separate figure) ---
    # If encoder data exists, interpolate to uniform grid and display/save
    if len(encoder_times) > 3:
        uni = _interpolate_to_uniform(encoder_times, encoder_vals, min_N=256)
        if uni is not None:
            t_u, y_u, fs_u = uni
            try:
                # choose NFFT as a power-of-two up to a cap, but not larger than the data length
                max_cap = 4096
                nfft_candidate = min(max_cap, len(y_u))
                if nfft_candidate < 256:
                    NFFT = 256
                else:
                    # highest power of two <= nfft_candidate
                    NFFT = 2 ** int(np.floor(np.log2(nfft_candidate)))
                noverlap = int(NFFT * 0.75)

                spec_fig = plt.figure(figsize=(10, 4))
                ax_spec = spec_fig.add_subplot(1, 1, 1)

                # compute spectrogram using matplotlib.mlab.specgram to get the raw Pxx
                from matplotlib import mlab
                Pxx, freqs, bins = mlab.specgram(y_u, NFFT=NFFT, Fs=fs_u, noverlap=noverlap)

                # convert to dB, avoid log of zero
                eps = 1e-12
                Pxx_dB = 10.0 * np.log10(Pxx + eps)
                max_db = float(np.nanmax(Pxx_dB)) if Pxx_dB.size else 0.0
                vmin = max_db - 60.0

                # plot with time on x and frequency on y; Pxx has shape (freqs, times)
                im = ax_spec.pcolormesh(bins, freqs, Pxx_dB, shading='auto', cmap='viridis', vmin=vmin, vmax=max_db)
                ax_spec.set_xlabel('Time (s)')
                ax_spec.set_ylabel('Frequency (Hz)')
                ax_spec.set_title('Encoder spectrogram (dB)')
                cb = spec_fig.colorbar(im, ax=ax_spec)
                cb.set_label('Power (dB)')
                spec_path = os.path.join(out_dir, f"MoveMotorPlot_{timestamp}_encoder_spectrogram.svg")
                spec_fig.tight_layout()
                plt.savefig(spec_path, format='svg')
                print(f"Saved encoder spectrogram to: {spec_path}")
            except Exception as e:
                print(f"Failed to create spectrogram: {e}")
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

    cmd_times, cmd_vals, loadcell_times, loadcell_vals, encoder_times, encoder_vals, csv = read_from_serial(port)
    # plot_data expects (telemetry_times, telemetry_vals, encoder_times, encoder_vals, gnd_times, gnd_vals, csv)
    # we map load-cell -> telemetry slot and cmd -> gnd slot for plotting
    plot_data(loadcell_times, loadcell_vals, encoder_times, encoder_vals, cmd_times, cmd_vals, csv)


if __name__ == '__main__':
    main()