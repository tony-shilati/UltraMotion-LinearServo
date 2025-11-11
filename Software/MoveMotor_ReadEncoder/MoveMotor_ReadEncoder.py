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
from math import ceil
import matplotlib.pyplot as plt


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

    # New behavior: capture 'G:data,time' and 'S:data1,data2,time' messages
    # and save them to CSV with columns [G, G_time, S1, S2, S_time].
    def read_GS_and_save_csv(port=None, baud=115200, inactivity_timeout=5.0):
        opened_here = False
        if port is None:
            # attempt to auto-detect a Teensy/USB serial device
            ports = list(serial.tools.list_ports.comports())
            found = None
            for p in ports:
                desc = (p.description or '').lower()
                if 'teensy' in desc or 'usb' in desc:
                    found = p.device
                    break
            if found is None:
                print("No serial port specified and no Teensy-like device auto-detected.\nAvailable ports:")
                for p in ports:
                    print(f"  {p.device}: {p.description}")
                raise SystemExit(1)
            port = found

        print(f"Opening port {port} @ {baud} baud")
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=1.0)
            opened_here = True
        except Exception as e:
            print(f"Failed to open port {port}: {e}")
            raise SystemExit(1)

        out_dir = os.path.join(os.getcwd(), 'outputs')
        os.makedirs(out_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(out_dir, f"MoveMotor_GS_{timestamp}.csv")

        g_times = []
        g_vals = []
        s1_vals = []
        s2_vals = []
        s_times = []

        last_recv = time.time()
        print(f"Reading G/S lines (timeout after {inactivity_timeout}s inactivity)...")
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
                    g_vals.append(val)
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

                # ignore other prefixes here

        except KeyboardInterrupt:
            print('\nInterrupted by user, stopping capture.')
        finally:
            if opened_here:
                try:
                    ser.close()
                except Exception:
                    pass

        # Save CSV with columns: [G, G_time, S1, S2, S_time]
        try:
            with open(csv_path, 'w', newline='') as csvfile:
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

        print(f"Saved GS CSV to: {csv_path}")
        return (np.array(g_times), np.array(g_vals), np.array(s1_vals), np.array(s2_vals), np.array(s_times), csv_path)

    g_times, g_vals, s1_vals, s2_vals, s_times, csv = read_GS_and_save_csv(port)
    print('Done. CSV saved to:', csv)


if __name__ == '__main__':
    main()