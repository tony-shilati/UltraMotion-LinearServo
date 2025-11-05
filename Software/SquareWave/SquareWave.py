#!/usr/bin/env python3
"""SquareWave serial reader

Connect to a Teensy 4.1 over USB serial, read telemetry lines formatted as
  telemetry, time
Collect samples until no message is received for 3 seconds (inactivity
timeout). After the timeout the script saves a CSV and plots telemetry vs time
and saves an SVG of the plot.

Usage:
  python SquareWave.py [serial_port]

If serial_port is not supplied the script will try to auto-detect a USB
serial device.
"""

import serial
import serial.tools.list_ports
import time
import os
import sys
import csv
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

# Configuration
BAUD = 115200
INACTIVITY_TIMEOUT = 5.0


def find_port():
	ports = list(serial.tools.list_ports.comports())
	for p in ports:
		desc = (p.description or '').lower()
		if 'teensy' in desc or 'usb' in desc:
			return p.device
	return None


def ensure_outdir():
	out = os.path.join(os.path.dirname(__file__), 'outputs')
	os.makedirs(out, exist_ok=True)
	return out


def parse_line(line):
	"""Parse a line in one of these forms:
	  T:<telemetry>,<time>
	  G:<reading>,<time>
	or a fallback comma/whitespace-separated pair.

	Returns (msg_type: 'T'|'G', value: float, time: float) or (None, None, None)
	on failure.
	"""
	if not line:
		return None, None, None
	s = line.strip()
	if not s:
		return None, None, None

	msg_type = None
	payload = s
	# detect prefix
	if len(s) >= 2 and s[1] == ':' and s[0].upper() in ('T', 'G', 'E'):
		msg_type = s[0].upper()
		payload = s[2:]

	# allow comma or whitespace separated
	parts = [p.strip() for p in payload.replace('\t', ' ').replace(';', ',').split(',') if p.strip()]
	if len(parts) == 1:
		parts = payload.split()
	if len(parts) < 2:
		return None, None, None
	try:
		val = float(parts[0])
		t = float(parts[1])
		# if no explicit prefix, assume telemetry (T)
		if msg_type is None:
			msg_type = 'T'
		return msg_type, val, t
	except ValueError:
		return None, None, None


def normalize_times(parsed_times, arrival_times):
	"""Convert parsed times to relative seconds or fall back to arrival times."""
	if len(parsed_times) == 0:
		return np.array([])
	parsed = np.array(parsed_times, dtype=float)
	arrival = np.array(arrival_times, dtype=float)
	if np.ptp(parsed) < 1e-9:
		return arrival - arrival[0]
	maxv = np.nanmax(parsed)
	if not np.isfinite(maxv):
		return arrival - arrival[0]
	# crude unit detection
	if maxv > 1e12:
		parsed_s = parsed / 1e9
	elif maxv > 1e9:
		parsed_s = parsed
	elif maxv > 1e6:
		parsed_s = parsed / 1e6
	elif maxv > 1e3:
		parsed_s = parsed / 1e3
	else:
		parsed_s = parsed
	return parsed_s - parsed_s[0]


def main():
	port = None
	if len(sys.argv) > 1:
		port = sys.argv[1]
	else:
		port = find_port()

	if port is None:
		print("No serial port specified and no suitable device auto-detected.")
		print("Available ports:")
		for p in serial.tools.list_ports.comports():
			print(f"  {p.device}: {p.description}")
		port = input("Enter serial port: ")

	print(f"Opening {port} @ {BAUD} baud...")
	try:
		ser = serial.Serial(port, BAUD, timeout=1.0)
	except Exception as e:
		print(f"Failed to open {port}: {e}")
		sys.exit(1)

	outdir = ensure_outdir()
	ts = datetime.now().strftime('%Y%m%d_%H%M%S')
	csv_path = os.path.join(outdir, f"SquareWaveTelemetry_{ts}.csv")

	# separate storage for T, G and E messages
	vals_T = []
	times_parsed_T = []
	times_arrival_T = []

	vals_G = []
	times_parsed_G = []
	times_arrival_G = []

	vals_E = []
	times_parsed_E = []
	times_arrival_E = []

	last = time.time()
	print(f"Reading (timeout after {INACTIVITY_TIMEOUT}s of inactivity)...")
	try:
		with open(csv_path, 'w', newline='') as f:
			writer = csv.writer(f)
			writer.writerow(['type', 'value', 'time'])
			while True:
				try:
					raw = ser.readline()
				except (serial.SerialException, OSError) as e:
					print(f"Serial read error: {e}")
					break
				if not raw:
					if (time.time() - last) >= INACTIVITY_TIMEOUT:
						print('Inactivity timeout')
						break
					continue
				last = time.time()
				arrival = last
				try:
					line = raw.decode('utf-8', errors='replace').strip()
				except Exception:
					line = str(raw)
				msg_type, val, t = parse_line(line)
				if msg_type is None:
					# ignore malformed lines
					continue
				# record into appropriate series and write raw CSV with type
				writer.writerow([msg_type, f"{val:.4f}", f"{t:.4f}"])
				if msg_type == 'T':
					vals_T.append(val)
					times_parsed_T.append(t)
					times_arrival_T.append(arrival)
				elif msg_type == 'G':
					vals_G.append(val)
					times_parsed_G.append(t)
					times_arrival_G.append(arrival)
				elif msg_type == 'E':
					vals_E.append(val)
					times_parsed_E.append(t)
					times_arrival_E.append(arrival)
				total = len(vals_T) + len(vals_G) + len(vals_E)
				if total > 0 and (total % 50) == 0:
					print(f"Captured {total} samples")
	except KeyboardInterrupt:
		print('Interrupted by user')
	finally:
		ser.close()

	print(f"Saved CSV to {csv_path}")

	t_rel_T = normalize_times(times_parsed_T, times_arrival_T)
	t_rel_G = normalize_times(times_parsed_G, times_arrival_G)
	t_rel_E = normalize_times(times_parsed_E, times_arrival_E)

	if len(t_rel_T) == 0 and len(t_rel_G) == 0 and len(t_rel_E) == 0:
		print('No data captured')
		return

	# Plot telemetry and G on the same axis, and E on a separate right-hand y-axis
	fig, ax = plt.subplots(figsize=(10, 4))
	if len(t_rel_T) > 0:
		ax.plot(t_rel_T, vals_T, 'b.-', label='Telemetry (T)')
	if len(t_rel_G) > 0:
		ax.plot(t_rel_G, vals_G, 'g.-', label='G-reading (G)')

	ax.set_xlabel('Time (s)')
	ax.set_ylabel('Telemetry / G-reading')
	ax.grid(True, alpha=0.3)
	plt.title('Telemetry (T), G-reading (G), and Position (E) vs Time')

	# Plot E on a separate axis
	ax_e = ax.twinx()
	if len(t_rel_E) > 0:
		ax_e.plot(t_rel_E, vals_E, 'r.-', label='Position (E)')
		ax_e.set_ylabel('Position (E)', color='r')
		ax_e.tick_params(axis='y', labelcolor='r')

	# Combined legend: gather handles from both axes
	handles = []
	labels = []
	for a in (ax, ax_e):
		h, l = a.get_legend_handles_labels()
		handles += h; labels += l
	if handles:
		ax.legend(handles, labels)

	svg_path = os.path.join(outdir, f"SquareWavePlot_{ts}.svg")
	plt.tight_layout()
	plt.savefig(svg_path, format='svg')
	print(f"Saved plot to {svg_path}")
	plt.show()


if __name__ == '__main__':
	main()

