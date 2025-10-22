## Import libraries for serial comm and fft
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from datetime import datetime
import csv
import os
import sys
import pickle

## Define a timeout to be 1 second
SERIAL_TIMEOUT = 2.0
BAUD_RATE = 115200
MAX_FREQUENCY = 100

def find_teensy_port():
    """Find the Teensy serial port automatically"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'teensy' in port.description.lower() or 'usb' in port.description.lower():
            return port.device
    # If no Teensy found, return None (user will need to specify manually)
    return None

def ensure_output_dir():
    """Create outputs directory if it doesn't exist"""
    output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    return output_dir

## Open a serial port and start reading and storing the data (position (ticks), time (seconds)) 
## as well as prints the data to a date and timestamped csv in ../outputs
def read_serial_data(port=None):
    """Read position and time data from serial port"""
    if port is None:
        port = find_teensy_port()
        if port is None:
            print("Could not find Teensy. Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}: {p.description}")
            port = input("Enter port name (e.g., /dev/ttyACM0 or COM3): ")
    
    print(f"Connecting to {port} at {BAUD_RATE} baud...")
    
    # Prepare CSV file with timestamp
    output_dir = ensure_output_dir()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = os.path.join(output_dir, f"telemetry_{timestamp}.csv")
    
    positions = []
    times = []
    
    try:
        with serial.Serial(port, BAUD_RATE, timeout=SERIAL_TIMEOUT) as ser:
            print(f"Connected! Reading data... (timeout: {SERIAL_TIMEOUT}s)")
            print(f"Saving to: {csv_filename}")
            
            with open(csv_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time (s)', 'Position (ticks)'])
                
                start_time = None
                
                ## When a message hasn't been received for a while the serial port times out
                while True:
                    try:
                        line = ser.readline().decode('utf-8').strip()
                        
                        if not line:
                            print("\nSerial timeout - no data received.")
                            break
                        
                        # Try to parse the line as position data
                        # Assuming format: "position,time" or "position, time"
                        try:
                            # Check if line contains comma (position,time format)
                            if ',' in line:
                                parts = line.split(',')
                                position = int(parts[0].strip())
                                current_time = float(parts[1].strip())
                            # Check if space-separated (position time or time position)
                            elif ' ' in line:
                                parts = line.split()
                                # Try position,time order first
                                try:
                                    position = int(parts[0])
                                    current_time = float(parts[1])
                                except (ValueError, IndexError):
                                    # If that fails, skip this line
                                    continue
                            else:
                                # Only position provided, calculate time
                                position = int(line.strip())
                                if start_time is None:
                                    start_time = datetime.now()
                                current_time = (datetime.now() - start_time).total_seconds()
                            
                            times.append(current_time)
                            positions.append(position)
                            writer.writerow([current_time, position])
                            
                            # Print progress
                            if len(times) % 10 == 0:
                                print(f"  Samples: {len(times)}, Last: t={current_time:.3f}s, pos={position}")
                        
                        except (ValueError, IndexError):
                            # Line doesn't contain valid data, skip it
                            print(f"  Skipping: {line}")
                            continue
                    
                    except serial.SerialTimeoutException:
                        print("\nSerial timeout - stopping data collection.")
                        break
    
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    
    print(f"\nData collection complete. Collected {len(times)} samples.")
    return np.array(times), np.array(positions), csv_filename

## Once the serial port has timed out an fft is taken of the data and is plotted from 0-50 hz
def perform_fft_and_plot(times, positions, csv_filename):
    """Perform FFT on position data and plot frequency spectrum from 0-50 Hz"""
    if len(times) < 2:
        print("Not enough data for FFT analysis.")
        return
    
    # Calculate sampling rate
    dt = np.mean(np.diff(times))
    fs = 1.0 / dt
    print(f"\nSampling rate: {fs:.2f} Hz (dt={dt*1000:.2f} ms)")
    
    # Remove DC component (mean)
    positions_centered = positions - np.mean(positions)
    
    # Compute FFT
    N = len(positions_centered)
    fft_vals = np.fft.fft(positions_centered)
    fft_freq = np.fft.fftfreq(N, dt)
    
    # Get positive frequencies only
    positive_freq_idx = fft_freq > 0
    frequencies = fft_freq[positive_freq_idx]
    magnitudes = np.abs(fft_vals[positive_freq_idx]) / N  # Normalize
    
    # Filter to 0-MAX_FREQUENCY Hz range
    freq_mask = (frequencies >= 0) & (frequencies <= MAX_FREQUENCY)
    frequencies_plot = frequencies[freq_mask]
    magnitudes_plot = magnitudes[freq_mask]
    
    ## The plot is saved as a date and timestamped file in ../outputs
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.dirname(csv_filename)
    plot_filename = os.path.join(output_dir, f"fft_plot_{timestamp}.svg")
    
    # Create plot
    fig = plt.figure(figsize=(12, 8))
    
    # Subplot 1: Time domain
    plt.subplot(2, 1, 1)
    plt.plot(times, positions, 'b-', linewidth=0.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (ticks)')
    plt.title('Position vs Time')
    plt.grid(True, alpha=0.3)
    
    # Subplot 2: Frequency domain (0-25 Hz)
    plt.subplot(2, 1, 2)
    plt.plot(frequencies_plot, magnitudes_plot, 'r-', linewidth=1)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.title('FFT: Frequency Spectrum (0-25 Hz)')
    plt.xlim(0, MAX_FREQUENCY)
    # Compute tick positions (use 6 ticks from 0 to MAX_FREQUENCY to avoid non-integer step errors)
    ticks = np.linspace(0, MAX_FREQUENCY, 3)
    plt.xticks(ticks)  # Ticks from 0 to MAX_FREQUENCY inclusive
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    # Save as vector graphics (SVG format)
    plt.savefig(plot_filename, format='svg', dpi=150)
    print(f"Vector graphics plot saved to: {plot_filename}")
    
    # Show plot
    plt.show()
    
    # Print dominant frequencies
    if len(magnitudes_plot) > 0:
        peak_indices = signal.find_peaks(magnitudes_plot, height=np.max(magnitudes_plot)*0.1)[0]
        if len(peak_indices) > 0:
            print("\nDominant frequencies:")
            for idx in peak_indices[:5]:  # Show top 5
                print(f"  {frequencies_plot[idx]:.2f} Hz (magnitude: {magnitudes_plot[idx]:.2f})")

def main():
    """Main function"""
    print("=== Telemetry Reader and FFT Analyzer ===\n")
    
    # Read serial data
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    times, positions, csv_filename = read_serial_data(port)
    
    if len(times) > 0:
        # Perform FFT and plot
        perform_fft_and_plot(times, positions, csv_filename)
    else:
        print("No data collected.")

if __name__ == "__main__":
    main()
