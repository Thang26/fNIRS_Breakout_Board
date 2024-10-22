import serial
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

# Serial port settings
SERIAL_PORT = 'COM4'
BAUD_RATE = 115200
TIMEOUT = 1

# Packet settings
HEADER_SIZE = 4
FOOTER_SIZE = 4
DATA_SIZE = 80  # 40 samples * 2 bytes each
PACKET_SIZE = HEADER_SIZE + DATA_SIZE + FOOTER_SIZE  # Total packet size

# Create packet header and footer using little-endian format
PACKET_HEADER = struct.pack('<I', 0xFFFFFFFF)
PACKET_FOOTER = struct.pack('<I', 0xDEADBEEF)

# Open serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

# Data buffer and lock
data_buffer = []
data_lock = threading.Lock()
serial_buffer = bytearray()

def read_serial():
    global data_buffer, serial_buffer
    while True:
        try:
            # Read incoming data
            data = ser.read(ser.in_waiting or 1)
            if data:
                serial_buffer.extend(data)
                # Debugging output
                # print(f"Received data: {data.hex()}")
                while True:
                    # Look for the header
                    header_index = serial_buffer.find(PACKET_HEADER)
                    if header_index == -1:
                        # Header not found, keep last few bytes
                        if len(serial_buffer) > (HEADER_SIZE - 1):
                            serial_buffer = serial_buffer[-(HEADER_SIZE - 1):]
                        break
                    else:
                        # Check if enough data is available for a full packet
                        if len(serial_buffer) >= header_index + PACKET_SIZE:
                            # Extract the packet
                            packet = serial_buffer[header_index:header_index + PACKET_SIZE]
                            # Remove processed data from the buffer
                            serial_buffer = serial_buffer[header_index + PACKET_SIZE:]
                            # Verify the footer
                            if packet[-FOOTER_SIZE:] == PACKET_FOOTER:
                                # Extract ADC data
                                adc_data = packet[HEADER_SIZE:-FOOTER_SIZE]
                                # Unpack ADC samples
                                samples = struct.unpack('<40H', adc_data)
                                with data_lock:
                                    data_buffer.extend(samples)
                                # Debugging output
                                # print(f"Packet received and processed. Samples: {samples}")
                            else:
                                # Footer mismatch, discard data up to next header
                                serial_buffer = serial_buffer[header_index + 1:]
                                print("Footer mismatch. Discarding data up to next header.")
                                break
                        else:
                            # Not enough data yet, wait for more
                            if header_index > 0:
                                serial_buffer = serial_buffer[header_index:]
                            break
        except serial.SerialException as e:
            print(f"Serial exception: {e}")
            break

# Start the serial reading thread
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True
serial_thread.start()

# Set up the plot
fig, ax = plt.subplots()
line, = ax.plot([], [])
ax.set_ylim(0, 65535)  # Adjust based on ADC resolution
ax.set_xlabel('Sample Number')
ax.set_ylabel('ADC Value')
ax.set_title('Live ADC Data from STM32H723ZG')

def update(frame):
    global data_buffer
    with data_lock:
        ydata = data_buffer.copy()
    xdata = list(range(len(ydata)))
    line.set_data(xdata, ydata)
    ax.set_xlim(0, max(len(ydata), 1000))
    # Limit the displayed data to the last 1000 samples
    if len(ydata) > 1000:
        with data_lock:
            data_buffer = data_buffer[-1000:]
    return line,

# Animate the plot
ani = animation.FuncAnimation(fig, update, interval=100)
plt.show()
