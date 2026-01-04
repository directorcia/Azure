#!/usr/bin/env python3
"""
ARDUCAM IMAGE CAPTURE SCRIPT

This Python script communicates with an Arduino board running the Arducam Mega 3MP
camera sketch. It:
1. Establishes serial communication with the Arduino
2. Sends capture commands at various resolutions
3. Receives binary JPEG image data from the camera
4. Validates image integrity (JPEG markers)
5. Saves images with timestamps to the local filesystem

Requirements:
- Python 3.6+
- pyserial library (install with: pip install pyserial)
- Arduino board running the Arducam Mega capture sketch
- USB connection between computer and Arduino

Usage:
    python capture-image.py
    Enter commands when prompted:
    - 'c' for VGA capture
    - '1' for QVGA capture
    - '4' for 3MP capture
    - 'q' to quit
"""

import serial      # Library for serial port communication
import time        # Library for timing operations
import os          # Library for file and directory operations
from datetime import datetime  # Library for timestamp generation

# ============================================================================
# CONFIGURATION SECTION
# ============================================================================
# Modify these values to match your hardware setup

COM_PORT = "COM6"              # Serial port where Arduino is connected
                               # Windows: COM1-COM9, Linux: /dev/ttyUSB0, macOS: /dev/tty.usbserial*
BAUD_RATE = 115200             # Serial communication speed in bits per second
                               # Must match the Arduino sketch (115200)
OUTPUT_DIR = "captured_images" # Directory where captured JPEG images will be saved

# ============================================================================
# FUNCTION: ensure_output_dir()
# ============================================================================
def ensure_output_dir():
    """
    Create the output directory for storing captured images if it doesn't exist.
    
    This function checks if the OUTPUT_DIR directory exists. If it doesn't,
    it creates the directory (and any parent directories if needed).
    This is a safety measure to prevent file write errors.
    
    Returns:
        None
    """
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Created output directory: {OUTPUT_DIR}")


# ============================================================================
# FUNCTION: connect_to_board()
# ============================================================================
def connect_to_board(port, baudrate):
    """
    Establish serial connection with the Arduino board.
    
    Opens a serial port connection to communicate with the Arduino. This function:
    1. Attempts to open the specified COM port at the given baud rate
    2. Waits 2 seconds for the Arduino to initialize (important for boards like Leonardo)
    3. Returns the serial connection object if successful
    4. Returns None and prints error message if connection fails
    
    Parameters:
        port (str): Serial port name (e.g., "COM6", "/dev/ttyUSB0")
        baudrate (int): Communication speed in bits per second (e.g., 115200)
    
    Returns:
        serial.Serial: Connection object if successful, None if failed
    
    Raises:
        Exception: Caught internally and reported to user
    """
    try:
        # Create serial connection with 5-second timeout for read operations
        ser = serial.Serial(port, baudrate, timeout=5)
        
        # Wait 2 seconds for Arduino to initialize after connection
        # This is especially important for Arduino Leonardo which has USB bootloader
        time.sleep(2)
        
        # Display success message
        print(f"✓ Connected to {port} at {baudrate} baud")
        return ser
        
    except Exception as e:
        # Print error if connection fails
        # Common errors: port doesn't exist, already in use, permissions denied
        print(f"✗ Failed to connect: {e}")
        return None


# ============================================================================
# FUNCTION: send_command()
# ============================================================================
def send_command(ser, cmd):
    """
    Send a command character to the Arduino board.
    
    This function sends a single character command through the serial connection
    to tell the Arduino what to do. The Arduino's loop() function reads this
    character and processes it.
    
    Parameters:
        ser (serial.Serial): The serial connection object
        cmd (str): Single character command ('c', '1', '2', '3', '4', 'q', etc.)
    
    Returns:
        None
    
    Side Effects:
        Writes data to serial port, prints confirmation message
    """
    # Encode the string to bytes and send over serial connection
    # .encode() converts string to bytes format for transmission
    ser.write(cmd.encode())
    
    # Display confirmation showing what command was sent
    print(f"Sent command: '{cmd}'")

# ============================================================================
# FUNCTION: capture_image()
# ============================================================================
def capture_image(ser, resolution='c', timeout=120):
    """
    Capture an image from the Arduino camera and receive the binary data.
    
    This is the core function that:
    1. Sends a capture command to the Arduino
    2. Waits for the size information from the Arduino
    3. Waits for the "START_BINARY_DATA" marker
    4. Reads all the binary JPEG data byte-by-byte
    5. Monitors for JPEG end marker (0xFF 0xD9)
    6. Shows progress during transmission
    
    The Arduino sends data in this format:
        - Human-readable text messages
        - SIZE:[number]
        - START_BINARY_DATA
        - [Binary JPEG image data]
        - END_IMAGE_DATA
    
    Parameters:
        ser (serial.Serial): The serial connection object
        resolution (str): Resolution command character ('c', '1', '2', '3', '4')
        timeout (int): Maximum seconds to wait before giving up (default: 120)
    
    Returns:
        bytes: The complete binary JPEG image data, or empty bytes if failed
    
    Raises:
        None (errors are handled internally)
    """
    
    # Display capture header
    print("\n" + "="*40)
    print("Capturing image...")
    print("="*40)
    
    # Send the capture command to Arduino (e.g., 'c', '1', '2', '3', or '4')
    send_command(ser, resolution)
    
    # --------
    # PHASE 1: Wait for size marker and binary start marker
    # --------
    print("Waiting for image size...")
    start_time = time.time()          # Record when we started waiting
    buffer = ""                       # String buffer to accumulate received text
    image_size = 0                    # Will store the size sent by Arduino
    found_start_marker = False        # Flag for when we find START_BINARY_DATA
    
    # Loop until we get size info and start marker, or timeout
    while time.time() - start_time < timeout:
        
        # Check if any data has arrived from Arduino
        if ser.in_waiting:
            # Read one character (as bytes, then decode to string)
            char = ser.read(1).decode('utf-8', errors='ignore')
            buffer += char
            
            # Check if buffer contains the SIZE marker
            if "SIZE:" in buffer:
                try:
                    # Extract the number after "SIZE:"
                    # Example: "SIZE:47834" → extract "47834"
                    size_line = buffer.split("SIZE:")[1].split('\n')[0].strip()
                    image_size = int(size_line)
                    print(f"✓ Image size: {image_size} bytes ({image_size/1024:.1f} KB)")
                except:
                    # Silently continue if parsing fails
                    pass
            
            # Check if buffer contains the binary data start marker
            if "START_BINARY_DATA" in buffer:
                found_start_marker = True
                print("✓ Starting binary read...")
                break  # Exit loop to start reading binary data
            
            # Keep only the last 100 characters to prevent buffer from growing too large
            # This saves memory while still keeping recent data
            if len(buffer) > 100:
                buffer = buffer[-100:]
                
            # Print human-readable messages from Arduino (skip metadata lines)
            if '\n' in char:
                line = buffer.strip()
                # Only print if it's not a SIZE, START, or SET line (metadata)
                if line and "SIZE" not in line and "START" not in line and "SET" not in line:
                    print(line)
                buffer = ""
    
    # Check if we successfully got the required information
    if image_size == 0 or not found_start_marker:
        print("✗ Failed to get image size or start marker")
        return b''  # Return empty bytes to indicate failure
    
    # --------
    # PHASE 2: Read binary image data
    # --------
    print("Reading image data...")
    
    # Skip any trailing whitespace (newlines, spaces) after START_BINARY_DATA marker
    # This ensures the first byte we read is actual image data
    time.sleep(0.1)  # Brief delay to let Arduino finish sending markers
    
    first_byte = b''
    # Read and discard whitespace until we find the first data byte
    while ser.in_waiting:
        byte = ser.read(1)
        if byte not in b'\r\n \t':  # If not whitespace
            first_byte = byte  # This is the first real data byte
            break
    
    # Initialize image_data and read counter
    image_data = b''           # Will accumulate all JPEG bytes
    if first_byte:
        image_data = first_byte  # Add the first data byte we found
    
    bytes_read = len(image_data)
    last_two = image_data[-2:] if len(image_data) >= 2 else b''
    
    # Loop: read bytes until we have all image data or timeout
    while bytes_read < image_size and time.time() - start_time < timeout:
        
        if ser.in_waiting:
            # Read one byte from serial connection
            byte = ser.read(1)
            image_data += byte  # Append to our image data
            bytes_read += 1
            
            # Keep track of the last two bytes for JPEG end marker detection
            # JPEG files always end with: 0xFF 0xD9
            last_two += byte
            if len(last_two) > 2:
                last_two = last_two[-2:]  # Keep only last 2 bytes
            
            # Check if we found the JPEG end marker (0xFF 0xD9)
            # If found, we can stop early rather than waiting for all expected bytes
            if last_two == b'\xff\xd9':
                print(f"\n✓ Found JPEG end marker at {bytes_read} bytes")
                break  # Stop reading, we've got the complete image
            
            # Show progress indicator every 5KB of data read
            # '\r' returns cursor to start of line without newline (updates in place)
            if bytes_read % 5120 == 0:
                pct = min(100, 100*bytes_read/image_size)
                print(f"  {bytes_read}/{image_size} bytes ({pct:.0f}%)", end='\r')
    
    # Print final progress (overwrites the progress line)
    print(f"  {bytes_read}/{image_size} bytes (100%)     ")
    
    # Warn if we didn't get the exact number of bytes expected
    if bytes_read != image_size:
        print(f"⚠ Note: Expected {image_size} bytes but got {bytes_read}")
    
    # Return the complete binary image data
    return image_data

# ============================================================================
# FUNCTION: save_image()
# ============================================================================
def save_image(image_data):
    """
    Save the captured binary image data to a JPEG file.
    
    This function:
    1. Validates that image data was received
    2. Checks for valid JPEG markers (0xFF 0xD8 at start, 0xFF 0xD9 at end)
    3. Generates a timestamped filename
    4. Writes the binary data to a JPEG file
    5. Reports success or failure
    
    JPEG Format:
    - JPEG files always start with: 0xFF 0xD8 (Start of Image marker)
    - JPEG files always end with: 0xFF 0xD9 (End of Image marker)
    - If these markers are missing or wrong, the file may be corrupted
    
    Parameters:
        image_data (bytes): Binary JPEG image data received from Arduino
    
    Returns:
        str: Full path to saved file if successful, None if failed
    
    Raises:
        None (errors are handled internally)
    """
    
    # Check if we actually received any image data
    if not image_data:
        print("✗ No image data to save!")
        return None
    
    # --------
    # Validate JPEG format by checking start marker
    # --------
    if len(image_data) >= 2:
        # Display first two bytes in hexadecimal format
        print(f"✓ First 2 bytes: 0x{image_data[0]:02X} 0x{image_data[1]:02X}")
        
        # JPEG files must start with 0xFF 0xD8 (SOI = Start of Image)
        if image_data[0] == 0xFF and image_data[1] == 0xD8:
            print("✓ Valid JPEG start marker (0xFF 0xD8)")
        else:
            print("⚠ Warning: Invalid JPEG start marker!")
    
    # --------
    # Validate JPEG format by checking end marker
    # --------
    if len(image_data) >= 2:
        # Display last two bytes in hexadecimal format
        print(f"✓ Last 2 bytes: 0x{image_data[-2]:02X} 0x{image_data[-1]:02X}")
        
        # JPEG files must end with 0xFF 0xD9 (EOI = End of Image)
        if image_data[-2] == 0xFF and image_data[-1] == 0xD9:
            print("✓ Valid JPEG end marker (0xFF 0xD9)")
        else:
            print("⚠ Warning: Invalid JPEG end marker!")
    
    # --------
    # Generate timestamped filename
    # --------
    # Get current date and time (e.g., "20260104_143025" for Jan 4, 2026 at 14:30:25)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Create full file path: output_directory/image_timestamp.jpg
    # Example: "captured_images/image_20260104_143025.jpg"
    filename = os.path.join(OUTPUT_DIR, f"image_{timestamp}.jpg")
    
    # --------
    # Write image data to file
    # --------
    try:
        # Open file in binary write mode ('wb')
        with open(filename, 'wb') as f:
            # Write all the binary image data to the file
            f.write(image_data)
        
        # Report success with file details
        print(f"✓ Image saved: {filename}")
        print(f"  Size: {len(image_data)} bytes ({len(image_data)/1024:.1f} KB)")
        return filename
        
    except Exception as e:
        # Report error if file write fails
        # Common errors: permission denied, disk full, path doesn't exist
        print(f"✗ Failed to save image: {e}")
        return None

# ============================================================================
# FUNCTION: main()
# ============================================================================
def main():
    """
    Main program entry point.
    
    This function:
    1. Displays welcome banner
    2. Ensures output directory exists
    3. Connects to the Arduino board
    4. Displays available commands to the user
    5. Reads user commands in a loop
    6. Processes capture commands and saves images
    7. Handles graceful shutdown
    
    User Interaction:
    - Prompts user for command input
    - Accepts: 'c', '1', '2', '3', '4' for capture, 'q' to quit
    - Exits on 'q' or Ctrl+C (KeyboardInterrupt)
    
    Returns:
        None
    """
    
    # Display welcome banner
    print("\n╔════════════════════════════════════╗")
    print("║   ArduCAM Image Capture Script     ║")
    print("╚════════════════════════════════════╝\n")
    
    # Create output directory if needed
    ensure_output_dir()
    
    # --------
    # Connect to Arduino board
    # --------
    ser = connect_to_board(COM_PORT, BAUD_RATE)
    if not ser:
        # If connection failed, exit the program
        return
    
    try:
        # Wait for Arduino initialization (especially important for Leonardo)
        time.sleep(2)
        print("\nBoard is ready!")
        
        # Display available commands to user
        print("Commands:")
        print("  'c' - Capture VGA (640x480)")
        print("  '1' - Capture QVGA (320x240)")
        print("  '2' - Capture VGA (640x480)")
        print("  '3' - Capture 1080p")
        print("  '4' - Capture 3MP (2048x1536)")
        print("  'q' - Quit\n")
        
        # --------
        # Main interactive loop
        # --------
        # Loop continuously until user types 'q' to quit
        while True:
            # Prompt user and read command (convert to lowercase, remove whitespace)
            cmd = input("Enter command: ").strip().lower()
            
            # Check if user wants to quit
            if cmd == 'q':
                break
            
            # Check if the command is valid (one of the capture commands)
            elif cmd in ['c', '1', '2', '3', '4']:
                # Capture an image using the specified resolution
                image_data = capture_image(ser, cmd)
                
                # If capture was successful (got data), save it to a file
                if image_data:
                    save_image(image_data)
            else:
                # Command not recognized
                print("Invalid command!")
    
    except KeyboardInterrupt:
        # User pressed Ctrl+C to interrupt the program
        print("\n\nInterrupted by user")
    
    finally:
        # Always execute this block when exiting (normal or error)
        # Close the serial connection to release the port
        ser.close()
        print("✓ Disconnected")

# ============================================================================
# SCRIPT ENTRY POINT
# ============================================================================
if __name__ == "__main__":
    """
    This block ensures main() only runs when script is executed directly,
    not when imported as a module in another script.
    
    __name__ is automatically set to "__main__" when Python runs a script,
    but set to the module name when imported elsewhere.
    """
    main()
