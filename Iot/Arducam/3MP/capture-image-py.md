# ArduCAM Image Capture Script - Python Documentation

## Overview

The **ArduCAM Image Capture Script** (`capture-image.py`) is a Python 3 application that communicates with an Arduino board running the Arducam Mega 3MP camera sketch. It provides an interactive interface for capturing images at multiple resolutions and automatically saves them to disk with proper validation and timestamping.

### Key Features
- **Interactive Command Interface** - User-friendly serial command input
- **Multiple Resolution Support** - QVGA, VGA, FHD, and 3MP resolutions
- **Binary Data Handling** - Proper reception and validation of JPEG binary data
- **JPEG Validation** - Verification of start and end markers
- **Progress Monitoring** - Real-time feedback during image transmission
- **Timestamped Filenames** - Automatic naming with date/time stamps
- **Error Handling** - Comprehensive error detection and reporting
- **Cross-Platform** - Works on Windows, Linux, and macOS

---

## System Requirements

### Hardware
- **Computer** - Windows, macOS, or Linux system
- **Arduino Board** - Arduino Leonardo, Micro, or compatible
- **Arducam Mega 3MP** - Camera module connected to Arduino
- **USB Cable** - For serial communication between computer and Arduino

### Software
- **Python 3.6 or later** - Required for script execution
- **pyserial** - Python library for serial communication

### Internet Connection
- **Installation time only** - For downloading Python packages

---

## Installation Guide

### Step 1: Install Python 3

#### Windows
1. Download from [python.org](https://www.python.org/downloads/)
2. Run installer and check "Add Python to PATH"
3. Click "Install Now"
4. Verify installation:
   ```powershell
   python --version
   ```

#### macOS
1. Using Homebrew:
   ```bash
   brew install python3
   ```
2. Or download from python.org

#### Linux
```bash
sudo apt-get update
sudo apt-get install python3 python3-pip
```

### Step 2: Install pyserial Library

#### All Platforms
```bash
pip install pyserial
```

Or with pip3:
```bash
pip3 install pyserial
```

Verify installation:
```bash
python -m serial --version
```

### Step 3: Locate and Verify Script

1. Copy `capture-image.py` to your working directory
2. Verify file exists:
   ```bash
   ls capture-image.py          # Linux/macOS
   dir capture-image.py         # Windows PowerShell
   ```

---

## Configuration

### Before Running the Script

Edit the configuration section at the top of `capture-image.py`:

```python
# ============================================================================
# CONFIGURATION SECTION
# ============================================================================
COM_PORT = "COM6"              # Serial port
BAUD_RATE = 115200             # Communication speed
OUTPUT_DIR = "captured_images" # Output directory
```

### Finding Your Serial Port

#### Windows
1. Connect Arduino via USB
2. Open Device Manager (Win + X → Device Manager)
3. Expand "Ports (COM & LPT)"
4. Look for "Arduino Leonardo" or similar
5. Note the COM port (e.g., COM6)
6. Update script: `COM_PORT = "COM6"`

#### macOS
```bash
ls /dev/tty.usb*
# Typically: /dev/tty.usbserial-XXXXXXXXX
```
Update script: `COM_PORT = "/dev/tty.usbserial-XXXXXXXXX"`

#### Linux
```bash
ls /dev/ttyUSB*
# Typically: /dev/ttyUSB0 or /dev/ttyUSB1
```
Update script: `COM_PORT = "/dev/ttyUSB0"`

### Configuration Options

| Parameter | Default | Purpose | Notes |
|-----------|---------|---------|-------|
| `COM_PORT` | "COM6" | Serial port | Must match Arduino connection |
| `BAUD_RATE` | 115200 | Communication speed | Must match Arduino sketch |
| `OUTPUT_DIR` | "captured_images" | Save location | Created automatically if missing |

---

## Usage Guide

### Starting the Script

#### Windows PowerShell
```powershell
python capture-image.py
```

#### macOS/Linux Terminal
```bash
python3 capture-image.py
```

### Initial Output

When script starts successfully, you'll see:
```
╔════════════════════════════════════╗
║   ArduCAM Image Capture Script     ║
╚════════════════════════════════════╝

Created output directory: captured_images
✓ Connected to COM6 at 115200 baud

Board is ready!
Commands:
  'c' - Capture VGA (640x480)
  '1' - Capture QVGA (320x240)
  '2' - Capture VGA (640x480)
  '3' - Capture 1080p
  '4' - Capture 3MP (2048x1536)
  'q' - Quit

Enter command:
```

### Interactive Commands

| Command | Resolution | Dimensions | Use Case |
|---------|------------|-----------|----------|
| `c` | VGA | 640×480 | Quick test, standard resolution |
| `1` | QVGA | 320×240 | Fastest capture, lowest detail |
| `2` | VGA | 640×480 | Balanced quality and speed |
| `3` | 1080p (FHD) | 1920×1080 | High quality, slower |
| `4` | 3MP (QXGA) | 2048×1536 | Maximum quality, slowest |
| `q` | - | - | Quit program |

### Example Session

```
Enter command: 1
Sent command: '1'

════════════════════════════════════
Capturing image...
════════════════════════════════════
Waiting for image size...
✓ Image size: 15234 bytes (14.9 KB)
✓ Starting binary read...
Reading image data...
  15234/15234 bytes (100%)     
✓ First 2 bytes: 0xFF 0xD8
✓ Valid JPEG start marker (0xFF 0xD8)
✓ Last 2 bytes: 0xFF 0xD9
✓ Valid JPEG end marker (0xFF 0xD9)
✓ Image saved: captured_images/image_20260104_143025.jpg
  Size: 15234 bytes (14.9 KB)

Enter command: q
✓ Disconnected
```

---

## Execution Flow

### Program Startup

```
┌──────────────────────────────────────┐
│ Script Started (python capture-image.py)
└────────────┬───────────────────────────┘
             │
             ▼
┌──────────────────────────────────────┐
│ Display Welcome Banner               │
└────────────┬───────────────────────────┘
             │
             ▼
┌──────────────────────────────────────┐
│ Create Output Directory              │
│ (if doesn't exist)                   │
└────────────┬───────────────────────────┘
             │
             ▼
┌──────────────────────────────────────┐
│ Connect to Arduino                   │
│ Open Serial Port at 115200 baud      │
└────────────┬───────────────────────────┘
             │
      ┌──────┴──────┐
      ▼             ▼
   SUCCESS      FAILED
      │             │
      │          ┌──┴─────────────────┐
      │          │ Display Error      │
      │          │ Exit Program       │
      │          └────────────────────┘
      │
      ▼
┌──────────────────────────────────────┐
│ Wait 2 seconds for initialization    │
└────────────┬───────────────────────────┘
             │
             ▼
┌──────────────────────────────────────┐
│ Display Commands Reference           │
│ Enter Interactive Loop               │
└──────────────────────────────────────┘
```

### Main Interactive Loop

```
┌────────────────────────────────────────┐
│ Prompt: "Enter command: "              │
└────────────┬─────────────────────────────┘
             │
             ▼
    ┌────────┴────────┐
    │ User Input      │
    └────────┬────────┘
             │
        ┌────┴────┐
        │          │
    ┌───▼──┐  ┌───▼──┐
    │ 'q'  │  │Other │
    │ Quit │  │Cmd   │
    └───┬──┘  └───┬──┘
        │         │
        │      ┌──▼──────────────────┐
        │      │ Valid Command?      │
        │      │ (1,2,3,4,c)         │
        │      └──┬──────────────────┘
        │         │
        │    ┌────┴──────┐
        │    │ YES  │ NO │
        │    │      └┬───┴──────────┐
        │    │       │              │
        │    │    ┌──▼────────────┐ │
        │    │    │ Capture       │ │
        │    │    │ Image         │ │
        │    │    └──┬────────────┘ │
        │    │       │              │
        │    │    ┌──▼────────────┐ │
        │    │    │ Save          │ │
        │    │    │ Image         │ │
        │    │    └──┬────────────┘ │
        │    │       │              │
        │    │    ┌──▼────────────┐ │
        │    │    │ Print Error   │ │
        │    │    │ "Invalid"     │ │
        │    │    └──┬────────────┘ │
        │    └───────┴──────────────┘
        │               │
        └───────────────┤
                        │
                        ▼
            ┌──────────────────────┐
            │ Back to Prompt       │
            │ (or Exit if 'q')     │
            └──────────────────────┘
```

### Image Capture Sequence

```
┌──────────────────────────────────────┐
│ User Enters: 'c', '1', '2', '3', '4' │
└────────────┬─────────────────────────┘
             │
             ▼
┌──────────────────────────────────────┐
│ Send Command to Arduino              │
│ (serialwrite byte over serial link)  │
└────────────┬─────────────────────────┘
             │
             ▼
┌──────────────────────────────────────┐
│ Wait for SIZE marker                 │
│ Read text response from Arduino      │
│ Parse: "SIZE:[number]"               │
└────────────┬─────────────────────────┘
             │
      ┌──────┴──────┐
      ▼             ▼
   FOUND       NOT FOUND
    SIZE        SIZE
      │             │
      │          ┌──┴──────────────────┐
      │          │ Error & Return      │
      │          └────────────────────┘
      │
      ▼
┌──────────────────────────────────────┐
│ Wait for START_BINARY_DATA marker    │
│ Continue reading text responses      │
└────────────┬─────────────────────────┘
             │
      ┌──────┴──────┐
      ▼             ▼
   FOUND       NOT FOUND
    START       START
      │             │
      │          ┌──┴──────────────────┐
      │          │ Error & Return      │
      │          └────────────────────┘
      │
      ▼
┌──────────────────────────────────────┐
│ Skip Whitespace                      │
│ Find First Data Byte                 │
└────────────┬─────────────────────────┘
             │
             ▼
┌──────────────────────────────────────┐
│ Read Binary Data                     │
│ - Read byte-by-byte                  │
│ - Append to image_data buffer        │
│ - Track last 2 bytes                 │
│ - Check for JPEG end marker (0xFF 0xD9)
│ - Show progress every 5KB            │
└────────────┬─────────────────────────┘
             │
      ┌──────┴──────┬──────────┐
      ▼             ▼          ▼
  ALL DATA   JPEG END    TIMEOUT
   READ     MARKER FOUND
      │        │          │
      └────────┴──────────┤
                          ▼
                   ┌──────────────────┐
                   │ Return Image Data│
                   └──────────────────┘
```

### Image Saving Sequence

```
┌────────────────────────────────────────┐
│ Received Binary Image Data             │
└────────────┬─────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────┐
│ Validate JPEG Start Marker             │
│ Check: First 2 bytes = 0xFF 0xD8       │
└────────────┬─────────────────────────────┘
             │
      ┌──────┴──────┐
      ▼             ▼
   VALID      INVALID
      │             │
      │        Print Warning
      │
      ▼
┌────────────────────────────────────────┐
│ Validate JPEG End Marker               │
│ Check: Last 2 bytes = 0xFF 0xD9        │
└────────────┬─────────────────────────────┘
             │
      ┌──────┴──────┐
      ▼             ▼
   VALID      INVALID
      │             │
      │        Print Warning
      │
      ▼
┌────────────────────────────────────────┐
│ Generate Filename                      │
│ "image_YYYYMMDD_HHMMSS.jpg"           │
│ Example: "image_20260104_143025.jpg"  │
└────────────┬─────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────┐
│ Open File in Binary Write Mode         │
│ Write Image Data to Disk               │
│ Close File                             │
└────────────┬─────────────────────────────┘
             │
      ┌──────┴──────┐
      ▼             ▼
   SUCCESS      FAILED
      │             │
      │        Print Error
      │        Return None
      │
      ▼
  Return
  Filename
```

---

## Function Reference

### `ensure_output_dir()`

**Purpose**: Create output directory if it doesn't exist

```python
ensure_output_dir()
```

**Behavior**:
- Checks if `OUTPUT_DIR` exists
- Creates directory if missing
- Prints confirmation message
- No error if directory already exists

**Output Example**:
```
Created output directory: captured_images
```

---

### `connect_to_board(port, baudrate)`

**Purpose**: Establish serial connection with Arduino

```python
ser = connect_to_board(COM_PORT, BAUD_RATE)
```

**Parameters**:
| Parameter | Type | Example | Description |
|-----------|------|---------|-------------|
| `port` | str | "COM6" | Serial port name |
| `baudrate` | int | 115200 | Communication speed |

**Returns**:
- Serial connection object if successful
- `None` if connection fails

**Output Examples**:
```
✓ Connected to COM6 at 115200 baud
```

```
✗ Failed to connect: [Errno 13] Could not open port COM6: PermissionError
```

---

### `send_command(ser, cmd)`

**Purpose**: Send a single character command to Arduino

```python
send_command(ser, '1')  # Send QVGA capture command
```

**Parameters**:
| Parameter | Type | Example | Description |
|-----------|------|---------|-------------|
| `ser` | Serial | (connection) | Serial connection object |
| `cmd` | str | "c" | Single character command |

**Returns**: None

**Side Effects**:
- Sends data over serial port
- Prints confirmation message

**Output Example**:
```
Sent command: '1'
```

---

### `capture_image(ser, resolution='c', timeout=120)`

**Purpose**: Capture image from camera and receive binary data

```python
image_data = capture_image(ser, '1', timeout=120)
```

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ser` | Serial | - | Serial connection object |
| `resolution` | str | 'c' | Resolution command ('c', '1', '2', '3', '4') |
| `timeout` | int | 120 | Maximum seconds to wait |

**Returns**:
- Binary image data (bytes) if successful
- Empty bytes `b''` if failed

**Process**:
1. Send capture command to Arduino
2. Wait for size marker and parse image size
3. Wait for START_BINARY_DATA marker
4. Read binary data byte-by-byte
5. Detect JPEG end marker (0xFF 0xD9)
6. Return complete image data

**Output Examples**:
```
════════════════════════════════════
Capturing image...
════════════════════════════════════
Waiting for image size...
✓ Image size: 47834 bytes (46.71 KB)
✓ Starting binary read...
Reading image data...
  47834/47834 bytes (100%)
```

---

### `save_image(image_data)`

**Purpose**: Save binary image data to JPEG file

```python
filename = save_image(image_data)
```

**Parameters**:
| Parameter | Type | Description |
|-----------|------|-------------|
| `image_data` | bytes | Binary JPEG data |

**Returns**:
- Full file path (str) if successful
- `None` if failed

**Validation**:
- Checks JPEG start marker (0xFF 0xD8)
- Checks JPEG end marker (0xFF 0xD9)
- Warns if markers are invalid

**Output Examples**:
```
✓ First 2 bytes: 0xFF 0xD8
✓ Valid JPEG start marker (0xFF 0xD8)
✓ Last 2 bytes: 0xFF 0xD9
✓ Valid JPEG end marker (0xFF 0xD9)
✓ Image saved: captured_images/image_20260104_143025.jpg
  Size: 47834 bytes (46.71 KB)
```

---

### `main()`

**Purpose**: Main program entry point with user interaction

**Behavior**:
1. Display welcome banner
2. Create output directory
3. Connect to Arduino
4. Display command reference
5. Read user commands in loop
6. Process capture commands
7. Handle shutdown gracefully

**User Interaction**:
- Prompts for command input
- Validates command (c, 1, 2, 3, 4, q)
- Executes capture or quality change
- Handles Ctrl+C interruption

---

## Communication Protocol

### Serial Connection Specifications

| Property | Value |
|----------|-------|
| Baud Rate | 115200 bps |
| Data Bits | 8 |
| Stop Bits | 1 |
| Parity | None |
| Flow Control | None |
| Timeout | 5 seconds |

### Data Flow: Python → Arduino

```
┌─────────┐                    ┌────────┐
│ Python  │ Sends single byte: │Arduino │
│ Script  │────────'c'────────>│ Board  │
└─────────┘  (e.g., 'c')      └────────┘
```

**Command Format**:
- Single ASCII character
- No newline required
- Case-insensitive (both 'c' and 'C' work)

### Data Flow: Arduino → Python

```
┌────────┐                    ┌─────────┐
│Arduino │ Sends text:        │ Python  │
│ Board  │────────────────────>│ Script  │
│        │  "SIZE:47834"      │         │
│        │  "START_BINARY..."│         │
│        │  [Binary JPEG]     │         │
│        │  "END_IMAGE_DATA"  │         │
└────────┘                    └─────────┘
```

**Response Sequence**:
1. Text messages (capture status)
2. "SIZE:[bytes]" line
3. "START_BINARY_DATA" marker
4. [Binary JPEG data]
5. "END_IMAGE_DATA" marker

### JPEG File Structure

```
Byte 0-1:   0xFF 0xD8    ← Start of Image (SOI) marker
Byte 2-N:   JPEG Data    (segments, tables, image data)
Byte N-1:   0xFF 0xD9    ← End of Image (EOI) marker
```

---

## Troubleshooting Guide

### Issue: Connection Error

**Error Message**:
```
✗ Failed to connect: [Errno 13] Could not open port COM6
```

**Possible Causes**:
1. Arduino not connected via USB
2. Wrong COM port specified
3. Serial port in use by another application
4. Incorrect driver installation

**Solutions**:
1. Verify Arduino connection
2. Check Device Manager for correct port
3. Close other serial monitor applications
4. Reinstall Arduino drivers

---

### Issue: Timeout Error

**Symptom**: Script waits indefinitely or times out

**Error Message**:
```
✗ Failed to get image size or start marker
```

**Possible Causes**:
1. Arduino sketch not running
2. Arduino not responding
3. Baud rate mismatch
4. USB cable issues

**Solutions**:
1. Verify Arduino sketch is uploaded and running
2. Check serial monitor independently
3. Confirm baud rate matches (115200)
4. Try different USB cable

---

### Issue: Corrupted Image Files

**Symptom**: Images won't open or show as invalid

**Causes**:
1. Incomplete data transmission
2. Serial communication errors
3. Insufficient timeout value

**Solutions**:
1. Increase timeout: `capture_image(ser, cmd, timeout=180)`
2. Try lower resolution (slower transfer = more reliable)
3. Check USB cable quality
4. Close other USB devices to reduce interference

---

### Issue: "Invalid JPEG Marker" Warning

**Warning Message**:
```
⚠ Warning: Invalid JPEG start marker!
⚠ Warning: Invalid JPEG end marker!
```

**Possible Causes**:
1. Data corruption during transmission
2. JPEG compression settings on Arduino
3. Camera firmware issue

**Solutions**:
1. Try capture again
2. Reset Arduino and retry
3. Check camera power supply voltage
4. Try different resolution

---

### Issue: Port Already in Use

**Error Message**:
```
✗ Failed to connect: [Errno 2] Could not open port COM6: PermissionError
```

**Solutions**:
1. Close Arduino IDE Serial Monitor
2. Check for other running Python instances
3. Restart the Arduino board

---

### Issue: No "SIZE" Data Received

**Symptom**: Script waits but never gets size information

**Possible Causes**:
1. Arduino not sending data
2. Baud rate mismatch
3. Serial buffer issues

**Debug Steps**:
1. Open Arduino IDE Serial Monitor
2. Set baud rate to 115200
3. Send command 'c' manually
4. Observe output to verify communication

---

## Performance Characteristics

### Capture and Transmission Times

| Resolution | Typical Capture | Transmission @ 115200 | Total |
|------------|-----------------|----------------------|-------|
| QVGA (320×240) | 150-200ms | 0.5-1 sec | 1-2 sec |
| VGA (640×480) | 200-300ms | 2-3 sec | 3-4 sec |
| 1080p (1920×1080) | 400-600ms | 5-8 sec | 6-9 sec |
| 3MP (2048×1536) | 600-800ms | 8-12 sec | 9-13 sec |

### Factors Affecting Speed

1. **Resolution**: Higher = slower
2. **Quality Setting**: Can vary capture time
3. **Lighting**: Low light may require longer exposure
4. **USB Connection**: Quality of cable/port affects transmission
5. **System Load**: Other processes may cause delays

### Baud Rate Impact

```
Higher Baud Rate = Faster Transmission

115200 baud: ~14.4 KB/second
230400 baud: ~28.8 KB/second (requires code changes)
921600 baud: ~115.2 KB/second (requires code changes)
```

---

## File Structure

### Output Files

**Location**: `captured_images/` directory

**Filename Format**: `image_YYYYMMDD_HHMMSS.jpg`

**Examples**:
```
image_20260104_143025.jpg  (Jan 4, 2026 at 14:30:25)
image_20260104_143127.jpg  (Jan 4, 2026 at 14:31:27)
image_20260104_143230.jpg  (Jan 4, 2026 at 14:32:30)
```

**File Details**:
- **Format**: JPEG (binary)
- **Size Range**: 10KB - 150KB depending on resolution and quality
- **Integrity**: Verified by JPEG markers

---

## Example Workflows

### Workflow 1: Quick Test Capture

```
1. Run: python capture-image.py
2. Wait for "Board is ready!" message
3. Input: 1
4. Wait for image to be captured and saved
5. Find image in captured_images/ folder
```

**Expected Time**: ~2 seconds

---

### Workflow 2: Batch Capture

```
1. Run: python capture-image.py
2. Enter commands in sequence:
   c (or 1, 2, 3, 4)
   c
   c
   q
3. Monitor captured_images/ folder for 3 images
```

**Expected Time**: ~5-10 seconds per image

---

### Workflow 3: Different Resolutions

```
1. Run: python capture-image.py
2. Input: 1  (QVGA - smallest)
3. Input: 2  (VGA - standard)
4. Input: 3  (1080p - high)
5. Input: 4  (3MP - maximum)
6. Input: q  (quit)
```

**Expected Time**: ~20-30 seconds total

---

## Advanced Usage

### Changing Configuration Programmatically

Edit the configuration section before running:

```python
COM_PORT = "COM3"           # Different Arduino port
BAUD_RATE = 115200         # Keep consistent with Arduino
OUTPUT_DIR = "my_images"   # Custom output folder
```

### Extended Timeout for Unreliable Connections

```python
# In the command input section, modify:
image_data = capture_image(ser, cmd, timeout=180)  # 3 minutes
```

### Batch Processing Script

```python
# Add after "Board is ready!" message:
import time

for resolution in ['1', '2', '4']:
    image_data = capture_image(ser, resolution)
    if image_data:
        save_image(image_data)
    time.sleep(1)  # Wait between captures
```

---

## Best Practices

### Do's ✓
- ✓ Verify serial port before running
- ✓ Ensure Arduino is powered and initialized
- ✓ Use reliable USB cable
- ✓ Start with QVGA for testing
- ✓ Keep baud rate at 115200
- ✓ Check output directory exists

### Don'ts ✗
- ✗ Don't disconnect Arduino during capture
- ✗ Don't run multiple instances simultaneously
- ✗ Don't change baud rate without modifying code
- ✗ Don't use USB hubs with poor power
- ✗ Don't interrupt with Ctrl+C during transmission (use 'q' instead)

---

## Debugging Tips

### Enable Verbose Output

Add print statements to debug:

```python
def capture_image(ser, resolution='c', timeout=120):
    # ... existing code ...
    print(f"DEBUG: Capture start time = {start_time}")
    print(f"DEBUG: Buffer content = {repr(buffer)}")
    print(f"DEBUG: Image size = {image_size}")
```

### Test Serial Connection

```python
# Create simple test script:
import serial
import time

ser = serial.Serial("COM6", 115200, timeout=5)
time.sleep(2)
ser.write(b'c')  # Send test command
time.sleep(1)
print(ser.read(100))  # Read response
ser.close()
```

### Verify Arduino Independently

1. Open Arduino IDE Serial Monitor
2. Set baud rate to 115200
3. Send command character
4. Verify Arduino responds
5. If it does: Python script issue
6. If it doesn't: Arduino/hardware issue

---

## Support and References

### Documentation Files
- `DOCUMENTATION.md` - Arduino sketch documentation
- `capture-image.cpp` - Arduino sketch source code
- `capture-image.py` - Python script source code (this file)

### External Resources
- [PySerial Documentation](https://pyserial.readthedocs.io/)
- [Python Serial Communication](https://docs.python.org/3/library/serial.html)
- [JPEG Format Specification](https://en.wikipedia.org/wiki/JPEG)

### Getting Help

If script fails to work:
1. Check error message carefully
2. Review troubleshooting section above
3. Verify Arduino sketch is running
4. Test serial connection independently
5. Examine captured images for corruption
6. Check system event logs for port errors

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-04 | Initial release with comprehensive documentation |

---

**Last Updated**: January 4, 2026
**Script Name**: capture-image.py
**Compatibility**: Python 3.6+
**Platform**: Windows, macOS, Linux
