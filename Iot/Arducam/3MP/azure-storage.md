# Azure Blob Storage Setup Guide (Updated Jan 2026)
This guide provides the steps to configure an Azure Storage account to allow an ESP32 or similar IoT device to upload images using Shared Access Signatures (SAS).
## 📋 Prerequisites
- **Azure Account**: A subscription (Free or Pay-As-You-Go).
- **Hardware**: ESP32 board (standard Arduino Uno boards lack the native WiFi and memory required for HTTPS image uploads).
## 🛠️ Step 1: Create Azure Storage Account
1. **Log in**: Go to the [Azure Portal](https://portal.azure.com).
2. **Create Resource**:
   - Click "Create a resource".
   - Search for "Storage account" and click **Create**.
3. **Basics Tab** (Continue through all tabs):
   - **Subscription**: Select your active subscription.
   - **Resource Group**: Create a new one (e.g., `IoT_Projects`).
   - **Storage account name**: Enter a unique name (e.g., `myiotcamstorage` - replace with your own). **Note**: Only lowercase letters and numbers, must be globally unique.
   - **Region**: Select the region physically closest to you (affects latency and data residency).
   - **Performance**: Standard (Premium is unnecessary for this project).
   - **Redundancy**: Locally-redundant storage (LRS) — most cost-effective for hobby projects.
   - **Advanced tab**: Default settings are fine (no changes needed).
   - **Networking**: Default (Private endpoint can be configured later if needed).
   - **Data protection**: Optional - enable soft delete if you want recovery options (not required).
4. **Finish**: Click **Review + Create**, then click **Create**.
## 📂 Step 2: Create Blob Container
1. Navigate to your new Storage Account resource.
2. In the left sidebar, locate the **Data storage** section and click **Containers**.
3. Click **+ Container**.
4. **Configuration**:
   - **Name**: `images` (If you change this, update your ESP32 code to match).
   - **Public access level**: **Private** (No anonymous access). We will use a SAS token for security.
5. Click **Create**.
## 🔑 Step 3: Generate SAS Token (Critical Step - Two Options Available)

**⚠️ IMPORTANT: Two SAS Types Available**

The current guide uses **Account SAS** (older approach). However, Microsoft recommends **User Delegation SAS** for superior security. Both work, but choose one:

- **User Delegation SAS** (RECOMMENDED - more secure): Uses Microsoft Entra ID credentials, no storage account key exposed
- **Account SAS** (Legacy - still functional): Uses storage account key (less secure but simpler to set up)

### Option A: User Delegation SAS (RECOMMENDED - More Secure)

1. In Storage Account menu → **Security + networking** → **Shared access signature**
2. Under "Permissions to grant", select **User delegation SAS** (if available as option)
3. Set expiry date to 30-90 days
4. Click **Generate SAS and connection string**

### Option B: Account SAS (Legacy - Still Works)

The Account SAS method described below is still functional but less secure than User Delegation SAS.

1. **Wait for storage account creation** to complete fully (this may take 1-2 minutes).
2. In the Storage Account menu on the left sidebar, go to **Security + networking** section.
3. Click **Shared access signature**.
4. **Configure the following (ALL required)**:
   - **Allowed services**: ✅ **Blob** (uncheck others)
   - **Allowed resource types**: ✅ **Service**, ✅ **Container**, ✅ **Object** (all three required)
   - **Allowed permissions**: ✅ **Read**, ✅ **Write**, ✅ **Create**, ✅ **List** (uncheck Delete and Add)
   - **Start date/time**: Leave as default (or set to today)
   - **Expiry date/time**: Set to 30-90 days from today (balance security and convenience)
   - **Allowed protocols**: ✅ **HTTPS only** (required for security)
   - **Signing key**: Leave as **key1** (default)
5. **Generate**: Click **Generate SAS and connection string** button at bottom.
6. **Wait** - the page will refresh and show your new SAS token.
## 💾 Step 4: Using the Token in Your Code
1. Locate the field labeled **SAS token** in the Azure Portal.
2. **Important**: The SAS token will start with `?sv=` or `sv=`
3. **Clean the Token**: Copy the string but **remove the leading `?` if present**.
   - ✅ **Correct format**: Starts with `sv=` (version) followed by other parameters
   - ❌ **Incorrect format**: Starts with `?sv=` (question mark should be removed)
4. **Update io_config.h**:
This project uses a separate configuration file for sensitive credentials.
Edit [`include/io_config.h`](include/io_config.h):
```cpp
#ifndef IO_CONFIG_H
#define IO_CONFIG_H
// WiFi credentials - REPLACE WITH YOUR OWN VALUES
#define WIFI_SSID "YOUR_ACTUAL_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_ACTUAL_WIFI_PASSWORD"

// Azure Blob Storage configuration - REPLACE WITH YOUR OWN VALUES
// Get these from your Azure Storage Account (NOT publicly shared)
#define AZURE_STORAGE_ACCOUNT "your_storage_account_name"  // Your unique account name
#define AZURE_CONTAINER "images"                            // Container name you created
#define AZURE_SAS_TOKEN "your_sas_token_here"               // Your actual SAS token (without leading ?)

#endif // IO_CONFIG_H
```

⚠️ **CRITICAL SECURITY WARNING**:
- **NEVER commit `io_config.h` to public Git repositories**
- Add `io_config.h` to `.gitignore` before committing code
- Your SAS token provides write access to your storage container
- Anyone with this token can upload files to your Azure storage
- Treat this file as you would treat a password
- **Account SAS** exposes account key (less secure)
- **User Delegation SAS** uses only Entra ID credentials (more secure)
**URL Structure**: The ESP32 code builds the URL in this format:
```
https://[your_storage_account].blob.core.windows.net/[container_name]/[filename].jpg?[your_sas_token]
```
## Step 5: Hardware Requirements

⚠️ **NOTE: If Azure Portal UI differs from instructions above:**
The Azure Portal is updated frequently. If you don't see the exact options described in Step 3:
- Look for "Shared access signature" section in Security + networking
- If you see tabs or different organization, Azure Portal may have been updated
- The core concept remains the same: Allowed services (Blob), Resource types (Service, Container, Object), Permissions (Read, Write, Create)
- Contact Microsoft docs if the UI has significantly changed since Jan 2026

**IMPORTANT**: Arduino Uno doesn't have WiFi. You need one of these:
### Option A: ESP32 (Recommended)
- ESP32 DevKit board (~$5-10)
- Built-in WiFi and Bluetooth
- More memory for handling images
- Directly compatible with Arduino IDE/PlatformIO
### Option B: ESP8266
- Cheaper than ESP32
- Built-in WiFi
- Less memory but still works
### Wiring (ESP32 to Arducam)
```
ESP32          Arducam Mega
GPIO 18  ----> SCK
GPIO 23  ----> MOSI
GPIO 19  ----> MISO
GPIO 5   ----> CS
3.3V     ----> VCC
GND      ----> GND
```
## Step 6: Upload and Test
1. **Update io_config.h** with your actual credentials (WiFi SSID, password, storage account name, and SAS token).
2. **Add io_config.h to .gitignore** (if using Git) to prevent accidental credential exposure.
3. Connect your ESP32 to your computer via USB.
4. In PlatformIO:
   - Select the correct board (ESP32 DevKit)
   - Select the correct COM port
   - Click "Upload" to compile and flash the firmware
5. **Wait for upload to complete** (typically 20-30 seconds).
6. Open Serial Monitor in PlatformIO:
   - Set baud rate to **115200**
   - You should see the startup message with available commands
7. Test by sending commands:
   - `c` - Capture VGA image and stream to serial (no upload, no WiFi needed)
   - `u` or `u2` - Capture VGA and upload to Azure (requires WiFi)
   - `u1` - Capture QVGA and upload
   - `u3` - Capture 1080p and upload
   - `u4` - Capture 3MP and upload
   - `q` - Set quality HIGH (smaller files)
   - `w` - Set quality MEDIUM (default)
   - `e` - Set quality LOW (larger files)
8. **For first upload**: Watch the serial output - WiFi will connect (takes ~8 seconds) then upload proceeds
## Step 7: View Your Images
1. Go to Azure Portal
2. Navigate to your Storage Account
3. Click "Containers" → "images"
4. You'll see your uploaded images with timestamps: `image_1_12345.jpg`, `image_2_67890.jpg`, etc.
5. Click any image to download/view it
## Alternative: Using Storage Explorer
Download **Azure Storage Explorer** (free desktop app):
- https://azure.microsoft.com/en-us/features/storage-explorer/
- Easier way to browse and download images
## ⚠️ Common Troubleshooting
### 403 Forbidden
Access denied when attempting upload - usually configuration issue.

**Most common causes**:
- Missing **"Object"** checkmark under "Allowed resource types" (required to write files)
- Missing **"Create"** permission (required to create new blobs)
- Missing **"Write"** permission
- Container is set to "Public" when it should be "Private"
- SAS token has expired

**Verify in Azure Portal**:
1. Go to Storage Account → **Security + networking** → **Shared access signature**
2. Ensure these are checked:
   - **Allowed services**: Blob ✅
   - **Allowed resource types**: Service ✅, Container ✅, Object ✅ (all three required)
   - **Allowed permissions**: Read ✅, Write ✅, Create ✅, List ✅
3. Check **Expiry date** - SAS token may have expired
4. Go to **Containers** → verify container is "Private" (not "Blob")

**Solution**: If unsure, regenerate a new SAS token with correct settings
### Time Sync Error (SAS Token not yet valid)
The ESP32 must have correct time for SAS token validation.

**Problem**: Azure rejects SAS token with error like "AuthenticationFailed" or "not yet valid"

**Root cause**: ESP32 internal clock is wrong (e.g., shows year 1970 on startup)

**Why it matters**: SAS tokens include start/expiry times. If device clock doesn't match Azure time, token is rejected.

**Current code limitation**: Does NOT automatically sync time via NTP

**Solutions** (in order of recommendation):
1. **Implement NTP sync** in setup() - most reliable
   ```cpp
   configTime(0, 0, "pool.ntp.org", "time.nist.gov");
   delay(2000);
   ```
2. **Wait for WiFi to sync time** - takes ~30 seconds after WiFi connects
3. **Check SAS token start/expiry times** - regenerate if time window is too narrow
4. **Set narrow time window** when generating SAS (current time ± a few minutes)
### HTTPS/SSL Connection Fails
Ensure you are using the `WiFiClientSecure` library in Arduino to handle the encrypted connection.
- The code uses `client.setInsecure()` to simplify TLS for SAS upload
- For production, consider proper certificate validation
### WiFi Connection Issues
- **Check SSID and password** in `io_config.h` - must be exact match (case-sensitive for password)
- **Ensure 2.4GHz WiFi** - ESP32 standard models do not support 5GHz (check your router settings)
- **Verify WiFi is open or WPA2** - ESP32 does not support WPA3 or enterprise authentication
- **Signal strength** - Move ESP32 closer to router if signal is weak
- **Restart WiFi router** - Sometimes helps if connection hangs
- **Check serial output** - Waiting for WiFi connection shows "." every 500ms for up to 20 seconds
### HTTP Response code: 404
- Container name doesn't match (check `AZURE_CONTAINER` in `io_config.h`)
- Storage account name is wrong (check `AZURE_STORAGE_ACCOUNT`)
### "Operation could not be completed within the specified time"
- **Network is slow** - Check WiFi signal strength and bandwidth
- **Image is too large** - Uploading 3MP images on slow connection may timeout
- **Azure service latency** - Occasionally Azure takes longer to respond
- **Solutions**:
  - Try smaller resolutions first (QVGA or VGA instead of 3MP)
  - Increase quality setting (q=HIGH) to reduce file size
  - Get closer to WiFi router
  - Try upload during off-peak hours
  - Check Azure service status (azure.microsoft.com/status)
### Memory allocation failed
- **Root cause**: Image too large for available ESP32 RAM
- **Typical ESP32 RAM**: 4-16 MB (varies by model)
- **Image sizes**: QVGA ~12KB, VGA ~45KB, 1080p ~70KB, 3MP ~90KB
- **Solutions**:
  - Use QVGA or VGA resolutions for standard ESP32
  - Enable PSRAM option in Arduino/PlatformIO settings if your board has it
  - Use ESP32-S3 with PSRAM for larger images
  - Note: Even with standard RAM, code uses buffering so entire image doesn't load at once
## 💰 Cost Information
**Azure Free Tier** (includes):
- First 5 GB of LRS Blob Storage (free)
- 20,000 read operations per month (free)
- 10,000 write operations per month (free)

**Typical hobby project usage**:
- If you capture 1-3 images per day: ~30-90 images/month
- Assuming 50 KB per image: ~2-5 MB storage/month
- Operations: Well within free tier limits (20,000+ reads and 10,000+ writes)

**Result**: **Completely free** for casual use!

**If you exceed free tier**:
- Storage: $0.018 per GB/month (LRS)
- Write operations: $0.005 per 10,000 operations
- Even heavy use (100 images/day) costs <$1/month
## 🔒 Security Notes

⚠️ **CRITICAL - SAS Token Security**:
- The SAS token grants upload/write access to your Azure storage container
- **NEVER commit `io_config.h` to public repositories (GitHub, etc.)**
- **NEVER share your code with the write SAS token embedded**
- **ALWAYS use `.gitignore` to exclude `io_config.h`**

**SAS Token Best Practices**:
- Set realistic expiration dates (e.g., 30-90 days, not years)
- Use read-only SAS tokens only for accessing stored images
- Keep write/create tokens private (only on your device)
- Regenerate tokens regularly (every 30-90 days)
- Monitor Azure Storage Account access logs for suspicious activity
- Use strong container permissions (Private, not Public)

**If Token is Compromised**:
1. Immediately regenerate a new SAS token in Azure Portal
2. Update the token in your ESP32 firmware
3. Review storage account access logs for unauthorized uploads
4. Monitor your storage container for unexpected files
## Accessing Your Uploaded Images
After upload, your images are stored securely in your Azure Storage container.

**How to access:**
1. **Through Azure Portal**:
   - Navigate to your Storage Account → Containers → images
   - Click on the image file to view or download

2. **With SAS Token** (temporary read-only access):
   - Generate a separate read-only SAS token in Azure Portal for sharing
   - Share only read-only tokens, never your write token

3. **Publicly** (not recommended):
   - Change container access level from Private to Blob
   - Images become world-readable
   - Only do this if images are not sensitive

⚠️ **Never share your write/create SAS token** (used by ESP32). Only share read-only tokens for accessing stored images.