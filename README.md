# Project Setup Guide

## 1. Arduino Setup

### Prerequisites
- [Arduino IDE](https://www.arduino.cc/en/main/software)
- A compatible Arduino board (e.g., Arduino Nano 33 BLE Sense or similar)
- USB cable to connect the board to your computer

### Installation
1. Download and install the Arduino IDE from the official website.
2. Launch the Arduino IDE and install the required board support:
   - Go to **Tools** > **Board** > **Boards Manager...**  
   - Search for the desired board (e.g., "Arduino mbed-enabled Boards"), then **Install**.
3. Install the necessary libraries:
   - Go to **Sketch** > **Include Library** > **Manage Libraries...**  
   - Search for:
     - "Arduino_LSM9DS1"
     - "ArduinoBLE"
     - Any other libraries referenced in the code
   - Click **Install** for each.
4. Open the project’s main `.ino` file from this repository in Arduino IDE.
5. Select the board and port:
   - **Tools** > **Board** > your specific board
   - **Tools** > **Port** > the correct COM port (Windows) or /dev/tty... (macOS/Linux)
6. Click the **Upload** button to compile and upload the sketch to your board.

## 2. Flutter Setup

### Prerequisites
- [Flutter SDK](https://docs.flutter.dev/get-started/install)
- [Dart](https://dart.dev/get-dart) (Usually bundled with Flutter)
- An IDE or text editor (e.g., VS Code or Android Studio)
- For iOS deployment:
  - Xcode installed (macOS only)
  - An Apple Developer account for device testing

## Additional Prerequisites for iOS
- Ensure Ruby is installed (e.g. via `brew install ruby` on macOS) for managing CocoaPods.
- Update your PATH to include Ruby if necessary.
- Run `sudo gem install cocoapods` if CocoaPods is not installed or needs updating.

### Directory Structure
Your Flutter SDK can be placed anywhere on your system. Common locations include:
- macOS: /Users/<your_username>/development/flutter
- Windows: C:\src\flutter
### Directory Structure
Your Flutter SDK can be placed anywhere on your system. Common locations include:
- macOS: /Users/<your_username>/development/flutter
- Windows: C:\src\flutter
- Linux: /home/<your_username>/flutter

**IMPORTANT:** For the third required library, download it from:
[Download Third Library](https://github.com/tensorflow/tflite-micro-arduino-examples/tree/main)

Ensure you follow the provided instructions to integrate it with your project.

## Real Device iOS Setup - iPhone

- Ensure that the Flutter SDK is located in a directory where you have full read/write permissions.
- When using an actual iPhone, connect it via USB and then select your iPhone directly from Xcode’s toolbar at the top. This method bypasses the need for the "flutter devices" command if your device does not appear.

### Detailed Flutter Installation
1. After downloading and extracting Flutter to your chosen directory:
   - On macOS or Linux, add a line to your `~/.zshrc` or `~/.bashrc`:
     ```
     export PATH="$PATH:/path/to/flutter/bin"
     ```
   - On Windows, open “System Properties” → “Advanced” → “Environment Variables…”, and add the full Flutter `\bin` path to your PATH variable.
2. Run `flutter doctor` in a new terminal to verify your setup and install any missing dependencies.
3. (Optional) If using Android Studio, go to “Preferences” → “Plugins” → or “Marketplace” and install the “Flutter” and “Dart” plugins.

### Installation
1. Download Flutter from the official website and extract it to a chosen directory.
2. Add the Flutter `bin` folder to your system’s PATH to access the `flutter` command.
   - On macOS or Linux, edit your shell profile (e.g., `~/.zshrc` or `~/.bashrc`).
   - On Windows, update Environment Variables in System Settings.
3. Run `flutter doctor` to ensure all required dependencies are installed.
   - Follow any instructions to fix missing dependencies.

### Running Flutter - Additional Details
- In your Flutter project folder, run:
  ```
  flutter pub get
  flutter run
  ```
  This will fetch dependencies and compile your app.
- If using VS Code, open the command palette (**View** → **Command Palette**), type “Flutter: Select Device” to choose a recognized emulator or connected device, then press F5 to start debugging.

## 3. Running Flutter on an Emulator or Simulator

### Emulator (Android)
1. Install Android Studio or the standalone Android SDK.
2. Create an emulator (AVD) through the AVD Manager.  
3. Start the emulator, then navigate to your Flutter project folder in a terminal and run:
   ```
   flutter run
   ```
   The app should launch inside the emulator.

### Simulator (iOS)
1. On macOS, install Xcode from the App Store.
2. Run `xcode-select --install` in your terminal if needed.
3. In Xcode, open **Preferences** > **Locations** and ensure the command-line tools are set.
4. Use `open -a Simulator` or start the simulator from Xcode’s **Devices and Simulators**.
5. In the Flutter project folder, run:
   ```
   flutter run
   ```
   The app should launch on the iOS simulator.

## 4. Running on an Actual iPhone

### Device Setup
1. Connect your iPhone to your Mac via USB.
2. In Xcode, go to **Preferences** > **Accounts** and add your Apple ID.
3. Set up a provisioning profile in the **Signing & Capabilities** tab of your app in Xcode if required.

### Flutter on Real iOS Device
1. Enable Developer Mode on your iPhone (iOS Settings > Privacy & Security > Developer Mode).
2. In your Flutter project folder, run:
   ```
   flutter devices
   ```
   to confirm your iPhone is recognized.
3. Run:
   ```
   flutter run
   ```
   and select your iPhone. The Flutter app builds and installs on the device.

## Connecting the Flutter App with the BLE Device
- Once the Arduino board is powered on and running the BLE sketch, launch the Flutter app.
- The app should automatically discover and connect to the BLE device if permissions are granted.
- Make sure Bluetooth is enabled on your phone or emulator.

## Additional Notes
- Ensure Bluetooth is enabled on both the Arduino board (if supported) and your phone or emulator if you plan to interact via BLE.
- The Arduino code should be uploaded before testing the Flutter app’s BLE functionality.
- For a production iOS release, you must enroll in the Apple Developer Program and adhere to Apple’s guidelines.

