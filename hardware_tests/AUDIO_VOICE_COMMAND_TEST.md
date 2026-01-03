# Audio and Voice Command Test Documentation

This document describes the setup, testing, and configuration of audio devices and voice command recognition for the robot system.

## Date
December 7, 2024

---

## Overview

This system enables voice command recognition for robot control using:
- **Speech Recognition Library**: SpeechRecognition (Python)
- **Recognition Engine**: PocketSphinx (offline) or Google Speech-to-Text API (online)
- **Audio Capture**: PyAudio library or arecord command-line tool

---

## Hardware Requirements

### Microphone
- USB microphone (recommended)
- Or built-in microphone on Jetson
- USB 3.0 port for USB microphones

### Audio Devices Detected
- **Total devices**: 24 audio devices
- **Input devices**: 20 microphones detected
- **Device type**: NVIDIA Jetson Orin Nano APE (Audio Processing Engine)
- **Channels**: 16 channels per device
- **Sample rate**: 44100 Hz

---

## Prerequisites

### 1. Docker Configuration

Audio devices must be passed through to Docker. Configured in `~/.isaac_ros_dev-dockerargs`:

```
-v $HOME/.ngc:/home/admin/.ngc:ro
-v /dev/bus/usb:/dev/bus/usb
-v /sys/devices/platform:/sys/devices/platform
-v /dev/snd:/dev/snd
--group-add audio
```

### 2. Required Packages

Install audio libraries (auto-installed via entrypoint script):

```bash
sudo apt update
sudo apt install -y python3-pyaudio pocketsphinx pocketsphinx-en-us portaudio19-dev python3-dev
pip3 install SpeechRecognition
```

**Auto-installation**: The `01-install-audio-libs.sh` entrypoint script automatically installs these on container startup.

---

## Installation

### Step 1: Verify Audio Device Access

Check if audio devices are accessible:

```bash
python3 test_audio.py
```

**Expected Output**:
- Multiple audio devices detected
- Microphone accessible
- PocketSphinx engine available

### Step 2: Test Audio Recording

Test basic audio recording:

```bash
python3 test_record_simple.py
```

### Step 3: Test Speech Recognition

Test speech recognition with API:

```bash
python3 test_speech_api.py
```

---

## Testing Procedures

### Test 1: Audio Device Detection

Run the comprehensive audio test:

```bash
cd /workspaces/isaac_ros-dev
python3 test_audio.py
```

**What it tests**:
1. Audio device detection (SpeechRecognition library)
2. Audio device detection (PyAudio directly)
3. Microphone access
4. Speech recognition engine availability

**Expected Results**:
- ✅ Multiple audio devices detected (20+ devices)
- ✅ Microphone opened successfully
- ✅ PocketSphinx engine available
- ✅ Multiple recognition methods available

### Test 2: Speech Recognition (API-based)

Test speech recognition using Google Speech-to-Text API:

```bash
python3 test_speech_api.py
```

**What it does**:
1. Records audio for 5 seconds using `arecord` command
2. Sends audio to Google Speech-to-Text API
3. Displays recognized text

**Requirements**:
- Internet connection (for Google API)
- Microphone connected and working

**Note**: For offline recognition, use PocketSphinx (see Test 3).

### Test 3: Speech Recognition (Offline)

Test offline speech recognition with PocketSphinx:

```bash
python3 test_speech_recognition.py
```

**What it does**:
1. Records audio for 5 seconds
2. Processes with PocketSphinx (offline)
3. Displays recognized text

**Advantages**:
- Works offline (no internet required)
- Fast response
- Privacy (no data sent to cloud)

**Limitations**:
- Lower accuracy than cloud APIs
- Limited vocabulary
- May need voice training for best results

---

## Test Scripts

### 1. `test_audio.py`

Comprehensive audio device and library test.

**Usage**:
```bash
python3 test_audio.py
```

**Tests**:
- Audio device detection
- Microphone access
- Speech recognition engine availability

### 2. `test_speech_api.py`

Speech recognition using arecord + Google API.

**Usage**:
```bash
python3 test_speech_api.py
```

**Features**:
- Uses `arecord` for reliable recording
- Google Speech-to-Text API for recognition
- Requires internet connection

### 3. `test_speech_recognition.py`

Speech recognition using PocketSphinx (offline).

**Usage**:
```bash
python3 test_speech_recognition.py
```

**Features**:
- Offline recognition
- Uses PocketSphinx engine
- No internet required

### 4. `test_record_simple.py`

Simple audio recording test.

**Usage**:
```bash
python3 test_record_simple.py
```

**Purpose**: Verify basic audio recording works.

---

## Recognition Engines Available

The SpeechRecognition library supports multiple engines:

### Offline Engines
- **PocketSphinx** (`recognize_sphinx`) - Lightweight, offline
- **Vosk** (`recognize_vosk`) - Better accuracy, offline
- **Whisper** (`recognize_whisper`) - High accuracy, offline (requires model)

### Online Engines (Require Internet)
- **Google Speech-to-Text** (`recognize_google`) - High accuracy, free tier
- **Google Cloud** (`recognize_google_cloud`) - Requires API key
- **Azure** (`recognize_azure`) - Requires API key
- **Amazon** (`recognize_amazon`) - Requires API key
- **OpenAI Whisper API** (`recognize_openai`) - Requires API key

---

## Common Issues and Solutions

### Issue 1: No Audio Devices Detected

**Symptoms**: `No input devices found!`

**Solutions**:
1. Check Docker audio device mount: `-v /dev/snd:/dev/snd`
2. Verify audio group: `--group-add audio`
3. Check host audio devices: `ls -la /dev/snd/` (on host)
4. Restart container after adding audio mounts

### Issue 2: Permission Denied

**Error**: `Cannot access microphone: Permission denied`

**Solutions**:
1. Check user is in audio group: `groups | grep audio`
2. Set permissions: `sudo chmod 666 /dev/snd/*`
3. Verify Docker args include `--group-add audio`

### Issue 3: Recording Hangs

**Symptoms**: Script hangs during recording

**Solutions**:
1. Use `arecord` command-line tool instead of Python library
2. Try different microphone device index
3. Check microphone is not being used by another process
4. Use `test_speech_api.py` which uses `arecord`

### Issue 4: ALSA Warnings

**Symptoms**: Many ALSA library warnings

**Note**: ALSA warnings are normal and harmless. They occur because ALSA tries to find audio configurations that don't exist. The actual devices work fine.

### Issue 5: Speech Recognition Fails

**Symptoms**: `Could not understand audio`

**Solutions**:
- Speak more clearly and closer to microphone
- Reduce background noise
- Try different recognition engine (Google API vs PocketSphinx)
- Increase recording duration
- Check microphone is working (test with `test_record_simple.py`)

---

## Integration with ROS 2

### Future: Voice Command ROS 2 Node

A ROS 2 node can be created to:
1. Listen for voice commands
2. Recognize keywords (e.g., "forward", "stop", "turn left")
3. Publish commands to ROS topics (e.g., `/cmd_vel`)

**Example Architecture**:
```
Microphone → Audio Capture → Speech Recognition → Command Parser → ROS Topics
                                                                    ↓
                                                          /cmd_vel (robot movement)
                                                          /voice_command (custom)
```

---

## Auto-Installation Script

The `01-install-audio-libs.sh` entrypoint script automatically installs audio libraries:

```bash
#!/bin/bash
# Auto-install audio libraries for voice command support

if ! python3 -c "import speech_recognition" 2>/dev/null; then
    echo "Installing audio libraries for voice commands..."
    sudo apt update > /dev/null 2>&1
    sudo apt install -y python3-pyaudio pocketsphinx pocketsphinx-en-us portaudio19-dev python3-dev > /dev/null 2>&1
    pip3 install SpeechRecognition > /dev/null 2>&1
    echo "✓ Audio libraries installed"
else
    echo "✓ Audio libraries already installed"
fi
```

This ensures audio libraries are available even after container recreation.

---

## Test Results Summary

### Successful Test Output

**Audio Devices**:
- ✅ 20+ input devices (microphones) detected
- ✅ Microphone accessible
- ✅ Sample rate: 44100 Hz
- ✅ Channels: 16 per device

**Speech Recognition**:
- ✅ PocketSphinx engine available
- ✅ Multiple recognition methods available
- ✅ Google API accessible (with internet)

**Performance**:
- ✅ Recording works (with arecord)
- ✅ Recognition processing functional
- ⚠️ Python library recording can hang (use arecord instead)

---

## Next Steps

1. **Create ROS 2 voice command node** for robot control
2. **Implement keyword recognition** for simple commands
3. **Integrate with motor controller** for voice-controlled movement
4. **Add natural language processing** for complex commands
5. **Test with actual robot** for real-world validation

---

## References

- [SpeechRecognition Library](https://github.com/Uberi/speech_recognition)
- [PocketSphinx](https://github.com/cmusphinx/pocketsphinx)
- [Google Speech-to-Text API](https://cloud.google.com/speech-to-text)
- [PyAudio Documentation](https://people.csail.mit.edu/hubert/pyaudio/)

