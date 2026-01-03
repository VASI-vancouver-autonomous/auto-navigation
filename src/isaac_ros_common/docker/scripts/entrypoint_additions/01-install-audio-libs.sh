#!/bin/bash
# Auto-install audio libraries for voice command support
# This ensures audio libraries are available even after container recreation

# Check if SpeechRecognition is installed
if ! python3 -c "import speech_recognition" 2>/dev/null; then
    echo "Installing audio libraries for voice commands..."
    sudo apt update > /dev/null 2>&1
    sudo apt install -y python3-pyaudio pocketsphinx pocketsphinx-en-us portaudio19-dev python3-dev > /dev/null 2>&1
    pip3 install SpeechRecognition > /dev/null 2>&1
    echo "✓ Audio libraries installed"
else
    echo "✓ Audio libraries already installed"
fi

