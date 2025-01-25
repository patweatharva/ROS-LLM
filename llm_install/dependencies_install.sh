#!/bin/bash

set -e

echo "Installing system dependencies..."
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3
sudo apt install -y python3-pip
sudo apt install -y gnome-terminal
sudo apt install -y libcanberra-gtk-module libcanberra-gtk3-module
sudo apt install -y portaudio19-dev
sudo apt install -y ffmpeg
sudo apt install -y libportaudio2
sudo apt install -y alsa-utils
sudo apt install -y mpv

echo "Installing Python packages..."
pip install openai
pip install pysocks
pip install requests
pip install boto3
pip install numpy
pip install sounddevice
pip install pydub
pip install scipy
pip install numpy sounddevice cffi soundfile
pip install python-dotenv

echo "Installing Ollama..."
if ! command -v ollama &> /dev/null; then
    echo "Ollama not found. Installing..."
    curl -fsSL https://ollama.com/install.sh | sh
else
    echo "Ollama is already installed"
fi

echo "Waiting for Ollama service to start..."
sleep 5

echo "Pulling Llama3 3B model..."
if ! ollama list | grep -q "llama3.2:3b"; then
    echo "Pulling llama3.2:3b model..."
    ollama pull llama3.2:3b
else
    echo "llama3.2:3b model already exists"
fi

echo "Creating custom Tiago model..."
cd "$(dirname "$0")"
ollama create tiago -f Modelfile

echo "Checking for updates one last time..."
sudo apt update
sudo apt upgrade -y

echo "All dependencies installed successfully!"
echo "Ollama is installed and custom Tiago model is ready to use."
echo "You can now use the 'tiago' model in your ROS node."
