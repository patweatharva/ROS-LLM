# Eleven Labs TTS ROS Node

This ROS package provides text-to-speech (TTS) functionality using the Eleven Labs API. The node listens to a text input topic and converts the received text into speech using Eleven Labs' high-quality AI voice synthesis.

## Installation Instructions

### 1. Install Dependencies

Before running the node, ensure you have installed the required dependencies:

```bash
pip install requests pydub rospkg
sudo apt install ffmpeg
```

### 2. Set Up the API key

To use this package, you must obtain an API key from [Eleven Labs](https://elevenlabs.io/).

Follow these steps to securely store your API key:

1. **Create the configuration directory** (if it doesn't already exist):

    ```bash
    cd ~/catkin_ws/src/ROS-LLM/llm_output
    mkdir -p config
    ```

2. **Save your API key** to a text file inside the `config` folder:

    ```bash
    echo "your-eleven-labs-api-key" > config/.elevenlabs_api_key
    ```

3. **Secure the key file** by changing its permissions to prevent unauthorized access:

    ```bash
    chmod 600 config/elevenlabs_api_key.txt
    ```

4. **(Optional) Ignore the key file in version control** to prevent it from being shared:

    ```bash
    echo "config/elevenlabs_api_key.txt" >> .gitignore
    ```
