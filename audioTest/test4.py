import sounddevice as sd
import numpy as np
import wavio

# Configuration
num_mics = 4
channels_per_mic = 2
total_channels = num_mics * channels_per_mic
sampling_rate = 44100
duration = 5  # seconds

# Function to record audio simultaneously
def record_audio():
    print("Recording audio...")
    audio = sd.rec(frames=duration * sampling_rate,
                   samplerate=sampling_rate,
                   channels=total_channels,
                   blocking=True)
    return audio

# Recording audio
raw_recordings = record_audio()

# Extract the first channel from each microphone
recordings = []
for mic_idx in range(num_mics):
    for ch_idx in range(channels_per_mic):
        channel_idx = mic_idx * channels_per_mic + ch_idx
        print("channel_idx = ",channel_idx)
        recording = raw_recordings[:, channel_idx]
    
        # Save the recording to a WAV file
        # output_file = f"PiTestMic{mic_idx+1}_ch_{ch_idx + 1}.wav"
        output_file = f"PiTestMic{channel_idx+1}.wav"
        wavio.write(output_file, recording, sampling_rate, sampwidth=4)
        print(f"Recording from microphone {channel_idx +1} saved")

print("Recording complete. Saved to output.wav")
