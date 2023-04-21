import sounddevice as sd
import numpy as np
import wavio
import noisereduce as nr

# Configuration
num_mics = 4
channels_per_mic = 2
total_channels = num_mics * channels_per_mic
sampling_rate = 44100
duration = 5  # seconds

# Function to record audio simultaneously
def record_audio():
    # Background noise
    noise_file = "rotor_sound.wav"
    noise_audio_file=wavio.read(noise_file)
    noise_audio=noise_audio_file.data.astype(np.float32)

    print("Recording audio...")
    audio = sd.rec(frames=duration * sampling_rate,
                   samplerate=sampling_rate,
                   channels=total_channels,
                   blocking=True)
    return audio

# Recording audio
raw_recordings = record_audio()

# Specify the desired channel indices
desired_channel_indices = [1, 4, 5, 7]

# Extract the first channel from each microphone
for i, channel_idx in enumerate(desired_channel_indices):
    recording = raw_recordings[:,channel_idx]
    
    # Perform noise reduction using the separate noise audio file
    noise_reduced_recording = nr.reduce_noise(audio_clip=recording, noise_clip=noise_audio, verbose=False)

    output_file = f"PiTestMic{i + 1}.wav"
    wavio.write(output_file, recording, sampling_rate, sampwidth=4)
    print(f"Recording from microphone {channel_idx + 1} saved as PiTestMic{i + 1}")

print("Recording complete. Saved to output.wav")
