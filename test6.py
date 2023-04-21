import sounddevice as sd
import numpy as np
import wavio
import noisereduce as nr

def record_and_save_multichannel_audio(num_mics=4, channels_per_mic=2, sampling_rate=44100, duration=10, sampwidth=3, noise_file="noise_audio.wav"):
    total_channels = num_mics * channels_per_mic

    def record_audio():
        print("Recording audio...")
        audio = sd.rec(frames=duration * sampling_rate,
                       samplerate=sampling_rate,
                       channels=total_channels,
                       blocking=True)
        return audio

    raw_recordings = record_audio()

    # Read the noise audio file
    noise_audio = wavio.read(noise_file).data

    # Specify the desired channel indices
    desired_channel_indices = [1, 4, 5, 7]

    # Loop through the desired channel indices
    for i, channel_idx in enumerate(desired_channel_indices):
        recording = raw_recordings[:, channel_idx]

        # Perform noise reduction using the separate noise audio file
        noise_reduced_recording = nr.reduce_noise(audio_clip=recording, noise_clip=noise_audio, verbose=False)

        output_file = f"PiTestMic{i + 1}.wav"
        wavio.write(output_file, noise_reduced_recording, sampling_rate, sampwidth)  # 3 bytes (24 bits) per sample
        print(f"Recording from channel {channel_idx + 1} saved to {output_file}")

    print("Recording complete.")

# Call the function with default parameters
record_and_save_multichannel_audio()
