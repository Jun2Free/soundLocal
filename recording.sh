#!/bin/bash

# Set the recording duration in seconds
DURATION=10

# Set the output file format 
FILE_FORMAT=wav

# Set the sample rate
SAMPLE_RATE=44100

audacity

# Start recording 8 channels
for i in {1..8}; do
	audacity --export-format ${FILE_FORMAT} --end ${DURATION} --output "recording_${i}.${FILE_FORMAT}" --rate ${SAMPLE_RATE} &
done

wait

# Remove noisy files
for i in 2 4 6 8; do
	rm "recording_${i}.${FILE_FORMAT}"
done

