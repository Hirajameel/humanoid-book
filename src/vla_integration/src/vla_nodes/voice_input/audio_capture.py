import pyaudio
import numpy as np
import threading
import time
from typing import Callable, Optional
from dataclasses import dataclass
from .config_manager import ConfigManager


@dataclass
class AudioData:
    """Represents captured audio data."""
    audio_data: bytes
    sample_rate: int
    channels: int
    duration: float


class AudioCaptureService:
    """
    Service for capturing real-time audio input from microphone.
    """
    def __init__(self, config_manager: ConfigManager):
        self.config = config_manager
        self.audio = pyaudio.PyAudio()

        # Get audio configuration
        self.sample_rate = self.config.get('audio.sample_rate', 16000)
        self.chunk_size = self.config.get('audio.chunk_size', 1024)
        self.channels = self.config.get('audio.channels', 1)
        self.device_index = self.config.get('audio.device_index', None)

        self.is_recording = False
        self.recording_thread = None
        self.audio_callback = None

    def set_audio_callback(self, callback: Callable[[AudioData], None]):
        """Set the callback function to handle captured audio data."""
        self.audio_callback = callback

    def start_recording(self):
        """Start recording audio from the microphone."""
        if self.is_recording:
            return

        # Open audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            input_device_index=self.device_index
        )

        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._record_audio)
        self.recording_thread.start()

        print(f"Started audio recording at {self.sample_rate}Hz, {self.channels} channel(s)")

    def stop_recording(self):
        """Stop recording audio."""
        if not self.is_recording:
            return

        self.is_recording = False

        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()

        if self.recording_thread:
            self.recording_thread.join()

        print("Stopped audio recording")

    def _record_audio(self):
        """Internal method to record audio in a separate thread."""
        while self.is_recording:
            try:
                # Read audio data from the stream
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Calculate duration of this chunk
                duration = len(data) / (self.sample_rate * self.channels * 2)  # 2 bytes per sample for paInt16

                # Create AudioData object
                audio_data = AudioData(
                    audio_data=data,
                    sample_rate=self.sample_rate,
                    channels=self.channels,
                    duration=duration
                )

                # Call the callback if it exists
                if self.audio_callback:
                    self.audio_callback(audio_data)

            except Exception as e:
                print(f"Error during audio recording: {e}")
                break

    def record_single_clip(self, duration: float = 5.0) -> AudioData:
        """Record a single audio clip for a specified duration."""
        frames = []

        # Open audio stream
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            input_device_index=self.device_index
        )

        # Record for the specified duration
        num_chunks = int((self.sample_rate * duration) / self.chunk_size)

        for _ in range(num_chunks):
            if not self.is_recording:  # Allow interruption
                break
            data = stream.read(self.chunk_size, exception_on_overflow=False)
            frames.append(data)

        # Close stream
        stream.stop_stream()
        stream.close()

        # Combine frames into single audio data
        combined_data = b''.join(frames)
        total_duration = duration

        return AudioData(
            audio_data=combined_data,
            sample_rate=self.sample_rate,
            channels=self.channels,
            duration=total_duration
        )

    def __del__(self):
        """Clean up audio resources."""
        if hasattr(self, 'audio'):
            self.audio.terminate()