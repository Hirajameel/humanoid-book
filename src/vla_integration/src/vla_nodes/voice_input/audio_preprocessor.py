import numpy as np
import pyaudio
import io
import wave
from typing import Tuple, Optional
from scipy import signal
from .config_manager import ConfigManager


class AudioPreprocessor:
    """
    Preprocesses audio data to improve speech recognition accuracy.
    """
    def __init__(self, config_manager: ConfigManager):
        self.config = config_manager
        self.sample_rate = self.config.get('audio.sample_rate', 16000)

    def preprocess_audio(self, audio_data: bytes) -> bytes:
        """
        Apply preprocessing to raw audio data to improve recognition quality.

        Args:
            audio_data: Raw audio bytes

        Returns:
            Preprocessed audio bytes
        """
        # Convert raw bytes to numpy array
        audio_array = self._bytes_to_array(audio_data)

        # Apply noise reduction
        audio_array = self._reduce_noise(audio_array)

        # Normalize audio
        audio_array = self._normalize(audio_array)

        # Convert back to bytes
        processed_audio = self._array_to_bytes(audio_array)

        return processed_audio

    def _bytes_to_array(self, audio_bytes: bytes) -> np.ndarray:
        """Convert raw audio bytes to numpy array."""
        # Convert bytes to numpy array (assuming 16-bit PCM)
        audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
        return audio_array.astype(np.float32)

    def _array_to_bytes(self, audio_array: np.ndarray) -> bytes:
        """Convert numpy array back to raw audio bytes."""
        # Ensure values are within int16 range
        audio_array = np.clip(audio_array, -32768, 32767)
        audio_array = audio_array.astype(np.int16)
        return audio_array.tobytes()

    def _reduce_noise(self, audio_array: np.ndarray) -> np.ndarray:
        """
        Apply basic noise reduction using spectral gating.
        This is a simplified approach - for production, consider more sophisticated methods.
        """
        # Simple noise reduction by applying a threshold
        threshold = np.std(audio_array) * 0.1  # 10% of standard deviation
        audio_array[np.abs(audio_array) < threshold] = 0

        return audio_array

    def _normalize(self, audio_array: np.ndarray) -> np.ndarray:
        """Normalize audio to a consistent level."""
        # Find the maximum absolute value
        max_val = np.max(np.abs(audio_array))

        # Avoid division by zero
        if max_val == 0:
            return audio_array

        # Normalize to -1 to 1 range
        normalized = audio_array / max_val

        # Scale to a target range (e.g., 80% of maximum)
        target_level = 0.8
        return normalized * target_level

    def apply_bandpass_filter(self, audio_array: np.ndarray, low_freq: float = 300, high_freq: float = 3400) -> np.ndarray:
        """
        Apply a bandpass filter to focus on human speech frequencies.

        Args:
            audio_array: Input audio array
            low_freq: Low frequency cutoff (Hz)
            high_freq: High frequency cutoff (Hz)

        Returns:
            Filtered audio array
        """
        nyquist = self.sample_rate / 2
        low = low_freq / nyquist
        high = high_freq / nyquist

        # Create a Butterworth bandpass filter
        b, a = signal.butter(4, [low, high], btype='band', analog=False)

        # Apply the filter
        filtered_audio = signal.filtfilt(b, a, audio_array)

        return filtered_audio

    def detect_silence(self, audio_array: np.ndarray, threshold: float = 0.01, min_duration: float = 0.5) -> bool:
        """
        Detect if the audio contains silence based on energy threshold.

        Args:
            audio_array: Input audio array
            threshold: Energy threshold for silence detection
            min_duration: Minimum duration of silence to consider (in seconds)

        Returns:
            True if silence is detected, False otherwise
        """
        # Calculate energy of the signal
        energy = np.mean(audio_array ** 2)

        # Calculate minimum number of samples for min_duration
        min_samples = int(min_duration * self.sample_rate)

        # For simplicity, just check if average energy is below threshold
        # In a more sophisticated implementation, you'd check for sustained silence
        return energy < threshold

    def preprocess_for_whisper(self, audio_data: bytes) -> bytes:
        """
        Apply preprocessing specifically optimized for Whisper API.

        Args:
            audio_data: Raw audio bytes

        Returns:
            Preprocessed audio bytes optimized for Whisper
        """
        # Convert to array
        audio_array = self._bytes_to_array(audio_data)

        # Apply bandpass filter for human speech
        audio_array = self.apply_bandpass_filter(audio_array)

        # Normalize
        audio_array = self._normalize(audio_array)

        # Convert back to bytes
        processed_audio = self._array_to_bytes(audio_array)

        return processed_audio