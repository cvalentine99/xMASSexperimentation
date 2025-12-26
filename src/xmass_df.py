#!/usr/bin/env python3
"""
xMASS Direction Finding Module

Python implementation for multi-xSDR synchronization and direction finding
using the xMASS carrier board with 4 xSDR modules.

Copyright 2025 Wavelet Lab
License: MIT
"""

import numpy as np
import time
import json
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any
from enum import Enum

# Try to import SoapySDR - if not available, use mock
try:
    import SoapySDR
    from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32
    SOAPY_AVAILABLE = True
except ImportError:
    SOAPY_AVAILABLE = False
    print("[xMASS] Warning: SoapySDR not available, using mock mode")


class XMASSError(Exception):
    """Base exception for xMASS errors"""
    pass


class CalibrationError(XMASSError):
    """Calibration-related errors"""
    pass


class DeviceError(XMASSError):
    """Device communication errors"""
    pass


@dataclass
class CalibrationData:
    """Stores phase and amplitude calibration data"""
    phase_offsets: np.ndarray = field(default_factory=lambda: np.zeros(4))
    amplitude_corrections: np.ndarray = field(default_factory=lambda: np.ones(4))
    phase_stability: np.ndarray = field(default_factory=lambda: np.zeros(4))
    calibration_freq: float = 100e6
    timestamp: float = 0.0
    valid: bool = False
    count: int = 0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        return {
            'phase_offsets': self.phase_offsets.tolist(),
            'amplitude_corrections': self.amplitude_corrections.tolist(),
            'phase_stability': self.phase_stability.tolist(),
            'calibration_freq': self.calibration_freq,
            'timestamp': self.timestamp,
            'valid': self.valid,
            'count': self.count
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CalibrationData':
        """Create from dictionary"""
        return cls(
            phase_offsets=np.array(data['phase_offsets']),
            amplitude_corrections=np.array(data['amplitude_corrections']),
            phase_stability=np.array(data['phase_stability']),
            calibration_freq=data['calibration_freq'],
            timestamp=data['timestamp'],
            valid=data['valid'],
            count=data['count']
        )


class XMASSSync:
    """
    Multi-xSDR Synchronization Manager
    
    Manages 4 xSDR modules on the xMASS carrier board for coherent
    direction finding applications.
    """
    
    def __init__(self, num_devices: int = 4):
        """
        Initialize the synchronization manager.
        
        Args:
            num_devices: Number of xSDR devices (default 4)
        """
        self.num_devices = num_devices
        self.devices: List[Any] = []
        self.streams: List[Any] = []
        self.calibration = CalibrationData()
        
        # Reference clock from carrier board (read-only)
        self.ref_clock = 26e6  # 26 MHz from LMK05318B
        
        # State
        self.initialized = False
        self.streaming = False
        
    def open_devices(self) -> None:
        """Open all xSDR devices."""
        print(f"[xMASS] Opening {self.num_devices} xSDR devices...")
        
        if not SOAPY_AVAILABLE:
            # Mock mode
            self.devices = [{'index': i, 'mock': True} for i in range(self.num_devices)]
            self.initialized = True
            print(f"[xMASS] Mock mode: {self.num_devices} virtual devices created")
            return
        
        # Enumerate devices
        results = SoapySDR.Device.enumerate("driver=usdr")
        if len(results) < self.num_devices:
            raise DeviceError(f"Only found {len(results)} devices, need {self.num_devices}")
        
        # Open each device
        for i in range(self.num_devices):
            try:
                dev = SoapySDR.Device(dict(driver="usdr", index=str(i)))
                self.devices.append(dev)
                print(f"  [+] Opened usdr{i}")
            except Exception as e:
                raise DeviceError(f"Failed to open usdr{i}: {e}")
        
        self.initialized = True
        print(f"[xMASS] Successfully opened {len(self.devices)} devices")
    
    def verify_reference_clock(self) -> Dict[int, float]:
        """
        Verify reference clock on all devices.
        
        Returns:
            Dictionary mapping device index to clock frequency
        """
        print("[xMASS] Verifying reference clock on all devices...")
        
        clocks = {}
        for i, dev in enumerate(self.devices):
            if isinstance(dev, dict) and dev.get('mock'):
                clocks[i] = self.ref_clock
                print(f"  usdr{i}: {self.ref_clock/1e6:.2f} MHz (mock)")
            else:
                master_clk = dev.getMasterClockRate()
                clocks[i] = master_clk
                print(f"  usdr{i}: Master clock = {master_clk/1e6:.2f} MHz")
        
        print("[xMASS] Note: LMK05318B configuration is pre-set by carrier board")
        return clocks
    
        def set_lo_frequency(self, frequency: float) -> None:
        """
        Set the common LO frequency for all devices.

        Args:
            frequency: LO frequency in Hz
        """
        print(f"[xMASS] Setting LO frequency to {frequency/1e6:.3f} MHz")
        # In a real implementation, this would call the C function
        # xmass_carrier_set_lo_frequency. For the mock, we just store it.
        self.lo_freq = frequency

    def configure_all(self, frequency: float, sample_rate: float,
                      bandwidth: Optional[float] = None, gain: float = 30.0) -> None:
        """
        Configure all xSDRs with identical settings.
        
        Args:
            frequency: Center frequency in Hz
            sample_rate: Sample rate in Hz
            bandwidth: Bandwidth in Hz (defaults to sample_rate)
            gain: Gain in dB
        """
        print(f"[xMASS] Configuring all devices:")
        print(f"    Frequency: {frequency/1e6:.3f} MHz")
        print(f"    Sample Rate: {sample_rate/1e6:.3f} MSPS")
        print(f"    Gain: {gain} dB")
        
        if bandwidth is None:
            bandwidth = sample_rate
        
        for i, dev in enumerate(self.devices):
            if isinstance(dev, dict) and dev.get('mock'):
                print(f"  usdr{i}: {frequency/1e6:.3f} MHz, {sample_rate/1e6:.3f} MSPS, {gain:.1f} dB (mock)")
                continue
            
            # Set frequency
            dev.setFrequency(SOAPY_SDR_RX, 0, frequency)
            actual_freq = dev.getFrequency(SOAPY_SDR_RX, 0)
            
            # Set sample rate
            dev.setSampleRate(SOAPY_SDR_RX, 0, sample_rate)
            actual_rate = dev.getSampleRate(SOAPY_SDR_RX, 0)
            
            # Set bandwidth
            dev.setBandwidth(SOAPY_SDR_RX, 0, bandwidth)
            
            # Set gain
            dev.setGain(SOAPY_SDR_RX, 0, gain)
            actual_gain = dev.getGain(SOAPY_SDR_RX, 0)
            
            print(f"  usdr{i}: {actual_freq/1e6:.3f} MHz, {actual_rate/1e6:.3f} MSPS, {actual_gain:.1f} dB")
        
        print("[xMASS] All devices configured")
    
    def setup_streams(self) -> None:
        """Setup RX streams on all devices."""
        print("[xMASS] Setting up RX streams...")
        
        self.streams = []
        for i, dev in enumerate(self.devices):
            if isinstance(dev, dict) and dev.get('mock'):
                self.streams.append({'index': i, 'mock': True})
                print(f"  [+] Stream configured for usdr{i} (mock)")
                continue
            
            stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
            self.streams.append(stream)
            print(f"  [+] Stream configured for usdr{i}")
        
        print("[xMASS] All streams ready")
    
    def start_streaming(self) -> None:
        """Start streaming on all devices simultaneously."""
        print("[xMASS] Starting streaming on all devices...")
        
        for i, (dev, stream) in enumerate(zip(self.devices, self.streams)):
            if isinstance(dev, dict) and dev.get('mock'):
                continue
            dev.activateStream(stream)
        
        time.sleep(0.1)  # Settle time
        self.streaming = True
        print("[xMASS] All devices streaming")
    
    def stop_streaming(self) -> None:
        """Stop streaming on all devices."""
        print("[xMASS] Stopping streaming...")
        
        for i, (dev, stream) in enumerate(zip(self.devices, self.streams)):
            if isinstance(dev, dict) and dev.get('mock'):
                continue
            dev.deactivateStream(stream)
        
        self.streaming = False
        print("[xMASS] Streaming stopped")
    
    def capture_samples(self, num_samples: int = 4096) -> List[np.ndarray]:
        """
        Capture samples from all devices simultaneously.
        
        Args:
            num_samples: Number of samples to capture
            
        Returns:
            List of complex sample arrays, one per device
        """
        buffers = []
        
        for i, (dev, stream) in enumerate(zip(self.devices, self.streams)):
            buffer = np.zeros(num_samples, dtype=np.complex64)
            
            if isinstance(dev, dict) and dev.get('mock'):
                # Generate mock data with phase offset
                t = np.arange(num_samples) / 1e6
                phase_offset = i * np.pi / 4  # 45° offset per channel
                buffer = np.exp(1j * (2 * np.pi * 1000 * t + phase_offset)).astype(np.complex64)
                buffer += 0.1 * (np.random.randn(num_samples) + 1j * np.random.randn(num_samples)).astype(np.complex64)
                buffers.append(buffer)
                continue
            
            sr = dev.readStream(stream, [buffer], num_samples, timeoutUs=1000000)
            
            if sr.ret > 0:
                buffers.append(buffer[:sr.ret])
            else:
                print(f"  [!] usdr{i}: Read failed with code {sr.ret}")
                buffers.append(np.zeros(num_samples, dtype=np.complex64))
        
        return buffers
    
    def calibrate_phase(self, calibration_freq: float = 100e6,
                        num_samples: int = 8192) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calibrate phase offsets between devices using a known signal.
        
        Args:
            calibration_freq: Calibration signal frequency in Hz
            num_samples: Number of samples for calibration
            
        Returns:
            Tuple of (phase_offsets, amplitude_corrections)
            
        Note:
            Requires a strong CW signal at calibration_freq
        """
        print(f"[xMASS] Starting phase calibration at {calibration_freq/1e6:.1f} MHz...")
        print("    Ensure a strong CW signal is present at this frequency!")
        
        # Configure for calibration
        self.set_lo_frequency(calibration_freq)
        self.configure_all(0, sample_rate=1e6, gain=40)
        self.setup_streams()
        self.start_streaming()
        
        # Capture multiple snapshots for averaging
        num_snapshots = 10
        phase_measurements = np.zeros((num_snapshots, self.num_devices))
        amplitude_measurements = np.zeros((num_snapshots, self.num_devices))
        
        print(f"[xMASS] Capturing {num_snapshots} snapshots for calibration...")
        
        for snapshot in range(num_snapshots):
            buffers = self.capture_samples(num_samples)
            
            for i in range(self.num_devices):
                if i == 0:
                    phase_measurements[snapshot, 0] = 0.0
                    amplitude_measurements[snapshot, 0] = np.abs(np.mean(buffers[0]))
                else:
                    # Cross-correlation with reference (device 0)
                    correlation = np.mean(buffers[i] * np.conj(buffers[0]))
                    phase_measurements[snapshot, i] = np.angle(correlation)
                    amplitude_measurements[snapshot, i] = np.abs(np.mean(buffers[i]))
        
        self.stop_streaming()
        
        # Average measurements
        self.calibration.phase_offsets = np.mean(phase_measurements, axis=0)
        avg_amplitudes = np.mean(amplitude_measurements, axis=0)
        
        # Compute amplitude corrections (normalize to device 0)
        self.calibration.amplitude_corrections = avg_amplitudes[0] / avg_amplitudes
        
        # Calculate stability (std deviation across snapshots)
        self.calibration.phase_stability = np.std(phase_measurements, axis=0)
        
        # Update calibration metadata
        self.calibration.calibration_freq = calibration_freq
        self.calibration.timestamp = time.time()
        self.calibration.valid = True
        self.calibration.count += 1
        
        # Report results
        print("\n[xMASS] Calibration complete:")
        print("    Device | Phase Offset | Amplitude Correction | Stability")
        print("    -------|--------------|----------------------|----------")
        for i in range(self.num_devices):
            print(f"    usdr{i}  | {np.degrees(self.calibration.phase_offsets[i]):>7.2f}°    | "
                  f"{self.calibration.amplitude_corrections[i]:>6.3f}x              | "
                  f"±{np.degrees(self.calibration.phase_stability[i]):.2f}°")
        
        return self.calibration.phase_offsets, self.calibration.amplitude_corrections
    
    def apply_corrections(self, buffers: List[np.ndarray]) -> List[np.ndarray]:
        """
        Apply phase and amplitude corrections to captured samples.
        
        Args:
            buffers: List of sample arrays to correct
            
        Returns:
            List of corrected sample arrays
        """
        if not self.calibration.valid:
            raise CalibrationError("Calibration not valid")
        
        corrected = []
        for i, buffer in enumerate(buffers):
            # Apply amplitude correction
            corrected_buffer = buffer * self.calibration.amplitude_corrections[i]
            
            # Apply phase correction (rotate by negative of measured offset)
            phase_correction = np.exp(-1j * self.calibration.phase_offsets[i])
            corrected_buffer = corrected_buffer * phase_correction
            
            corrected.append(corrected_buffer)
        
        return corrected
    
    def save_calibration(self, filename: str) -> None:
        """Save calibration data to file."""
        with open(filename, 'w') as f:
            json.dump(self.calibration.to_dict(), f, indent=2)
        print(f"[xMASS] Calibration saved to {filename}")
    
    def load_calibration(self, filename: str) -> None:
        """Load calibration data from file."""
        with open(filename, 'r') as f:
            data = json.load(f)
        self.calibration = CalibrationData.from_dict(data)
        print(f"[xMASS] Calibration loaded from {filename}")
    
    def close_all(self) -> None:
        """Close all devices."""
        print("[xMASS] Closing all devices...")
        
        # Close streams
        for dev, stream in zip(self.devices, self.streams):
            if isinstance(dev, dict) and dev.get('mock'):
                continue
            try:
                dev.closeStream(stream)
            except:
                pass
        
        self.devices.clear()
        self.streams.clear()
        self.initialized = False
        
        print("[xMASS] All devices closed")


class DirectionFinder:
    """
    Direction Finding using 4-element antenna array.
    
    Implements phase comparison and MUSIC algorithms for
    direction of arrival estimation.
    """
    
    SPEED_OF_LIGHT = 299792458.0
    
    def __init__(self, element_spacing: float = 0.5, array_orientation: float = 0.0):
        """
        Initialize direction finder.
        
        Args:
            element_spacing: Antenna spacing in wavelengths (0.5 = λ/2)
            array_orientation: Array orientation in degrees (0 = north)
        """
        self.element_spacing = element_spacing
        self.array_orientation = array_orientation
        self.sync = XMASSSync(num_devices=4)
        
    def initialize(self, calibration_freq: float = 100e6) -> None:
        """Initialize hardware and calibrate."""
        print("=== xMASS Direction Finder Initialization ===\n")
        
        self.sync.open_devices()
        self.sync.verify_reference_clock()
        
        print(f"\n=== CALIBRATION REQUIRED ===")
        print(f"Place a signal source at {calibration_freq/1e6:.1f} MHz")
        print("Signal should be strong (>-40 dBm) and in known direction")
        
        self.sync.calibrate_phase(calibration_freq=calibration_freq)
        
        print("\n[+] Initialization complete")
    
    def phase_comparison_doa(self, buffers: List[np.ndarray],
                             frequency: float) -> float:
        """
        Simple phase comparison DOA.
        
        Uses phase difference between elements 0 and 1 to estimate bearing.
        
        Args:
            buffers: Corrected sample buffers
            frequency: Signal frequency in Hz
            
        Returns:
            Estimated bearing in degrees
        """
        wavelength = self.SPEED_OF_LIGHT / frequency
        d = self.element_spacing * wavelength  # Physical spacing in meters
        k = 2 * np.pi / wavelength
        
        # Compute phase difference between element 0 and 1
        correlation = np.mean(buffers[0] * np.conj(buffers[1]))
        phase_diff = np.angle(correlation)
        
        # DOA calculation: sin(θ) = Δφ / (k * d)
        sin_theta = phase_diff / (k * d)
        
        # Clamp to valid range
        sin_theta = np.clip(sin_theta, -1.0, 1.0)
        
        # Angle in radians, convert to degrees
        theta_rad = np.arcsin(sin_theta)
        theta_deg = np.degrees(theta_rad)
        
        # Adjust for array orientation
        bearing = (theta_deg + self.array_orientation) % 360
        
        return bearing
    
    def music_doa(self, buffers: List[np.ndarray], frequency: float,
                  num_sources: int = 1) -> Tuple[float, np.ndarray, np.ndarray]:
        """
        MUSIC algorithm for DOA estimation.
        
        More accurate than phase comparison, but requires knowing
        number of sources.
        
        Args:
            buffers: Corrected sample buffers
            frequency: Signal frequency in Hz
            num_sources: Number of signal sources
            
        Returns:
            Tuple of (bearing, angles, spectrum)
        """
        # Stack samples into matrix (rows = antennas, cols = samples)
        X = np.vstack([buf for buf in buffers])
        
        # Compute covariance matrix
        R = (X @ X.conj().T) / X.shape[1]
        
        # Eigendecomposition
        eigenvalues, eigenvectors = np.linalg.eigh(R)
        
        # Sort by eigenvalue (descending)
        idx = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Noise subspace (all but first num_sources eigenvectors)
        noise_subspace = eigenvectors[:, num_sources:]
        
        # Scan angles from -90° to +90°
        angles = np.linspace(-90, 90, 1801)  # 0.1° resolution
        spectrum = np.zeros(len(angles))
        
        wavelength = self.SPEED_OF_LIGHT / frequency
        d = self.element_spacing * wavelength
        k = 2 * np.pi / wavelength
        
        for i, theta_deg in enumerate(angles):
            theta = np.radians(theta_deg)
            
            # Steering vector for this angle
            a = np.exp(-1j * k * d * np.arange(4) * np.sin(theta))
            
            # MUSIC pseudo-spectrum
            denominator = a.conj().T @ noise_subspace @ noise_subspace.conj().T @ a
            spectrum[i] = 1.0 / np.abs(denominator)
        
        # Find peak
        peak_idx = np.argmax(spectrum)
        estimated_angle = angles[peak_idx]
        
        # Adjust for array orientation
        bearing = (estimated_angle + self.array_orientation) % 360
        
        return bearing, angles, spectrum
    
    def find_bearing(self, frequency: float, sample_rate: float = 1e6,
                     num_samples: int = 8192, method: str = 'music') -> float:
        """
        Find bearing of signal at specified frequency.
        
        Args:
            frequency: Target frequency in Hz
            sample_rate: Sample rate in Hz
            num_samples: Number of samples to capture
            method: 'phase' or 'music'
            
        Returns:
            Estimated bearing in degrees
        """
        # Configure all devices
        self.sync.configure_all(frequency, sample_rate, gain=40)
        self.sync.setup_streams()
        self.sync.start_streaming()
        
        # Capture samples
        buffers = self.sync.capture_samples(num_samples)
        
        # Apply calibration corrections
        corrected_buffers = self.sync.apply_corrections(buffers)
        
        self.sync.stop_streaming()
        
        # Compute DOA
        if method == 'phase':
            bearing = self.phase_comparison_doa(corrected_buffers, frequency)
            return bearing
        elif method == 'music':
            bearing, angles, spectrum = self.music_doa(corrected_buffers, frequency)
            return bearing
        else:
            raise ValueError(f"Unknown method: {method}")
    
    def close(self) -> None:
        """Cleanup."""
        self.sync.close_all()


def main():
    """Example usage of xMASS direction finding."""
    print("=== xMASS Multi-xSDR Direction Finding ===\n")
    
    # Create synchronization manager
    sync = XMASSSync(num_devices=4)
    
    try:
        # Initialize
        sync.open_devices()
        sync.verify_reference_clock()
        
        # Calibrate
        print("\n=== CALIBRATION MODE ===")
        print("Place a CW signal generator at 100.0 MHz before continuing!")
        print("(Press Enter to continue with mock data...)")
        input()
        
        sync.calibrate_phase(calibration_freq=100e6)
        
        # Normal operation
        print("\n=== NORMAL OPERATION ===")
        sync.configure_all(frequency=100e6, sample_rate=1e6, gain=30)
        sync.setup_streams()
        sync.start_streaming()
        
        # Capture and process
        print("[*] Capturing synchronized samples...")
        buffers = sync.capture_samples(num_samples=4096)
        
        # Apply corrections
        corrected_buffers = sync.apply_corrections(buffers)
        
        print(f"[+] Captured {len(corrected_buffers[0])} samples from each device")
        
        # Verify coherence
        print("\n[*] Verifying phase coherence:")
        for i in range(1, len(corrected_buffers)):
            correlation = np.mean(corrected_buffers[i] * np.conj(corrected_buffers[0]))
            phase_diff = np.angle(correlation)
            print(f"    usdr{i} vs usdr0: {np.degrees(phase_diff):>6.2f}°")
        
        sync.stop_streaming()
        
        # Save calibration
        sync.save_calibration("/tmp/xmass_calibration.json")
        
    finally:
        sync.close_all()
    
    print("\n[+] Test complete!")
    
    # Direction finding example
    print("\n=== DIRECTION FINDING EXAMPLE ===\n")
    
    df = DirectionFinder(element_spacing=0.5, array_orientation=0.0)
    
    try:
        df.initialize(calibration_freq=100e6)
        
        print("\n=== DIRECTION FINDING ===")
        target_freq = 150e6
        print(f"[*] Finding bearing of signal at {target_freq/1e6:.1f} MHz...")
        
        # Phase comparison method
        bearing_phase = df.find_bearing(target_freq, method='phase')
        print(f"[+] Phase comparison: {bearing_phase:.1f}°")
        
        # MUSIC method
        bearing_music = df.find_bearing(target_freq, method='music')
        print(f"[+] MUSIC algorithm: {bearing_music:.1f}°")
        
        # Average
        bearing_avg = (bearing_phase + bearing_music) / 2.0
        print(f"[+] Average bearing: {bearing_avg:.1f}°")
        
    finally:
        df.close()
    
    print("\n[+] Direction finding complete!")


if __name__ == "__main__":
    main()
