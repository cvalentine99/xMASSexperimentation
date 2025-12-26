#!/usr/bin/env python3
"""
xMASS SoapySDR Integration Module

Provides SoapySDR-based access to xSDR devices with synchronization support.

Copyright 2025 Wavelet Lab
License: MIT
"""

import numpy as np
import time
import threading
from typing import List, Optional, Dict, Any, Tuple
from dataclasses import dataclass
from enum import Enum

# Try to import SoapySDR
try:
    import SoapySDR
    from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX, SOAPY_SDR_CF32, SOAPY_SDR_CS16
    SOAPY_AVAILABLE = True
except ImportError:
    SOAPY_AVAILABLE = False


class SyncMode(Enum):
    """Synchronization modes"""
    NONE = 0            # No synchronization
    SOFTWARE = 1        # Software-based synchronization
    HARDWARE = 2        # Hardware SYNC via LMK05318B
    TIMESTAMP = 3       # Timestamp-based synchronization


@dataclass
class DeviceInfo:
    """Information about a detected xSDR device"""
    index: int
    driver: str
    serial: str
    label: str
    hardware_key: str
    hardware_info: Dict[str, str]


class XMASSDevice:
    """
    Wrapper for a single xSDR device with SoapySDR.
    """
    
    def __init__(self, index: int = 0):
        """
        Initialize device wrapper.
        
        Args:
            index: Device index (0-3 for xMASS)
        """
        self.index = index
        self.device = None
        self.rx_stream = None
        self.tx_stream = None
        self._lock = threading.Lock()
        
    def open(self, driver: str = "usdr") -> bool:
        """
        Open the device.
        
        Args:
            driver: SoapySDR driver name
            
        Returns:
            True if successful
        """
        if not SOAPY_AVAILABLE:
            raise RuntimeError("SoapySDR not available")
        
        try:
            args = dict(driver=driver, index=str(self.index))
            self.device = SoapySDR.Device(args)
            return True
        except Exception as e:
            print(f"[xMASS] Failed to open device {self.index}: {e}")
            return False
    
    def close(self) -> None:
        """Close the device."""
        with self._lock:
            if self.rx_stream:
                try:
                    self.device.closeStream(self.rx_stream)
                except:
                    pass
                self.rx_stream = None
            
            if self.tx_stream:
                try:
                    self.device.closeStream(self.tx_stream)
                except:
                    pass
                self.tx_stream = None
            
            self.device = None
    
    def get_info(self) -> Dict[str, Any]:
        """Get device information."""
        if not self.device:
            return {}
        
        info = {
            'driver': self.device.getDriverKey(),
            'hardware': self.device.getHardwareKey(),
            'hardware_info': dict(self.device.getHardwareInfo()),
            'num_rx_channels': self.device.getNumChannels(SOAPY_SDR_RX),
            'num_tx_channels': self.device.getNumChannels(SOAPY_SDR_TX),
        }
        
        # Get frequency ranges
        try:
            freq_ranges = self.device.getFrequencyRange(SOAPY_SDR_RX, 0)
            if freq_ranges:
                info['freq_min'] = freq_ranges[0].minimum()
                info['freq_max'] = freq_ranges[-1].maximum()
        except:
            pass
        
        # Get sample rate ranges
        try:
            rate_ranges = self.device.getSampleRateRange(SOAPY_SDR_RX, 0)
            if rate_ranges:
                info['rate_min'] = rate_ranges[0].minimum()
                info['rate_max'] = rate_ranges[-1].maximum()
        except:
            pass
        
        return info
    
    def configure_rx(self, frequency: float, sample_rate: float,
                     bandwidth: Optional[float] = None, gain: float = 30.0,
                     channel: int = 0) -> None:
        """
        Configure RX channel.
        
        Args:
            frequency: Center frequency in Hz
            sample_rate: Sample rate in Hz
            bandwidth: Bandwidth in Hz (defaults to sample_rate)
            gain: Gain in dB
            channel: Channel number
        """
        if not self.device:
            raise RuntimeError("Device not open")
        
        with self._lock:
            self.device.setFrequency(SOAPY_SDR_RX, channel, frequency)
            self.device.setSampleRate(SOAPY_SDR_RX, channel, sample_rate)
            
            if bandwidth is None:
                bandwidth = sample_rate
            self.device.setBandwidth(SOAPY_SDR_RX, channel, bandwidth)
            
            self.device.setGain(SOAPY_SDR_RX, channel, gain)
    
    def setup_rx_stream(self, format_str: str = SOAPY_SDR_CF32,
                        channels: List[int] = None) -> None:
        """
        Setup RX stream.
        
        Args:
            format_str: Sample format (CF32 or CS16)
            channels: List of channel numbers
        """
        if not self.device:
            raise RuntimeError("Device not open")
        
        if channels is None:
            channels = [0]
        
        with self._lock:
            if self.rx_stream:
                self.device.closeStream(self.rx_stream)
            
            self.rx_stream = self.device.setupStream(SOAPY_SDR_RX, format_str, channels)
    
    def activate_rx(self, flags: int = 0, time_ns: int = 0,
                    num_elems: int = 0) -> None:
        """Activate RX stream."""
        if not self.rx_stream:
            raise RuntimeError("RX stream not setup")
        
        self.device.activateStream(self.rx_stream, flags, time_ns, num_elems)
    
    def deactivate_rx(self) -> None:
        """Deactivate RX stream."""
        if self.rx_stream:
            self.device.deactivateStream(self.rx_stream)
    
    def read_stream(self, num_samples: int, timeout_us: int = 1000000) -> Tuple[np.ndarray, int, int]:
        """
        Read samples from RX stream.
        
        Args:
            num_samples: Number of samples to read
            timeout_us: Timeout in microseconds
            
        Returns:
            Tuple of (samples, flags, time_ns)
        """
        if not self.rx_stream:
            raise RuntimeError("RX stream not setup")
        
        buffer = np.zeros(num_samples, dtype=np.complex64)
        
        sr = self.device.readStream(self.rx_stream, [buffer], num_samples, timeoutUs=timeout_us)
        
        if sr.ret > 0:
            return buffer[:sr.ret], sr.flags, sr.timeNs
        else:
            return np.array([], dtype=np.complex64), sr.flags, 0
    
    def get_master_clock(self) -> float:
        """Get master clock rate."""
        if not self.device:
            return 0.0
        return self.device.getMasterClockRate()
    
    def set_master_clock(self, rate: float) -> None:
        """Set master clock rate."""
        if not self.device:
            raise RuntimeError("Device not open")
        self.device.setMasterClockRate(rate)
    
    def get_clock_source(self) -> str:
        """Get clock source."""
        if not self.device:
            return ""
        return self.device.getClockSource()
    
    def set_clock_source(self, source: str) -> None:
        """Set clock source (internal/external)."""
        if not self.device:
            raise RuntimeError("Device not open")
        self.device.setClockSource(source)


class XMASSArray:
    """
    Multi-device array manager for xMASS.
    
    Manages 4 xSDR devices as a coherent array for direction finding.
    """
    
    def __init__(self, num_devices: int = 4):
        """
        Initialize array manager.
        
        Args:
            num_devices: Number of devices (default 4)
        """
        self.num_devices = num_devices
        self.devices: List[XMASSDevice] = []
        self.sync_mode = SyncMode.SOFTWARE
        self._lock = threading.Lock()
        
    @staticmethod
    def enumerate_devices(driver: str = "usdr") -> List[DeviceInfo]:
        """
        Enumerate available xSDR devices.
        
        Args:
            driver: SoapySDR driver name
            
        Returns:
            List of DeviceInfo objects
        """
        if not SOAPY_AVAILABLE:
            return []
        
        results = SoapySDR.Device.enumerate(f"driver={driver}")
        
        devices = []
        for i, result in enumerate(results):
            info = DeviceInfo(
                index=i,
                driver=result.get('driver', driver),
                serial=result.get('serial', ''),
                label=result.get('label', f'{driver}{i}'),
                hardware_key=result.get('hardware', ''),
                hardware_info=dict(result)
            )
            devices.append(info)
        
        return devices
    
    def open_all(self, driver: str = "usdr") -> int:
        """
        Open all devices.
        
        Args:
            driver: SoapySDR driver name
            
        Returns:
            Number of devices opened
        """
        print(f"[xMASS Array] Opening {self.num_devices} devices...")
        
        self.devices = []
        count = 0
        
        for i in range(self.num_devices):
            dev = XMASSDevice(index=i)
            if dev.open(driver):
                self.devices.append(dev)
                count += 1
                print(f"  [+] Opened {driver}{i}")
            else:
                print(f"  [!] Failed to open {driver}{i}")
        
        print(f"[xMASS Array] Opened {count}/{self.num_devices} devices")
        return count
    
    def close_all(self) -> None:
        """Close all devices."""
        print("[xMASS Array] Closing all devices...")
        
        for dev in self.devices:
            dev.close()
        
        self.devices.clear()
        print("[xMASS Array] All devices closed")
    
    def configure_all(self, frequency: float, sample_rate: float,
                      bandwidth: Optional[float] = None, gain: float = 30.0) -> None:
        """
        Configure all devices with identical settings.
        
        Args:
            frequency: Center frequency in Hz
            sample_rate: Sample rate in Hz
            bandwidth: Bandwidth in Hz
            gain: Gain in dB
        """
        print(f"[xMASS Array] Configuring all devices:")
        print(f"    Frequency: {frequency/1e6:.3f} MHz")
        print(f"    Sample Rate: {sample_rate/1e6:.3f} MSPS")
        print(f"    Gain: {gain:.1f} dB")
        
        for i, dev in enumerate(self.devices):
            dev.configure_rx(frequency, sample_rate, bandwidth, gain)
            print(f"  [+] Configured device {i}")
    
    def setup_streams(self) -> None:
        """Setup RX streams on all devices."""
        print("[xMASS Array] Setting up streams...")
        
        for i, dev in enumerate(self.devices):
            dev.setup_rx_stream()
            print(f"  [+] Stream ready for device {i}")
    
    def start_streaming(self, sync_mode: SyncMode = SyncMode.SOFTWARE) -> None:
        """
        Start streaming on all devices.
        
        Args:
            sync_mode: Synchronization mode
        """
        print(f"[xMASS Array] Starting streaming (mode: {sync_mode.name})...")
        
        self.sync_mode = sync_mode
        
        if sync_mode == SyncMode.TIMESTAMP:
            # Calculate future timestamp for synchronized start
            # Get current hardware time from first device
            # Add offset for all devices to start at same time
            future_time_ns = int(time.time() * 1e9) + int(100e6)  # 100ms in future
            
            for dev in self.devices:
                dev.activate_rx(flags=SoapySDR.SOAPY_SDR_HAS_TIME, time_ns=future_time_ns)
        else:
            # Sequential activation
            for dev in self.devices:
                dev.activate_rx()
        
        time.sleep(0.1)  # Settle time
        print("[xMASS Array] All devices streaming")
    
    def stop_streaming(self) -> None:
        """Stop streaming on all devices."""
        print("[xMASS Array] Stopping streaming...")
        
        for dev in self.devices:
            dev.deactivate_rx()
        
        print("[xMASS Array] Streaming stopped")
    
    def capture_synchronized(self, num_samples: int = 4096,
                             timeout_us: int = 1000000) -> List[np.ndarray]:
        """
        Capture synchronized samples from all devices.
        
        Args:
            num_samples: Number of samples per device
            timeout_us: Timeout in microseconds
            
        Returns:
            List of sample arrays, one per device
        """
        buffers = []
        timestamps = []
        
        # Read from all devices
        for dev in self.devices:
            samples, flags, time_ns = dev.read_stream(num_samples, timeout_us)
            buffers.append(samples)
            timestamps.append(time_ns)
        
        # If using timestamp mode, verify alignment
        if self.sync_mode == SyncMode.TIMESTAMP and timestamps[0] > 0:
            max_diff = max(timestamps) - min(timestamps)
            if max_diff > 1000:  # More than 1µs difference
                print(f"[xMASS Array] Warning: Timestamp skew = {max_diff} ns")
        
        return buffers
    
    def get_clock_info(self) -> Dict[int, Dict[str, Any]]:
        """
        Get clock information from all devices.
        
        Returns:
            Dictionary mapping device index to clock info
        """
        info = {}
        
        for i, dev in enumerate(self.devices):
            info[i] = {
                'master_clock': dev.get_master_clock(),
                'clock_source': dev.get_clock_source()
            }
        
        return info
    
    def set_external_clock(self) -> None:
        """
        Set all devices to use external reference clock.
        
        This should be used when the LMK05318B is providing
        synchronized clocks to all devices.
        """
        print("[xMASS Array] Setting external clock reference...")
        
        for i, dev in enumerate(self.devices):
            try:
                dev.set_clock_source("external")
                print(f"  [+] Device {i}: external clock")
            except Exception as e:
                print(f"  [!] Device {i}: {e}")
    
    def verify_clock_alignment(self) -> bool:
        """
        Verify that all devices have aligned clocks.
        
        Returns:
            True if clocks appear aligned
        """
        print("[xMASS Array] Verifying clock alignment...")
        
        clocks = self.get_clock_info()
        
        master_clocks = [info['master_clock'] for info in clocks.values()]
        
        if len(set(master_clocks)) == 1:
            print(f"  [+] All devices at {master_clocks[0]/1e6:.2f} MHz")
            return True
        else:
            print(f"  [!] Clock mismatch detected:")
            for i, info in clocks.items():
                print(f"      Device {i}: {info['master_clock']/1e6:.2f} MHz")
            return False


def main():
    """Test xMASS SoapySDR integration."""
    print("=== xMASS SoapySDR Integration Test ===\n")
    
    if not SOAPY_AVAILABLE:
        print("[!] SoapySDR not available, cannot run test")
        return
    
    # Enumerate devices
    print("[*] Enumerating devices...")
    devices = XMASSArray.enumerate_devices()
    print(f"    Found {len(devices)} device(s)")
    
    for dev in devices:
        print(f"    - {dev.label}: {dev.hardware_key}")
    
    if len(devices) < 4:
        print("[!] Need 4 devices for xMASS array")
        return
    
    # Create array
    array = XMASSArray(num_devices=4)
    
    try:
        # Open devices
        count = array.open_all()
        if count < 4:
            print("[!] Could not open all devices")
            return
        
        # Get clock info
        print("\n[*] Clock information:")
        clocks = array.get_clock_info()
        for i, info in clocks.items():
            print(f"    Device {i}: {info['master_clock']/1e6:.2f} MHz ({info['clock_source']})")
        
        # Configure
        array.configure_all(frequency=100e6, sample_rate=1e6, gain=30)
        
        # Setup and start
        array.setup_streams()
        array.start_streaming(sync_mode=SyncMode.SOFTWARE)
        
        # Capture
        print("\n[*] Capturing synchronized samples...")
        buffers = array.capture_synchronized(num_samples=4096)
        
        print(f"    Captured {len(buffers[0])} samples from each device")
        
        # Check phase alignment
        print("\n[*] Phase alignment check:")
        for i in range(1, len(buffers)):
            correlation = np.mean(buffers[i] * np.conj(buffers[0]))
            phase_diff = np.angle(correlation)
            print(f"    Device {i} vs 0: {np.degrees(phase_diff):>6.2f}°")
        
        # Stop
        array.stop_streaming()
        
    finally:
        array.close_all()
    
    print("\n[+] Test complete!")


if __name__ == "__main__":
    main()
