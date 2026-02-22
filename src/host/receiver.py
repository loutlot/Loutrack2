"""
UDP receiver module for collecting frame messages from Raspberry Pi cameras.

Provides functionality to:
- Receive UDP JSON messages from multiple cameras
- Buffer frames per camera with timestamp-based ordering
- Pair frames across cameras using timestamp (primary) and frame_index (secondary)
- Integrate with logger and metrics collectors
"""

import socket
import json
import threading
import time
from typing import Optional, Dict, Any, List, Callable
from dataclasses import dataclass, field
from collections import defaultdict, deque
from datetime import datetime


@dataclass
class Frame:
    """Received frame data with metadata."""
    camera_id: str
    timestamp: int  # microseconds
    frame_index: int
    blobs: List[Dict[str, float]]
    received_at: float  # system time in seconds
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "timestamp": self.timestamp,
            "frame_index": self.frame_index,
            "blobs": self.blobs
        }


@dataclass
class PairedFrames:
    """Set of frames from multiple cameras paired by timestamp."""
    timestamp: int
    frames: Dict[str, Frame]  # camera_id -> Frame
    timestamp_range_us: int = 0  # actual timestamp spread in the pair
    
    @property
    def camera_ids(self) -> List[str]:
        return list(self.frames.keys())
    
    @property
    def frame_count(self) -> int:
        return len(self.frames)


class FrameBuffer:
    """
    Per-camera frame buffer for pairing across cameras.
    
    Maintains a time-ordered buffer of frames for each camera,
    supporting timestamp-based pairing with configurable tolerance.
    """
    
    def __init__(
        self,
        buffer_size: int = 300,  # ~5 seconds at 60fps
        max_age_seconds: float = 2.0
    ):
        """
        Initialize frame buffer.
        
        Args:
            buffer_size: Maximum frames to keep per camera
            max_age_seconds: Maximum age of frames before cleanup
        """
        self.buffer_size = buffer_size
        self.max_age_seconds = max_age_seconds
        
        # Per-camera frame buffers (newest first)
        self._buffers: Dict[str, deque] = defaultdict(
            lambda: deque(maxlen=buffer_size)
        )
        self._lock = threading.Lock()
    
    def add_frame(self, frame: Frame) -> None:
        """Add a frame to the appropriate camera buffer."""
        with self._lock:
            self._buffers[frame.camera_id].appendleft(frame)
    
    def get_frames_in_window(
        self,
        camera_id: str,
        center_timestamp: int,
        window_us: int
    ) -> List[Frame]:
        """
        Get frames within a timestamp window for a camera.
        
        Args:
            camera_id: Camera to query
            center_timestamp: Center of the time window (microseconds)
            window_us: Window size in microseconds (± from center)
            
        Returns:
            List of frames within the window, sorted by timestamp distance
        """
        with self._lock:
            if camera_id not in self._buffers:
                return []
            
            frames = []
            for frame in self._buffers[camera_id]:
                if abs(frame.timestamp - center_timestamp) <= window_us:
                    frames.append(frame)
            
            # Sort by distance from center timestamp
            frames.sort(key=lambda f: abs(f.timestamp - center_timestamp))
            return frames
    
    def get_latest_frame(self, camera_id: str) -> Optional[Frame]:
        """Get the most recent frame for a camera."""
        with self._lock:
            if camera_id in self._buffers and self._buffers[camera_id]:
                return self._buffers[camera_id][0]
            return None
    
    def get_latest_timestamp(self, camera_id: str) -> Optional[int]:
        """Get the most recent timestamp for a camera."""
        frame = self.get_latest_frame(camera_id)
        return frame.timestamp if frame else None
    
    def get_camera_ids(self) -> List[str]:
        """Get list of cameras with frames in buffer."""
        with self._lock:
            return list(self._buffers.keys())
    
    def cleanup_old_frames(self) -> int:
        """
        Remove frames older than max_age_seconds.
        
        Returns:
            Number of frames removed
        """
        cutoff_time = time.time() - self.max_age_seconds
        removed = 0
        
        with self._lock:
            for camera_id in list(self._buffers.keys()):
                buffer = self._buffers[camera_id]
                original_len = len(buffer)
                
                # Remove old frames (from the end, as newest are first)
                while buffer and buffer[-1].received_at < cutoff_time:
                    buffer.pop()
                    removed += 1
        
        return removed


class FramePairer:
    """
    Pairs frames across multiple cameras using timestamp.
    
    Uses timestamp as primary pairing key with configurable tolerance,
    falling back to frame_index for cameras with timestamp issues.
    """
    
    def __init__(
        self,
        timestamp_tolerance_us: int = 5000,  # 5ms default tolerance
        min_cameras: int = 2,
        frame_index_fallback: bool = True
    ):
        """
        Initialize frame pairer.
        
        Args:
            timestamp_tolerance_us: Maximum timestamp difference for pairing (microseconds)
            min_cameras: Minimum cameras required for a valid pair
            frame_index_fallback: Whether to use frame_index as secondary pairing
        """
        self.timestamp_tolerance_us = timestamp_tolerance_us
        self.min_cameras = min_cameras
        self.frame_index_fallback = frame_index_fallback
    
    def pair_frames(
        self,
        buffer: FrameBuffer,
        reference_camera: Optional[str] = None
    ) -> List[PairedFrames]:
        """
        Find paired frames across cameras.
        
        Args:
            buffer: FrameBuffer containing frames from all cameras
            reference_camera: Optional camera to use as reference (default: most frames)
            
        Returns:
            List of PairedFrames, each containing frames from multiple cameras
        """
        camera_ids = buffer.get_camera_ids()
        if len(camera_ids) < self.min_cameras:
            return []
        
        # Choose reference camera (one with most frames, or specified)
        if reference_camera and reference_camera in camera_ids:
            ref_cam = reference_camera
        else:
            ref_cam = max(camera_ids, key=lambda c: len(buffer._buffers[c]))
        
        # Get all reference frames
        ref_frames = list(buffer._buffers.get(ref_cam, []))
        if not ref_frames:
            return []
        
        paired = []
        used_frame_indices: Dict[str, set] = defaultdict(set)
        
        for ref_frame in ref_frames:
            pair = PairedFrames(
                timestamp=ref_frame.timestamp,
                frames={ref_cam: ref_frame}
            )
            
            # Find matching frames from other cameras
            for cam_id in camera_ids:
                if cam_id == ref_cam:
                    continue
                
                # Try timestamp matching first
                candidates = buffer.get_frames_in_window(
                    cam_id,
                    ref_frame.timestamp,
                    self.timestamp_tolerance_us
                )
                
                # Filter out already-used frames
                candidates = [
                    f for f in candidates
                    if f.frame_index not in used_frame_indices[cam_id]
                ]
                
                if not candidates and self.frame_index_fallback:
                    # Fallback to frame_index matching
                    target_index = ref_frame.frame_index
                    all_frames = buffer._buffers.get(cam_id, [])
                    candidates = [
                        f for f in all_frames
                        if f.frame_index == target_index
                        and f.frame_index not in used_frame_indices[cam_id]
                    ]
                
                if candidates:
                    best_match = candidates[0]
                    pair.frames[cam_id] = best_match
                    used_frame_indices[cam_id].add(best_match.frame_index)
            
            # Only return pairs with enough cameras
            if pair.frame_count >= self.min_cameras:
                # Calculate actual timestamp spread
                timestamps = [f.timestamp for f in pair.frames.values()]
                pair.timestamp_range_us = max(timestamps) - min(timestamps)
                paired.append(pair)
        
        return paired


class UDPReceiver:
    """
    UDP server for receiving frame messages from Raspberry Pi cameras.
    
    Usage:
        receiver = UDPReceiver(port=5000)
        receiver.set_frame_callback(on_frame_received)
        receiver.start()
        # ... receive frames ...
        receiver.stop()
    """
    
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 5000,
        buffer_size: int = 65536
    ):
        """
        Initialize UDP receiver.
        
        Args:
            host: Host address to bind (default: all interfaces)
            port: UDP port to listen on
            buffer_size: Socket receive buffer size
        """
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        
        self._socket: Optional[socket.socket] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        
        self._frame_callback: Optional[Callable[[Frame], None]] = None
        self._error_callback: Optional[Callable[[Exception], None]] = None
        
        # Camera discovery
        self._camera_addresses: Dict[str, tuple] = {}  # camera_id -> (ip, port)
        
        # Statistics
        self._frames_received = 0
        self._bytes_received = 0
        self._errors = 0
    
    def set_frame_callback(self, callback: Callable[[Frame], None]) -> None:
        """Set callback for received frames."""
        self._frame_callback = callback
    
    def set_error_callback(self, callback: Callable[[Exception], None]) -> None:
        """Set callback for errors."""
        self._error_callback = callback
    
    def start(self) -> None:
        """Start the UDP receiver."""
        if self._running:
            return
        
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
        self._socket.bind((self.host, self.port))
        self._socket.settimeout(1.0)  # Allow periodic checking of _running
        
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
    
    def stop(self) -> None:
        """Stop the UDP receiver."""
        self._running = False
        if self._socket:
            self._socket.close()
        if self._thread:
            self._thread.join(timeout=2.0)
    
    def _receive_loop(self) -> None:
        """Main receive loop."""
        while self._running:
            try:
                data, addr = self._socket.recvfrom(65536)
                self._bytes_received += len(data)
                
                # Parse JSON
                try:
                    msg = json.loads(data.decode('utf-8'))
                except json.JSONDecodeError as e:
                    self._errors += 1
                    if self._error_callback:
                        self._error_callback(e)
                    continue
                
                # Validate required fields
                required = ['camera_id', 'timestamp', 'frame_index', 'blobs']
                if not all(k in msg for k in required):
                    self._errors += 1
                    continue
                
                # Create Frame object
                frame = Frame(
                    camera_id=msg['camera_id'],
                    timestamp=int(msg['timestamp']),
                    frame_index=int(msg['frame_index']),
                    blobs=msg['blobs'],
                    received_at=time.time()
                )
                
                # Record camera address
                self._camera_addresses[frame.camera_id] = addr
                
                # Increment counter
                self._frames_received += 1
                
                # Call callback
                if self._frame_callback:
                    self._frame_callback(frame)
                    
            except socket.timeout:
                continue
            except Exception as e:
                self._errors += 1
                if self._error_callback:
                    self._error_callback(e)
    
    def get_camera_address(self, camera_id: str) -> Optional[tuple]:
        """Get the (ip, port) for a camera."""
        return self._camera_addresses.get(camera_id)
    
    def get_camera_addresses(self) -> Dict[str, tuple]:
        """Get all discovered camera addresses."""
        return self._camera_addresses.copy()
    
    @property
    def is_running(self) -> bool:
        return self._running
    
    @property
    def stats(self) -> Dict[str, int]:
        """Get receiver statistics."""
        return {
            "frames_received": self._frames_received,
            "bytes_received": self._bytes_received,
            "errors": self._errors,
            "cameras_discovered": len(self._camera_addresses)
        }


class FrameProcessor:
    """
    Combined receiver, buffer, and pairer for processing camera frames.
    
    Integrates UDPReceiver, FrameBuffer, and FramePairer into a single
    processing pipeline.
    """
    
    def __init__(
        self,
        udp_port: int = 5000,
        buffer_size: int = 300,
        timestamp_tolerance_us: int = 5000,
        min_cameras_for_pair: int = 2
    ):
        """
        Initialize frame processor.
        
        Args:
            udp_port: UDP port to listen on
            buffer_size: Frame buffer size per camera
            timestamp_tolerance_us: Timestamp tolerance for pairing
            min_cameras_for_pair: Minimum cameras for valid pair
        """
        self.buffer = FrameBuffer(buffer_size=buffer_size)
        self.pairer = FramePairer(
            timestamp_tolerance_us=timestamp_tolerance_us,
            min_cameras=min_cameras_for_pair
        )
        self.receiver = UDPReceiver(port=udp_port)
        
        self._paired_callback: Optional[Callable[[PairedFrames], None]] = None
        self._last_pair_time = 0.0
        self._pair_interval = 0.016  # ~60fps
    
    def set_paired_callback(self, callback: Callable[[PairedFrames], None]) -> None:
        """Set callback for paired frames."""
        self._paired_callback = callback
    
    def start(self) -> None:
        """Start the frame processor."""
        self.receiver.set_frame_callback(self._on_frame_received)
        self.receiver.start()
    
    def stop(self) -> None:
        """Stop the frame processor."""
        self.receiver.stop()
    
    def _on_frame_received(self, frame: Frame) -> None:
        """Handle received frame."""
        self.buffer.add_frame(frame)
        
        # Periodically try to pair frames
        now = time.time()
        if now - self._last_pair_time >= self._pair_interval:
            self._last_pair_time = now
            self._process_pairs()
    
    def _process_pairs(self) -> None:
        """Process and emit paired frames."""
        pairs = self.pairer.pair_frames(self.buffer)
        
        for pair in pairs:
            if self._paired_callback:
                self._paired_callback(pair)
    
    def get_stats(self) -> Dict[str, Any]:
        """Get processor statistics."""
        return {
            "receiver": self.receiver.stats,
            "cameras": self.buffer.get_camera_ids(),
            "buffer_sizes": {
                cam: len(self.buffer._buffers[cam])
                for cam in self.buffer.get_camera_ids()
            }
        }
