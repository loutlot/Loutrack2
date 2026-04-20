"""
UDP receiver module for collecting frame messages from Raspberry Pi cameras.

Provides functionality to:
- Receive UDP JSON messages from multiple cameras
- Buffer frames per camera with timestamp-based ordering
- Pair frames across cameras using timestamp (primary) and optional frame_index fallback
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
    frame_index: Optional[int]
    blobs: List[Dict[str, float]]
    received_at: float  # system time in seconds
    host_received_at_us: int
    timestamp_source: Optional[str] = None
    sensor_timestamp_ns: Optional[int] = None
    capture_to_process_ms: Optional[float] = None
    capture_to_send_ms: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "camera_id": self.camera_id,
            "timestamp": self.timestamp,
            "host_received_at_us": self.host_received_at_us,
            "blobs": self.blobs
        }
        if self.frame_index is not None:
            payload["frame_index"] = self.frame_index
        if self.timestamp_source is not None:
            payload["timestamp_source"] = self.timestamp_source
        if self.sensor_timestamp_ns is not None:
            payload["sensor_timestamp_ns"] = self.sensor_timestamp_ns
        if self.capture_to_process_ms is not None:
            payload["capture_to_process_ms"] = self.capture_to_process_ms
        if self.capture_to_send_ms is not None:
            payload["capture_to_send_ms"] = self.capture_to_send_ms
        return payload


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
        
        # Per-camera frame buffers (oldest first).
        self._buffers: Dict[str, deque] = defaultdict(
            lambda: deque(maxlen=buffer_size)
        )
        self._lock = threading.Lock()
    
    def add_frame(self, frame: Frame) -> None:
        """Add a frame to the appropriate camera buffer."""
        with self._lock:
            buffer = self._buffers[frame.camera_id]
            if not buffer or frame.timestamp >= buffer[-1].timestamp:
                buffer.append(frame)
                return

            frames = list(buffer)
            insert_at = len(frames)
            for index, current in enumerate(frames):
                if frame.timestamp < current.timestamp:
                    insert_at = index
                    break
            frames.insert(insert_at, frame)
            if len(frames) > self.buffer_size:
                frames = frames[-self.buffer_size :]
            self._buffers[frame.camera_id] = deque(frames, maxlen=self.buffer_size)
    
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
                return self._buffers[camera_id][-1]
            return None

    def get_oldest_frame(self, camera_id: str) -> Optional[Frame]:
        """Get the oldest buffered frame for a camera."""
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

    def get_all_frames(self, camera_id: str) -> List["Frame"]:
        """Return a snapshot of all buffered frames for a camera (newest first)."""
        with self._lock:
            if camera_id not in self._buffers:
                return []
            return list(reversed(self._buffers[camera_id]))

    def get_buffer_size(self, camera_id: str) -> int:
        """Return current number of buffered frames for a camera."""
        with self._lock:
            return len(self._buffers.get(camera_id, []))

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
                while buffer and buffer[0].received_at < cutoff_time:
                    buffer.popleft()
                    removed += 1
        
        return removed

    def remove_frame(self, camera_id: str, frame: Frame) -> bool:
        """Remove a specific frame from a camera buffer."""
        with self._lock:
            buffer = self._buffers.get(camera_id)
            if not buffer:
                return False
            try:
                buffer.remove(frame)
                return True
            except ValueError:
                return False

    def pop_oldest_frame(self, camera_id: str) -> Optional[Frame]:
        """Pop and return the oldest buffered frame for a camera."""
        with self._lock:
            buffer = self._buffers.get(camera_id)
            if not buffer:
                return None
            return buffer.popleft()

    def find_best_match(
        self,
        camera_id: str,
        center_timestamp: int,
        window_us: int,
        *,
        frame_index: Optional[int] = None,
        frame_index_fallback: bool = False,
    ) -> tuple[Optional[Frame], bool]:
        """
        Find the best timestamp match for a frame, with optional frame-index fallback.

        Returns:
            (frame, matched_by_frame_index)
        """
        with self._lock:
            buffer = self._buffers.get(camera_id)
            if not buffer:
                return None, False

            best_match: Optional[Frame] = None
            best_distance = window_us + 1
            fallback_match: Optional[Frame] = None
            lower = center_timestamp - window_us
            upper = center_timestamp + window_us

            for candidate in buffer:
                if candidate.timestamp < lower:
                    continue
                if candidate.timestamp > upper:
                    break
                distance = abs(candidate.timestamp - center_timestamp)
                if distance <= window_us and distance < best_distance:
                    best_match = candidate
                    best_distance = distance
                if (
                    frame_index_fallback
                    and frame_index is not None
                    and candidate.frame_index is not None
                    and candidate.frame_index == frame_index
                    and fallback_match is None
                ):
                    fallback_match = candidate

            if best_match is not None:
                return best_match, False
            if fallback_match is not None:
                return fallback_match, True
            return None, False


class FramePairer:
    """
    Pairs frames across multiple cameras using timestamp.
    
    Uses timestamp as primary pairing key with configurable tolerance,
    optionally falling back to frame_index for legacy capture flows.
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
        self._timestamp_unmatched_frames = 0
        self._frame_index_fallback_pairs = 0
        self._stale_frames_dropped = 0
        self._pair_attempts = 0
        self._pairs_emitted = 0
    
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
            ref_cam = max(camera_ids, key=lambda c: buffer.get_buffer_size(c))
        
        paired: List[PairedFrames] = []

        while True:
            ref_frame = buffer.get_oldest_frame(ref_cam)
            if ref_frame is None:
                break

            self._pair_attempts += 1
            pair = PairedFrames(
                timestamp=ref_frame.timestamp,
                frames={ref_cam: ref_frame}
            )
            
            # Find matching frames from other cameras
            for cam_id in camera_ids:
                if cam_id == ref_cam:
                    continue
                best_match, matched_by_frame_index = buffer.find_best_match(
                    cam_id,
                    ref_frame.timestamp,
                    self.timestamp_tolerance_us,
                    frame_index=ref_frame.frame_index,
                    frame_index_fallback=self.frame_index_fallback,
                )

                if best_match is None:
                    self._timestamp_unmatched_frames += 1
                    continue

                pair.frames[cam_id] = best_match
                if matched_by_frame_index:
                    self._frame_index_fallback_pairs += 1
            
            # Only return pairs with enough cameras
            if pair.frame_count >= self.min_cameras:
                # Calculate actual timestamp spread
                timestamps = [f.timestamp for f in pair.frames.values()]
                pair.timestamp_range_us = max(timestamps) - min(timestamps)
                paired.append(pair)
                self._pairs_emitted += 1
                buffer.remove_frame(ref_cam, ref_frame)
                for camera_id, frame in pair.frames.items():
                    if camera_id == ref_cam:
                        continue
                    buffer.remove_frame(camera_id, frame)
                continue

            if self._should_drop_stale_reference(buffer, ref_cam, ref_frame, pair.frame_count, camera_ids):
                buffer.pop_oldest_frame(ref_cam)
                self._stale_frames_dropped += 1
                continue
            break

        return paired

    def _should_drop_stale_reference(
        self,
        buffer: FrameBuffer,
        ref_cam: str,
        ref_frame: Frame,
        matched_frame_count: int,
        camera_ids: List[str],
    ) -> bool:
        """
        Drop a reference frame once it can no longer reach the minimum camera count.
        """
        cutoff = ref_frame.timestamp + self.timestamp_tolerance_us
        unresolved_cameras = 0
        for cam_id in camera_ids:
            if cam_id == ref_cam:
                continue
            latest_timestamp = buffer.get_latest_timestamp(cam_id)
            if latest_timestamp is None:
                unresolved_cameras += 1
                continue
            if latest_timestamp <= cutoff:
                unresolved_cameras += 1
        return matched_frame_count + unresolved_cameras < self.min_cameras

    def get_stats(self) -> Dict[str, int]:
        return {
            "timestamp_unmatched_frames": self._timestamp_unmatched_frames,
            "frame_index_fallback_pairs": self._frame_index_fallback_pairs,
            "stale_frames_dropped": self._stale_frames_dropped,
            "pair_attempts": self._pair_attempts,
            "pairs_emitted": self._pairs_emitted,
        }


# Try to use orjson for 5-10x faster JSON parsing; fall back to stdlib json.
try:
    import orjson as _json_lib

    def _parse_json(data: bytes) -> Any:
        return _json_lib.loads(data)

except ImportError:
    def _parse_json(data: bytes) -> Any:  # type: ignore[misc]
        return json.loads(data.decode("utf-8"))


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

    # Maximum single UDP datagram we'll accept (64 KiB covers any JSON blob payload)
    _RECV_BYTES: int = 65536
    # Kernel socket receive buffer (2 MiB).
    # With 3 cameras at 60 fps and ~500-byte frames the steady-state throughput is
    # ~90 KB/s; the enlarged kernel buffer absorbs bursts without dropping packets.
    _KERNEL_RECV_BUF: int = 2 * 1024 * 1024

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 5000,
        buffer_size: int = 65536,
    ):
        """
        Initialize UDP receiver.

        Args:
            host:        Host address to bind (default: all interfaces).
            port:        UDP port to listen on.
            buffer_size: Max datagram size to read (default 65536).
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
        # Set a large kernel receive buffer to absorb bursts from multiple cameras.
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self._KERNEL_RECV_BUF)
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
                data, addr = self._socket.recvfrom(self._RECV_BYTES)
                self._bytes_received += len(data)

                # Parse JSON (uses orjson if available for ~5-10x speedup)
                try:
                    msg = _parse_json(data)
                except (ValueError, KeyError) as e:
                    self._errors += 1
                    if self._error_callback:
                        self._error_callback(e)
                    continue
                
                # Validate required fields
                required = ['camera_id', 'timestamp', 'blobs']
                if not all(k in msg for k in required):
                    self._errors += 1
                    continue

                host_received_at = time.time()
                host_received_at_us = int(host_received_at * 1_000_000)
                frame_index = msg.get("frame_index")
                
                # Create Frame object
                frame = Frame(
                    camera_id=msg['camera_id'],
                    timestamp=int(msg['timestamp']),
                    frame_index=int(frame_index) if frame_index is not None else None,
                    blobs=msg['blobs'],
                    received_at=host_received_at,
                    host_received_at_us=host_received_at_us,
                    timestamp_source=str(msg["timestamp_source"]) if msg.get("timestamp_source") is not None else None,
                    sensor_timestamp_ns=int(msg["sensor_timestamp_ns"]) if msg.get("sensor_timestamp_ns") is not None else None,
                    capture_to_process_ms=float(msg["capture_to_process_ms"]) if msg.get("capture_to_process_ms") is not None else None,
                    capture_to_send_ms=float(msg["capture_to_send_ms"]) if msg.get("capture_to_send_ms") is not None else None,
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
        min_cameras_for_pair: int = 2,
        pair_interval_s: float = 0.0,
        frame_index_fallback: bool = True,
    ):
        """
        Initialize frame processor.
        
        Args:
            udp_port: UDP port to listen on
            buffer_size: Frame buffer size per camera
            timestamp_tolerance_us: Timestamp tolerance for pairing
            min_cameras_for_pair: Minimum cameras for valid pair
            pair_interval_s: Minimum seconds between pairing passes. Defaults
                to 0 so live tracking emits newly completed pairs immediately.
            frame_index_fallback: Whether to use legacy frame_index fallback.
        """
        self.buffer = FrameBuffer(buffer_size=buffer_size)
        self.pairer = FramePairer(
            timestamp_tolerance_us=timestamp_tolerance_us,
            min_cameras=min_cameras_for_pair,
            frame_index_fallback=frame_index_fallback,
        )
        self.receiver = UDPReceiver(port=udp_port)
        
        self._paired_callback: Optional[Callable[[PairedFrames], None]] = None
        self._last_pair_time = 0.0
        self._pair_interval = max(0.0, float(pair_interval_s))
        self._pair_pass_durations_ms: deque[float] = deque(maxlen=120)
    
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
        if self._pair_interval == 0.0 or now - self._last_pair_time >= self._pair_interval:
            self._last_pair_time = now
            self._process_pairs()
    
    def _process_pairs(self) -> None:
        """Process and emit paired frames."""
        self.buffer.cleanup_old_frames()
        started = time.perf_counter()
        pairs = self.pairer.pair_frames(self.buffer)
        self._pair_pass_durations_ms.append((time.perf_counter() - started) * 1000.0)
        
        for pair in pairs:
            if self._paired_callback:
                self._paired_callback(pair)
    
    def get_stats(self) -> Dict[str, Any]:
        """Get processor statistics."""
        cam_ids = self.buffer.get_camera_ids()
        pair_pass_values = list(self._pair_pass_durations_ms)
        pair_pass_mean = sum(pair_pass_values) / len(pair_pass_values) if pair_pass_values else 0.0
        pair_pass_max = max(pair_pass_values) if pair_pass_values else 0.0
        return {
            "receiver": self.receiver.stats,
            "cameras": cam_ids,
            "buffer_sizes": {
                cam: self.buffer.get_buffer_size(cam) for cam in cam_ids
            },
            "pairer": self.pairer.get_stats(),
            "pair_pass_ms": {
                "mean": round(pair_pass_mean, 3),
                "last": round(pair_pass_values[-1], 3) if pair_pass_values else 0.0,
                "max": round(pair_pass_max, 3),
            },
        }
