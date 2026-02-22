"""
Replay module for reproducing logged frame messages.

Provides functionality to:
- Read recorded log files (JSONL format)
- Replay frames with original timing or at accelerated speed
- Emit frames to callback handlers for processing
- Support seeking and filtering by camera_id
"""

import json
import time
from datetime import datetime
from typing import Optional, Dict, Any, List, Callable, Generator, Iterator
from pathlib import Path
import threading
from dataclasses import dataclass


@dataclass
class LogHeader:
    """Metadata from log file header."""
    schema_version: str
    capture_start: str
    log_format: str


@dataclass
class LogFooter:
    """Metadata from log file footer."""
    capture_end: str
    total_frames: int


@dataclass
class FrameEntry:
    """A single frame entry from the log."""
    received_at: str
    data: Dict[str, Any]
    
    @property
    def camera_id(self) -> str:
        return self.data.get("camera_id", "")
    
    @property
    def timestamp(self) -> int:
        return self.data.get("timestamp", 0)
    
    @property
    def frame_index(self) -> int:
        return self.data.get("frame_index", 0)
    
    @property
    def blobs(self) -> List[Dict[str, float]]:
        return self.data.get("blobs", [])


@dataclass
class EventEntry:
    """A custom event from the log."""
    event_type: str
    timestamp: str
    data: Dict[str, Any]


class FrameReplay:
    """
    Replay recorded frame logs with timing control.
    
    Usage:
        replay = FrameReplay("logs/20260222_120000.jsonl")
        
        # Iterate with original timing
        for entry in replay.replay(realtime=True):
            process_frame(entry)
        
        # Or iterate at max speed
        for entry in replay.replay(realtime=False):
            process_frame(entry)
            
        # Use callback for async processing
        replay.start_realtime_replay(callback=process_frame)
        replay.stop()
    """
    
    def __init__(self, log_file: str):
        """
        Initialize the replay reader.
        
        Args:
            log_file: Path to the JSONL log file
        """
        self.log_file = Path(log_file)
        if not self.log_file.exists():
            raise FileNotFoundError(f"Log file not found: {log_file}")
        
        self._header: Optional[LogHeader] = None
        self._footer: Optional[LogFooter] = None
        self._frames: List[FrameEntry] = []
        self._events: List[EventEntry] = []
        self._loaded = False
        self._stop_flag = threading.Event()
        self._replay_thread: Optional[threading.Thread] = None
        
        self._load_log()
    
    def _load_log(self) -> None:
        """Load and parse the log file."""
        with open(self.log_file, 'r', encoding='utf-8') as f:
            for line in f:
                entry = json.loads(line.strip())
                entry_type = entry.get("_type")
                
                if entry_type == "header":
                    self._header = LogHeader(
                        schema_version=entry.get("schema_version", "unknown"),
                        capture_start=entry.get("capture_start", ""),
                        log_format=entry.get("log_format", "jsonl")
                    )
                elif entry_type == "footer":
                    self._footer = LogFooter(
                        capture_end=entry.get("capture_end", ""),
                        total_frames=entry.get("total_frames", 0)
                    )
                elif entry_type == "frame":
                    self._frames.append(FrameEntry(
                        received_at=entry.get("received_at", ""),
                        data=entry.get("data", {})
                    ))
                elif entry_type == "event":
                    self._events.append(EventEntry(
                        event_type=entry.get("event_type", ""),
                        timestamp=entry.get("timestamp", ""),
                        data=entry.get("data", {})
                    ))
        
        self._loaded = True
    
    @property
    def header(self) -> Optional[LogHeader]:
        """Get log file header metadata."""
        return self._header
    
    @property
    def footer(self) -> Optional[LogFooter]:
        """Get log file footer metadata."""
        return self._footer
    
    @property
    def frame_count(self) -> int:
        """Total number of frames in the log."""
        return len(self._frames)
    
    @property
    def events(self) -> List[EventEntry]:
        """Get all events from the log."""
        return self._events.copy()
    
    def get_cameras(self) -> List[str]:
        """Get list of unique camera IDs in the log."""
        return list(set(f.camera_id for f in self._frames))
    
    def replay(
        self,
        realtime: bool = True,
        speed: float = 1.0,
        camera_filter: Optional[List[str]] = None,
        start_frame: int = 0,
        end_frame: Optional[int] = None
    ) -> Generator[FrameEntry, None, None]:
        """
        Replay frames from the log.
        
        Args:
            realtime: If True, replay with original timing; if False, replay at max speed
            speed: Playback speed multiplier (1.0 = realtime, 2.0 = 2x speed, etc.)
            camera_filter: Optional list of camera IDs to filter (None = all cameras)
            start_frame: Frame index to start from
            end_frame: Frame index to end at (None = until end)
            
        Yields:
            FrameEntry objects in order
        """
        if not self._loaded:
            self._load_log()
        
        frames = self._frames[start_frame:end_frame]
        
        if camera_filter:
            frames = [f for f in frames if f.camera_id in camera_filter]
        
        if not realtime:
            for frame in frames:
                yield frame
            return
        
        # Realtime replay with timing
        prev_received_at = None
        
        for frame in frames:
            if self._stop_flag.is_set():
                break
            
            if prev_received_at is not None:
                # Calculate delay based on original timestamps
                prev_dt = datetime.fromisoformat(prev_received_at)
                curr_dt = datetime.fromisoformat(frame.received_at)
                delay = (curr_dt - prev_dt).total_seconds() / speed
                
                if delay > 0:
                    time.sleep(delay)
            
            prev_received_at = frame.received_at
            yield frame
    
    def start_realtime_replay(
        self,
        callback: Callable[[FrameEntry], None],
        speed: float = 1.0,
        camera_filter: Optional[List[str]] = None,
        on_complete: Optional[Callable[[], None]] = None
    ) -> None:
        """
        Start asynchronous realtime replay with callback.
        
        Args:
            callback: Function to call for each frame
            speed: Playback speed multiplier
            camera_filter: Optional list of camera IDs to filter
            on_complete: Optional callback when replay finishes
        """
        self._stop_flag.clear()
        
        def _replay_thread():
            try:
                for frame in self.replay(
                    realtime=True,
                    speed=speed,
                    camera_filter=camera_filter
                ):
                    if self._stop_flag.is_set():
                        break
                    callback(frame)
            finally:
                if on_complete:
                    on_complete()
        
        self._replay_thread = threading.Thread(target=_replay_thread, daemon=True)
        self._replay_thread.start()
    
    def stop(self) -> None:
        """Stop an ongoing replay."""
        self._stop_flag.set()
        if self._replay_thread:
            self._replay_thread.join(timeout=2.0)
    
    def get_frame_at(self, index: int) -> Optional[FrameEntry]:
        """
        Get a specific frame by index.
        
        Args:
            index: Frame index (0-based)
            
        Returns:
            FrameEntry or None if index out of range
        """
        if 0 <= index < len(self._frames):
            return self._frames[index]
        return None
    
    def get_frames_by_camera(self, camera_id: str) -> List[FrameEntry]:
        """
        Get all frames from a specific camera.
        
        Args:
            camera_id: Camera identifier
            
        Returns:
            List of FrameEntry objects
        """
        return [f for f in self._frames if f.camera_id == camera_id]
    
    def get_timestamp_range(self) -> tuple:
        """
        Get the timestamp range of the log.
        
        Returns:
            Tuple of (min_timestamp, max_timestamp) in microseconds
        """
        if not self._frames:
            return (0, 0)
        
        timestamps = [f.timestamp for f in self._frames]
        return (min(timestamps), max(timestamps))
    
    def get_duration_seconds(self) -> float:
        """
        Get the duration of the recording in seconds.
        
        Returns:
            Duration in seconds
        """
        if not self._frames:
            return 0.0
        
        min_ts, max_ts = self.get_timestamp_range()
        return (max_ts - min_ts) / 1_000_000.0  # Convert us to seconds


def validate_log_integrity(log_file: str) -> Dict[str, Any]:
    """
    Validate a log file for integrity and consistency.
    
    Args:
        log_file: Path to the JSONL log file
        
    Returns:
        Validation result dictionary
    """
    result = {
        "valid": True,
        "errors": [],
        "warnings": [],
        "stats": {}
    }
    
    try:
        replay = FrameReplay(log_file)
        
        # Check header
        if not replay.header:
            result["errors"].append("Missing header")
            result["valid"] = False
        elif replay.header.schema_version != "1.0":
            result["warnings"].append(f"Unknown schema version: {replay.header.schema_version}")
        
        # Check footer
        if not replay.footer:
            result["warnings"].append("Missing footer (log may be incomplete)")
        
        # Check frame continuity per camera
        cameras = replay.get_cameras()
        for cam_id in cameras:
            cam_frames = replay.get_frames_by_camera(cam_id)
            indices = [f.frame_index for f in cam_frames]
            
            # Check for gaps in frame_index
            if indices:
                expected_count = max(indices) - min(indices) + 1
                actual_count = len(set(indices))
                if expected_count != actual_count:
                    result["warnings"].append(
                        f"Camera {cam_id}: {expected_count - actual_count} frame(s) missing"
                    )
        
        result["stats"] = {
            "total_frames": replay.frame_count,
            "cameras": cameras,
            "duration_seconds": replay.get_duration_seconds(),
            "events_count": len(replay.events)
        }
        
    except Exception as e:
        result["valid"] = False
        result["errors"].append(str(e))
    
    return result
