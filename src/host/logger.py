"""
Logger module for recording frame messages received from Raspberry Pi cameras.

Provides functionality to:
- Record incoming frame messages to a timestamped log file
- Support both JSONL (JSON Lines) format for efficient streaming
- Include metadata for replay (schema version, capture start time)
"""

import json
import os
from datetime import datetime
from typing import Optional, Dict, Any, List
from pathlib import Path
import threading
import queue


class FrameLogger:
    """
    Thread-safe logger for recording frame messages.
    
    Usage:
        logger = FrameLogger(log_dir="./logs")
        logger.start_recording()
        logger.log_frame({"camera_id": "pi-01", "timestamp": 1234567890000000, ...})
        logger.stop_recording()
    """
    
    SCHEMA_VERSION = "1.0"
    
    def __init__(self, log_dir: str = "./logs"):
        """
        Initialize the frame logger.
        
        Args:
            log_dir: Directory to store log files
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self._recording = False
        self._log_file: Optional[os.PathLike] = None
        self._file_handle = None
        self._lock = threading.Lock()
        self._write_queue: queue.Queue = queue.Queue()
        self._writer_thread: Optional[threading.Thread] = None
        self._start_time: Optional[str] = None
        self._frame_count = 0
    
    def start_recording(self, session_name: Optional[str] = None) -> str:
        """
        Start recording frames to a new log file.
        
        Args:
            session_name: Optional name for the session (default: timestamp)
            
        Returns:
            Path to the created log file
            
        Raises:
            RuntimeError: If recording is already in progress
        """
        with self._lock:
            if self._recording:
                raise RuntimeError("Recording already in progress")
            
            self._start_time = datetime.now().isoformat()
            if session_name is None:
                session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            self._log_file = self.log_dir / f"{session_name}.jsonl"
            self._file_handle = open(self._log_file, 'w', encoding='utf-8')
            self._recording = True
            self._frame_count = 0
            
            # Write header/metadata as first line
            header = {
                "_type": "header",
                "schema_version": self.SCHEMA_VERSION,
                "capture_start": self._start_time,
                "log_format": "jsonl"
            }
            self._file_handle.write(json.dumps(header) + '\n')
            
            # Start writer thread for async writes
            self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
            self._writer_thread.start()
            
            return str(self._log_file)
    
    def stop_recording(self) -> Dict[str, Any]:
        """
        Stop recording and close the log file.
        
        Returns:
            Metadata about the recording session
        """
        with self._lock:
            if not self._recording:
                return {"status": "not_recording"}
            
            self._recording = False
            
            # Signal writer thread to stop
            self._write_queue.put(None)
            if self._writer_thread:
                self._writer_thread.join(timeout=5.0)
            
            # Write footer
            footer = {
                "_type": "footer",
                "capture_end": datetime.now().isoformat(),
                "total_frames": self._frame_count
            }
            self._file_handle.write(json.dumps(footer) + '\n')
            self._file_handle.close()
            
            metadata = {
                "log_file": str(self._log_file),
                "start_time": self._start_time,
                "end_time": datetime.now().isoformat(),
                "total_frames": self._frame_count
            }
            
            self._log_file = None
            self._file_handle = None
            self._frame_count = 0
            
            return metadata
    
    def log_frame(self, frame_data: Dict[str, Any]) -> None:
        """
        Log a frame message to the current recording.
        
        Args:
            frame_data: Frame message dictionary (must conform to schema/messages.json)
            
        Raises:
            RuntimeError: If not currently recording
        """
        if not self._recording:
            raise RuntimeError("Not currently recording")
        
        # Add received timestamp for latency measurement
        frame_entry = {
            "_type": "frame",
            "received_at": datetime.now().isoformat(),
            "data": frame_data
        }
        
        self._write_queue.put(frame_entry)
        self._frame_count += 1
    
    def log_event(self, event_type: str, event_data: Dict[str, Any]) -> None:
        """
        Log a custom event (e.g., calibration start, sync status change).
        
        Args:
            event_type: Type identifier for the event
            event_data: Event-specific data
        """
        if not self._recording:
            raise RuntimeError("Not currently recording")
        
        event_entry = {
            "_type": "event",
            "event_type": event_type,
            "timestamp": datetime.now().isoformat(),
            "data": event_data
        }
        
        self._write_queue.put(event_entry)
    
    def _writer_loop(self) -> None:
        """Background thread for writing log entries."""
        while True:
            entry = self._write_queue.get()
            if entry is None:
                break
            
            if self._file_handle:
                self._file_handle.write(json.dumps(entry) + '\n')
                self._file_handle.flush()
    
    @property
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._recording
    
    @property
    def current_log_file(self) -> Optional[str]:
        """Get the current log file path if recording."""
        return str(self._log_file) if self._log_file else None


def list_log_files(log_dir: str = "./logs") -> List[Dict[str, Any]]:
    """
    List available log files with metadata.
    
    Args:
        log_dir: Directory containing log files
        
    Returns:
        List of log file info dictionaries
    """
    log_path = Path(log_dir)
    if not log_path.exists():
        return []
    
    logs = []
    for f in sorted(log_path.glob("*.jsonl"), reverse=True):
        # Read header to get metadata
        try:
            with open(f, 'r', encoding='utf-8') as fp:
                first_line = fp.readline()
                header = json.loads(first_line)
                if header.get("_type") == "header":
                    logs.append({
                        "path": str(f),
                        "name": f.stem,
                        "size_bytes": f.stat().st_size,
                        "modified": datetime.fromtimestamp(f.stat().st_mtime).isoformat(),
                        "schema_version": header.get("schema_version"),
                        "capture_start": header.get("capture_start")
                    })
        except (json.JSONDecodeError, KeyError):
            logs.append({
                "path": str(f),
                "name": f.stem,
                "size_bytes": f.stat().st_size,
                "modified": datetime.fromtimestamp(f.stat().st_mtime).isoformat(),
                "schema_version": "unknown",
                "capture_start": None
            })
    
    return logs
