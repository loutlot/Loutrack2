"""
Visualization module for tracking data.

Provides simple ASCII and file-based visualization for debugging.
For real visualization, consider using matplotlib or a 3D viewer.
"""

import json
import time
from datetime import datetime
from typing import Optional, Dict, Any, List
from pathlib import Path


class TrackingVisualizer:
    """
    Simple visualization for tracking data.
    
    Provides:
    - Console output of poses
    - JSON file export for external visualization
    - ASCII art pose indicator
    """
    
    def __init__(
        self,
        output_dir: str = "./output",
        enable_console: bool = True,
        enable_file: bool = True
    ):
        """
        Initialize visualizer.
        
        Args:
            output_dir: Directory for output files
            enable_console: Print poses to console
            enable_file: Write poses to file
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.enable_console = enable_console
        self.enable_file = enable_file
        
        self._output_file = None
        self._frame_count = 0
    
    def start_session(self, session_name: Optional[str] = None) -> None:
        """Start a new visualization session."""
        if session_name is None:
            session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if self.enable_file:
            self._output_file = open(
                self.output_dir / f"{session_name}_poses.jsonl",
                'w'
            )
            self._output_file.write(json.dumps({
                "_type": "header",
                "session": session_name,
                "started_at": datetime.now().isoformat()
            }) + '\n')
        
        self._frame_count = 0
    
    def end_session(self) -> None:
        """End current session."""
        if self._output_file:
            self._output_file.write(json.dumps({
                "_type": "footer",
                "ended_at": datetime.now().isoformat(),
                "total_frames": self._frame_count
            }) + '\n')
            self._output_file.close()
            self._output_file = None
    
    def visualize_poses(
        self,
        poses: Dict[str, Any],
        timestamp: int,
        metrics: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Visualize pose data.
        
        Args:
            poses: Dict of body_name -> pose dict
            timestamp: Frame timestamp
            metrics: Optional metrics data
        """
        self._frame_count += 1
        
        # Console output
        if self.enable_console:
            self._print_poses(poses, timestamp, metrics)
        
        # File output
        if self.enable_file and self._output_file:
            entry = {
                "_type": "frame",
                "timestamp": timestamp,
                "poses": poses,
                "metrics": metrics
            }
            self._output_file.write(json.dumps(entry) + '\n')
    
    def _print_poses(
        self,
        poses: Dict[str, Any],
        timestamp: int,
        metrics: Optional[Dict[str, Any]]
    ) -> None:
        """Print poses to console."""
        # Clear line and print
        ts_str = datetime.fromtimestamp(timestamp / 1_000_000).strftime("%H:%M:%S.%f")
        
        print(f"\n[{ts_str}] Frame {self._frame_count}")
        print("-" * 50)
        
        for name, pose in poses.items():
            if pose.get("valid", False):
                pos = pose["position"]
                quat = pose["quaternion"]
                print(f"  {name}:")
                print(f"    pos: ({pos[0]:7.4f}, {pos[1]:7.4f}, {pos[2]:7.4f}) m")
                print(f"    quat: ({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f})")
                print(f"    error: {pose['rms_error']:.6f} m, markers: {pose['observed_markers']}")
            else:
                print(f"  {name}: [LOST]")
        
        if metrics:
            fps = metrics.get("global", {}).get("fps", 0)
            print(f"  FPS: {fps:.1f}")
    
    def print_ascii_pose(
        self,
        position: List[float],
        rotation_matrix: Optional[List[List[float]]] = None
    ) -> str:
        """
        Create ASCII art representation of pose.
        
        Args:
            position: 3D position
            rotation_matrix: Optional 3x3 rotation matrix
            
        Returns:
            ASCII art string
        """
        # Simple position indicator
        x, y, z = position
        
        # Map to 2D grid (top-down view)
        grid_size = 20
        grid = [[' ' for _ in range(grid_size)] for _ in range(grid_size)]
        
        # Center of grid
        cx, cy = grid_size // 2, grid_size // 2
        
        cells_per_meter = 50.0
        
        # Map position to grid
        gx = int(cx + x * cells_per_meter)
        gy = int(cy + y * cells_per_meter)
        
        # Clamp to grid
        gx = max(0, min(grid_size - 1, gx))
        gy = max(0, min(grid_size - 1, gy))
        
        # Draw position
        grid[gy][gx] = '●'
        
        # Draw axes
        grid[cy][cx] = '+'
        
        # Convert to string
        lines = [''.join(row) for row in grid]
        return '\n'.join(lines)


class SimpleGraph:
    """
    Simple time-series graph for console output.
    """
    
    def __init__(self, width: int = 60, height: int = 10):
        """
        Initialize graph.
        
        Args:
            width: Graph width in characters
            height: Graph height in lines
        """
        self.width = width
        self.height = height
        self.values: List[float] = []
        self.max_values = width
    
    def add_value(self, value: float) -> None:
        """Add a value to the graph."""
        self.values.append(value)
        if len(self.values) > self.max_values:
            self.values = self.values[-self.max_values:]
    
    def render(self, title: str = "") -> str:
        """
        Render graph as ASCII art.
        
        Args:
            title: Optional title
            
        Returns:
            ASCII art string
        """
        if not self.values:
            return "(no data)"
        
        min_val = min(self.values)
        max_val = max(self.values)
        range_val = max_val - min_val if max_val != min_val else 1
        
        # Create grid
        grid = [[' ' for _ in range(self.width)] for _ in range(self.height)]
        
        # Plot values
        for i, val in enumerate(self.values):
            x = int(i * (self.width - 1) / max(len(self.values) - 1, 1))
            y = int((self.height - 1) * (1 - (val - min_val) / range_val))
            grid[y][x] = '●'
        
        # Draw border
        lines = []
        if title:
            lines.append(f" {title} ")
        lines.append(f"+{'-' * self.width}+")
        
        for row in grid:
            lines.append(f"|{''.join(row)}|")
        
        lines.append(f"+{'-' * self.width}+")
        lines.append(f" {min_val:.2f}" + " " * (self.width - 20) + f"{max_val:.2f} ")
        
        return '\n'.join(lines)


def format_pose_for_log(pose: Dict[str, Any]) -> str:
    """Format a pose for one-line log output."""
    if not pose.get("valid", False):
        return "[LOST]"
    
    pos = pose["position"]
    return f"({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) err={pose['rms_error']:.2f}"


def create_pose_summary(poses: Dict[str, Dict[str, Any]]) -> str:
    """Create a one-line summary of all poses."""
    parts = []
    for name, pose in poses.items():
        if pose.get("valid", False):
            parts.append(f"{name[0].upper()}:{pose['rms_error']:.2f}")
        else:
            parts.append(f"{name[0].upper()}:X")
    return " | ".join(parts)
