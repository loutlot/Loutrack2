from __future__ import annotations

import time
from typing import Any, Dict


DEFAULT_SCHEDULED_STREAM_START_LEAD_US = 1_000_000


def epoch_us() -> int:
    return int(time.time_ns() // 1_000)


def planned_stream_start_at_us(lead_us: int = DEFAULT_SCHEDULED_STREAM_START_LEAD_US) -> int:
    return epoch_us() + max(0, int(lead_us))


def scheduled_start_kwargs(mode: str, *, lead_us: int = DEFAULT_SCHEDULED_STREAM_START_LEAD_US) -> Dict[str, Any]:
    return {
        "mode": str(mode),
        "start_at_us": planned_stream_start_at_us(lead_us),
    }
