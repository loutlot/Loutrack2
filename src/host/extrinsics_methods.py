from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, Protocol


class ExtrinsicsMethod(Protocol):
    """Contract for extrinsics solving backends."""

    def solve(self, **kwargs: Any) -> Dict[str, Any]:
        ...


@dataclass(frozen=True)
class RegisteredExtrinsicsMethod:
    name: str
    solve: Callable[..., Dict[str, Any]]
    description: str


class ExtrinsicsMethodRegistry:
    """Simple in-process method registry for extrinsics solvers."""

    def __init__(self) -> None:
        self._methods: Dict[str, RegisteredExtrinsicsMethod] = {}

    def register(
        self,
        *,
        name: str,
        solve: Callable[..., Dict[str, Any]],
        description: str,
    ) -> None:
        key = name.strip()
        if not key:
            raise ValueError("extrinsics method name must not be empty")
        if key in self._methods:
            raise ValueError(f"extrinsics method already registered: {key}")
        self._methods[key] = RegisteredExtrinsicsMethod(
            name=key,
            solve=solve,
            description=description,
        )

    def get(self, name: str) -> RegisteredExtrinsicsMethod:
        key = name.strip()
        method = self._methods.get(key)
        if method is None:
            known = ", ".join(sorted(self._methods))
            raise ValueError(f"unknown extrinsics_method: {key} (known: {known})")
        return method

    def to_payload(self) -> list[Dict[str, str]]:
        return [
            {"name": method.name, "description": method.description}
            for method in sorted(self._methods.values(), key=lambda item: item.name)
        ]


def build_default_extrinsics_registry(
    *,
    blob_pose_solver: Callable[..., Dict[str, Any]],
) -> ExtrinsicsMethodRegistry:
    registry = ExtrinsicsMethodRegistry()
    registry.register(
        name="blob_pose_v2",
        solve=blob_pose_solver,
        description="Legacy blob/wand pose-capture solver (v2 schema)",
    )
    return registry
