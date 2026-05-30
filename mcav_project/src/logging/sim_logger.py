# src/logging/sim_logger.py
import json, pandas as pd
from pathlib import Path

class SimLogger:
    def __init__(self):
        self._current: dict      = {}
        self._records: list[dict] = []

    def log(self, data: dict):
        """Accumulate fields during a single step."""
        self._current.update(data)

    def commit(self, t: float):
        """Seal the current step record. Called once per physics step by the wrapper."""
        self._current["t"] = t
        self._records.append(dict(self._current))
        self._current = {}

    def to_dataframe(self) -> pd.DataFrame:
        return pd.json_normalize(self._records)

    def save(self, path: str):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self._records, f, indent=2)

    @staticmethod
    def load(path: str) -> pd.DataFrame:
        with open(path) as f:
            return pd.json_normalize(json.load(f))