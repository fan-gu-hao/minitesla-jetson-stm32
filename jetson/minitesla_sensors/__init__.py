"""智慧車感測資料介面：僅整理觀測，不產生車輛控制命令。"""
from .core import SensorStore, TelemetryParser, crc8

__all__ = ["SensorStore", "TelemetryParser", "crc8"]
