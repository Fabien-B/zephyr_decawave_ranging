"""Python client for the PDEC binary UART protocol.

The main class is usable in two styles:

* From an application, call methods such as ``range()``, ``get_distance()``,
  ``list_devices()``, and ``send_data()``.
* From a Python REPL, call ``command("monitor")`` to toggle a background monitor
  that prints unsolicited ranging events, similarly to the device shell.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import queue
import struct
import threading
import time
from typing import Callable, Optional, Protocol


class SerialLike(Protocol):
    def read(self, size: int = 1) -> bytes:
        ...

    def write(self, data: bytes) -> int:
        ...

    def close(self) -> None:
        ...


class Command(IntEnum):
    RANGE = 0x01
    GET_DISTANCE = 0x02
    LIST_DEVICES = 0x03
    SEND_DATA = 0x04
    SET_DISCOVERY = 0x05


class Message(IntEnum):
    ERROR = 0x7F
    RANGE_RESULT = 0x81
    DISTANCE = 0x82
    DEVICE_LIST = 0x83
    SEND_DATA_RESULT = 0x84
    DISCOVERY_STATE = 0x85
    RANGING_EVENT = 0x90
    DATA_EVENT = 0x91
    TDOA_REPORT_EVENT = 0x92


class Status(IntEnum):
    OK = 0x00
    ERROR = 0x01
    INVALID_LENGTH = 0x02
    NOT_FOUND = 0x03
    RANGE_FAILED = 0x04
    MESSAGE_TOO_LONG = 0x05


class EventKind(IntEnum):
    SINGLE_SIDED = 0x00
    DOUBLE_SIDED = 0x01
    INDIRECT = 0x02


STATUS_NAMES = {
    Status.OK: "ok",
    Status.ERROR: "error",
    Status.INVALID_LENGTH: "invalid length",
    Status.NOT_FOUND: "not found",
    Status.RANGE_FAILED: "range failed",
    Status.MESSAGE_TOO_LONG: "message too long",
}

EVENT_KIND_NAMES = {
    EventKind.SINGLE_SIDED: "SS",
    EventKind.DOUBLE_SIDED: "DS",
    EventKind.INDIRECT: "IN",
}


@dataclass(frozen=True)
class DistanceResult:
    dst_id: int
    status: Status
    distance_m: float


@dataclass(frozen=True)
class SendDataResult:
    dst_id: int
    status: Status
    sent_len: int


@dataclass(frozen=True)
class RangingEvent:
    kind: EventKind
    src_id: int
    dst_id: int
    distance_m: float


@dataclass(frozen=True)
class DataEvent:
    src_id: int
    data: bytes


@dataclass(frozen=True)
class TdoaReportEntry:
    reporter_id: int
    distance_m: float


@dataclass(frozen=True)
class TdoaReportEvent:
    blink_id: int
    expected_count: int
    timed_out: bool
    reports: tuple[TdoaReportEntry, ...]


PdecEvent = RangingEvent | DataEvent | TdoaReportEvent


class PdecUartError(RuntimeError):
    pass


class PdecProtocolError(PdecUartError):
    def __init__(self, command: int, status: int):
        self.command = command
        self.status = Status(status) if status in Status._value2member_map_ else status
        label = STATUS_NAMES.get(self.status, f"status {status}")
        super().__init__(f"command 0x{command:02x} failed: {label}")


class PdecTimeoutError(PdecUartError, TimeoutError):
    pass


class PdecUart:
    SYNC = b"\xff\xff"
    MAX_PAYLOAD_LEN = 255
    APP_DATA_LEN_MAX = 113

    def __init__(
        self,
        serial_or_port: SerialLike | str,
        baudrate: int = 115200,
        timeout: float = 1.0,
        start_reader: bool = True,
    ):
        self.serial = self._open_serial(serial_or_port, baudrate, timeout)
        self.timeout = timeout
        self._responses: dict[Message, queue.Queue[bytes]] = {
            msg: queue.Queue()
            for msg in Message
            if msg not in {Message.RANGING_EVENT, Message.DATA_EVENT, Message.TDOA_REPORT_EVENT}
        }
        self._event_handlers: list[Callable[[PdecEvent], None]] = []
        self._write_lock = threading.Lock()
        self._stop_reader = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None
        self._monitor_stop = threading.Event()
        self._monitor_queue: queue.Queue[RangingEvent | TdoaReportEvent] = queue.Queue()

        if start_reader:
            self.start_reader()

    @staticmethod
    def _open_serial(serial_or_port: SerialLike | str, baudrate: int, timeout: float) -> SerialLike:
        if not isinstance(serial_or_port, str):
            return serial_or_port

        try:
            import serial
        except ImportError as exc:
            raise PdecUartError("pyserial is required when opening a serial port path") from exc

        return serial.Serial(serial_or_port, baudrate=baudrate, timeout=timeout)

    def __enter__(self) -> "PdecUart":
        self.start_reader()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        self.stop_monitor()
        self._stop_reader.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=2.0)
            self._reader_thread = None
        self.serial.close()

    def start_reader(self) -> None:
        if self._reader_thread is not None and self._reader_thread.is_alive():
            return

        self._stop_reader.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="pdec-uart-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def add_event_handler(self, handler: Callable[[PdecEvent], None]) -> None:
        self._event_handlers.append(handler)

    def remove_event_handler(self, handler: Callable[[PdecEvent], None]) -> None:
        self._event_handlers.remove(handler)

    def monitor(self, enabled: bool = True, printer: Callable[[str], None] = print) -> None:
        if enabled:
            self.start_monitor(printer=printer)
        else:
            self.stop_monitor()

    def start_monitor(self, printer: Callable[[str], None] = print) -> None:
        if self._monitor_thread is not None and self._monitor_thread.is_alive():
            return

        self._monitor_stop.clear()
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(printer,),
            name="pdec-uart-monitor",
            daemon=True,
        )
        self._monitor_thread.start()

    def stop_monitor(self) -> None:
        self._monitor_stop.set()
        if self._monitor_thread is not None:
            self._monitor_thread.join(timeout=2.0)
            self._monitor_thread = None

    def range(self, dst_id: int, timeout: Optional[float] = None) -> DistanceResult:
        self._send_frame(struct.pack("<BH", Command.RANGE, dst_id))
        payload = self._wait_for(Message.RANGE_RESULT, timeout)
        result = self._parse_distance_result(payload, Message.RANGE_RESULT)
        if result.status != Status.OK:
            raise PdecProtocolError(Command.RANGE, result.status)
        return result

    def get_distance(self, dst_id: int, timeout: Optional[float] = None) -> DistanceResult:
        self._send_frame(struct.pack("<BH", Command.GET_DISTANCE, dst_id))
        payload = self._wait_for(Message.DISTANCE, timeout)
        result = self._parse_distance_result(payload, Message.DISTANCE)
        if result.status != Status.OK:
            raise PdecProtocolError(Command.GET_DISTANCE, result.status)
        return result

    def list_devices(self, timeout: Optional[float] = None) -> list[int]:
        self._send_frame(struct.pack("<B", Command.LIST_DEVICES))
        payload = self._wait_for(Message.DEVICE_LIST, timeout)
        if len(payload) < 2 or payload[0] != Message.DEVICE_LIST:
            raise PdecUartError("invalid device list response")

        count = payload[1]
        expected_len = 2 + (count * 2)
        if len(payload) != expected_len:
            raise PdecUartError("truncated device list response")

        return list(struct.unpack(f"<{count}H", payload[2:])) if count else []

    def send_data(self, dst_id: int, data: bytes | str, timeout: Optional[float] = None) -> SendDataResult:
        if isinstance(data, str):
            data = data.encode()
        if len(data) > self.APP_DATA_LEN_MAX:
            raise ValueError(f"data payload is limited to {self.APP_DATA_LEN_MAX} bytes")

        self._send_frame(struct.pack("<BH", Command.SEND_DATA, dst_id) + data)
        payload = self._wait_for(Message.SEND_DATA_RESULT, timeout)
        if len(payload) != 5 or payload[0] != Message.SEND_DATA_RESULT:
            raise PdecUartError("invalid send-data response")

        _msg, response_dst, status, sent_len = struct.unpack("<BHB B".replace(" ", ""), payload)
        result = SendDataResult(response_dst, Status(status), sent_len)
        if result.status != Status.OK:
            raise PdecProtocolError(Command.SEND_DATA, result.status)
        return result

    def set_discovery(self, enabled: bool, timeout: Optional[float] = None) -> bool:
        """Enable/disable outgoing discovery on the connected module until reboot."""
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a bool")
        self._send_frame(bytes([Command.SET_DISCOVERY, int(enabled)]))
        payload = self._wait_for(Message.DISCOVERY_STATE, timeout)
        if len(payload) != 3 or payload[0] != Message.DISCOVERY_STATE:
            raise PdecUartError("invalid discovery-state response")
        if payload[1] != Status.OK:
            raise PdecProtocolError(Command.SET_DISCOVERY, payload[1])
        if payload[2] not in (0, 1):
            raise PdecUartError("invalid discovery enabled value")
        return bool(payload[2])

    def command(self, line: str) -> object:
        parts = line.strip().split()
        if not parts:
            return None

        cmd = parts[0].lower()
        if cmd == "monitor":
            if len(parts) >= 2 and parts[1].lower() in {"off", "stop", "0", "false"}:
                self.stop_monitor()
                return "monitor stopped"
            self.start_monitor()
            return "monitor started"

        if cmd == "tdoa" and len(parts) >= 2 and parts[1].lower() == "monitor":
            if len(parts) >= 3 and parts[2].lower() in {"off", "stop", "0", "false"}:
                self.stop_monitor()
                return "monitor stopped"
            self.start_monitor()
            return "monitor started"

        if cmd == "range" and len(parts) == 2:
            return self.range(_parse_id(parts[1]))

        if cmd == "get" and len(parts) == 2:
            return self.get_distance(_parse_id(parts[1]))

        if cmd == "list" and len(parts) == 1:
            return self.list_devices()

        if cmd == "discovery" and len(parts) == 2 and parts[1].lower() in {"on", "off"}:
            return self.set_discovery(parts[1].lower() == "on")

        if cmd == "send" and len(parts) >= 3:
            return self.send_data(_parse_id(parts[1]), " ".join(parts[2:]))

        raise ValueError("commands: monitor [off], tdoa monitor [off], range <id>, get <id>, list, discovery on|off, send <id> <message>")

    def _send_frame(self, payload: bytes) -> None:
        if len(payload) > self.MAX_PAYLOAD_LEN:
            raise ValueError(f"binary payload is limited to {self.MAX_PAYLOAD_LEN} bytes")

        frame = self.SYNC + bytes([len(payload)]) + payload + bytes([_checksum(payload)])
        with self._write_lock:
            self.serial.write(frame)

    def _wait_for(self, msg: Message, timeout: Optional[float]) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        response_queue = self._responses[msg]

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise PdecTimeoutError(f"timeout waiting for {msg.name}")

            payload = response_queue.get(timeout=remaining)
            if payload and payload[0] == Message.ERROR:
                if len(payload) >= 3:
                    raise PdecProtocolError(payload[1], payload[2])
                raise PdecUartError("malformed error response")
            return payload

    def _reader_loop(self) -> None:
        while not self._stop_reader.is_set():
            payload = self._read_frame()
            if payload is None:
                continue
            self._dispatch_payload(payload)

    def _read_frame(self) -> Optional[bytes]:
        if self.serial.read(1) != b"\xff":
            return None
        if self.serial.read(1) != b"\xff":
            return None

        len_raw = self.serial.read(1)
        if len(len_raw) != 1:
            return None

        payload_len = len_raw[0]
        payload = self.serial.read(payload_len)
        chk_raw = self.serial.read(1)
        if len(payload) != payload_len or len(chk_raw) != 1:
            return None
        if chk_raw[0] != _checksum(payload):
            return None
        return payload

    def _dispatch_payload(self, payload: bytes) -> None:
        msg_id = payload[0]
        if msg_id == Message.RANGING_EVENT:
            event = self._parse_ranging_event(payload)
            if self._monitor_thread is not None and self._monitor_thread.is_alive():
                self._monitor_queue.put(event)
            for handler in list(self._event_handlers):
                handler(event)
            return

        if msg_id == Message.DATA_EVENT:
            event = self._parse_data_event(payload)
            for handler in list(self._event_handlers):
                handler(event)
            return

        if msg_id == Message.TDOA_REPORT_EVENT:
            event = self._parse_tdoa_report_event(payload)
            if self._monitor_thread is not None and self._monitor_thread.is_alive():
                self._monitor_queue.put(event)
            for handler in list(self._event_handlers):
                handler(event)
            return

        if msg_id == Message.ERROR:
            for response_queue in self._responses.values():
                response_queue.put(payload)
            return

        if msg_id in Message._value2member_map_:
            msg = Message(msg_id)
            if msg in self._responses:
                self._responses[msg].put(payload)

    @staticmethod
    def _parse_distance_result(payload: bytes, expected_msg: Message) -> DistanceResult:
        if len(payload) != 8 or payload[0] != expected_msg:
            raise PdecUartError(f"invalid {expected_msg.name.lower()} response")

        _msg, dst_id, status, distance = struct.unpack("<BHBf", payload)
        return DistanceResult(dst_id, Status(status), distance)

    @staticmethod
    def _parse_ranging_event(payload: bytes) -> RangingEvent:
        if len(payload) != 10:
            raise PdecUartError("invalid ranging event")

        _msg, kind, src_id, dst_id, distance = struct.unpack("<BBHHf", payload)
        return RangingEvent(EventKind(kind), src_id, dst_id, distance)

    @staticmethod
    def _parse_data_event(payload: bytes) -> DataEvent:
        if len(payload) < 4 or payload[0] != Message.DATA_EVENT:
            raise PdecUartError("invalid data event")

        _msg, src_id, data_len = struct.unpack("<BHB", payload[:4])
        data = payload[4:]
        if len(data) != data_len:
            raise PdecUartError("truncated data event")
        return DataEvent(src_id, data)

    @staticmethod
    def _parse_tdoa_report_event(payload: bytes) -> TdoaReportEvent:
        header_len = 8
        if len(payload) < header_len or payload[0] != Message.TDOA_REPORT_EVENT:
            raise PdecUartError("invalid TDOA report event")

        _msg, blink_id, expected_count, count, timed_out = struct.unpack("<BIBBB", payload[:header_len])
        expected_len = header_len + (count * 6)
        if len(payload) != expected_len:
            raise PdecUartError("truncated TDOA report event")

        reports: list[TdoaReportEntry] = []
        offset = header_len
        for _ in range(count):
            reporter_id, distance_m = struct.unpack("<Hf", payload[offset:offset + 6])
            reports.append(TdoaReportEntry(reporter_id, distance_m))
            offset += 6

        return TdoaReportEvent(blink_id, expected_count, bool(timed_out), tuple(reports))

    def _monitor_loop(self, printer: Callable[[str], None]) -> None:
        while not self._monitor_stop.is_set():
            try:
                event = self._monitor_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            printer(format_event(event))


def format_event(event: RangingEvent | TdoaReportEvent) -> str:
    if isinstance(event, TdoaReportEvent):
        suffix = " timeout" if event.timed_out else ""
        reports = ", ".join(f"0x{r.reporter_id:04x}: {r.distance_m:.02f}m" for r in event.reports)
        return f"[TDOA] blink {event.blink_id} reports {len(event.reports)}/{event.expected_count}{suffix}: {reports}"

    label = EVENT_KIND_NAMES.get(event.kind, f"0x{event.kind:02x}")
    return f"[{label}] 0x{event.src_id:04x} <-> 0x{event.dst_id:04x}: {event.distance_m:.02f}m"


def _checksum(payload: bytes) -> int:
    checksum = len(payload)
    for byte in payload:
        checksum ^= byte
    return checksum


def _parse_id(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0 or parsed > 0xFFFF:
        raise ValueError("device ID must fit in 16 bits")
    return parsed
