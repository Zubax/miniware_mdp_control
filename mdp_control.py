#!/usr/bin/env python3
"""Control paired Miniware MDP-P906 supplies through a USB-connected MDP-M01.

The M01 serial port is auto-detected unless `--port` is supplied.

## Installation

    python3 -m pip install .

## Command syntax

`status`
: Read all six channel slots without changing them.

`chN ACTION... [chN ACTION...]...`
: Apply one or more channel clauses as a coordinated batch. Each action is a voltage, current limit, or output state.
  Clauses and actions are case-insensitive, and actions within a clause may appear in any order.

Values must include `V`, `mV`, `A`, or `mA`. A missing voltage or current reuses the configured value. A missing
`on`/`off` preserves the existing output state.

## Examples

Read status:

    mdp-control status
    mdp-control --json status

Configure channel 1 and turn it on:

    mdp-control ch1 9V 0.75A on
    mdp-control CH1 ON 750mA 9v

Reuse channel 1's configured voltage and current limit:

    mdp-control ch1 on
    mdp-control ch1 off

Apply several channel clauses in one invocation:

    mdp-control ch1 9V 750mA on ch2 5V 1A on ch3 off

Change one limit while preserving the channel's output state:

    mdp-control ch1 3300mV
    mdp-control ch2 250mA

Select a serial port and response timeout:

    mdp-control --port /dev/ttyACM0 --timeout 5 ch1 on

## Batch behavior

The complete batch is validated before any control command is sent. Outputs requested off are disabled first, all
setpoints are then sent, and outputs requested on are enabled last. If an already-on channel includes both new setpoints
and an explicit `on`, it is briefly disabled while the setpoints are changed.

The M01 wireless protocol is not transactional. A connection failure can still leave a batch only partly applied.

## Python API

`MDPController`, `ChannelCommand`, `Status`, `ChannelStatus`, and the `MDPError` exception hierarchy form the public
API. All library-defined exceptions inherit from `MDPError`. For example:

    from mdp_control import ChannelCommand, MDPController, MDPError

    try:
        with MDPController() as mdp:
            status = mdp.apply([
                ChannelCommand(1, voltage=9.0, current=0.75, output=True),
                ChannelCommand(2, output=False),
            ])
            print(status)
    except MDPError as exc:
        print(f"MDP command failed: {exc}")

`MDPController` also provides `set_voltage()`, `set_current()`, `set_limits()`, and `set_output()` as single-channel
convenience methods.

## Compatibility

The original single-command syntax remains available for existing scripts:

    mdp-control --channel 1 set 5 0.5

## Safety

Enabling an output can energize connected hardware. Verify the setpoints, polarity, wiring, and load before using `on`.
"""

import argparse as _argparse
import json as _json
import math as _math
import os as _os
import re as _re
import sys as _sys
import time as _time
from dataclasses import asdict as _asdict
from dataclasses import dataclass as _dataclass
from enum import IntEnum as _IntEnum
from functools import reduce as _reduce
from operator import xor as _xor_operator
from typing import Any as _Any
from typing import Callable as _Callable
from typing import Iterable as _Iterable
from typing import Sequence as _Sequence

_SERIAL_IMPORT_ERROR: ImportError | None
try:
    import serial as _serial
    from serial.tools import list_ports as _list_ports
except ImportError as exc:  # pragma: no cover - environment-specific
    _serial = None
    _list_ports = None
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


__all__ = [
    "ChannelCommand",
    "ChannelStatus",
    "MDPCommandTimeoutError",
    "MDPConnectionError",
    "MDPController",
    "MDPDependencyError",
    "MDPDeviceNotFoundError",
    "MDPDeviceStateError",
    "MDPError",
    "MDPProtocolError",
    "MDPValidationError",
    "Status",
]


_BAUD_RATE = 115_200
_MAGIC = b"\x5a\x5a"
_HEADER_SIZE = 6
_CHANNEL_COUNT = 6
_SYNTH_RECORD_MIN_SIZE = 24
_P906_MAX_VOLTAGE = 30.0
_P906_MAX_CURRENT = 10.0
_P906_MAX_POWER_MILLI = 300_000_000  # mV * mA for 300 W

_CHANNEL_TOKEN = _re.compile(r"(?:ch|channel)([1-6])", _re.IGNORECASE)
_QUANTITY_TOKEN = _re.compile(
    r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?)(m?[av])",
    _re.IGNORECASE,
)


class MDPError(Exception):
    """Base class for every error raised by this module's public API."""


class MDPDependencyError(MDPError):
    """A required runtime dependency is unavailable."""


class MDPConnectionError(MDPError):
    """The MDP-M01 serial connection could not be opened or used."""


class MDPDeviceNotFoundError(MDPConnectionError):
    """No suitable MDP-M01 serial port could be found."""


class MDPProtocolError(MDPError):
    """The MDP-M01 returned an invalid or unsupported response."""


class MDPCommandTimeoutError(MDPError):
    """A requested state was not confirmed before its deadline."""


class MDPValidationError(MDPError):
    """A command or value is invalid."""


class MDPDeviceStateError(MDPError):
    """The requested operation is incompatible with current device state."""


class _PacketType(_IntEnum):
    SYNTHESIZE = 0x11
    WAVE = 0x12
    ADDRESS = 0x13
    UPDATE_CHANNEL = 0x14
    MACHINE = 0x15
    SET_OUTPUT = 0x16
    GET_ADDRESS = 0x17
    SET_ADDRESS = 0x18
    SET_CHANNEL = 0x19
    SET_VOLTAGE = 0x1A
    SET_CURRENT = 0x1B
    SET_ALL_ADDRESSES = 0x1C
    START_AUTO_MATCH = 0x1D
    STOP_AUTO_MATCH = 0x1E
    RESET_TO_DFU = 0x1F
    RGB = 0x20
    GET_MACHINE = 0x21
    HEARTBEAT = 0x22
    ERROR_240 = 0x23


_MACHINE_NAMES = {
    0: "unconfigured",
    1: "P905",
    2: "P906",
    3: "L1060",
}

_P906_MODE_NAMES = {
    0: "OFF",
    1: "CC",
    2: "CV",
    3: "ON",
}


@_dataclass(frozen=True)
class _Packet:
    type: int
    channel: int
    payload: bytes


@_dataclass(frozen=True)
class ChannelStatus:
    """One MDP-M01 channel's latest reported state; channels are 1-based."""

    channel: int
    number: int
    machine_type: int
    machine: str
    online: bool
    output_enabled: bool
    mode: str
    locked: bool
    error: bool
    voltage: float
    current: float
    power: float
    set_voltage: float
    current_limit: float
    input_voltage: float
    input_current: float
    temperature_c: float


@_dataclass(frozen=True)
class Status:
    """A complete six-channel MDP-M01 status snapshot."""

    selected_channel: int
    channels: tuple[ChannelStatus, ...]


@_dataclass(frozen=True)
class ChannelCommand:
    """Desired changes for one channel; omitted fields preserve current state."""

    channel: int
    voltage: float | None = None
    current: float | None = None
    output: bool | None = None


@_dataclass(frozen=True)
class _ResolvedCommand:
    command: ChannelCommand
    before: ChannelStatus
    voltage_mv: int
    current_ma: int

    @property
    def has_setpoints(self) -> bool:
        return self.command.voltage is not None or self.command.current is not None

    @property
    def target_output(self) -> bool:
        if self.command.output is None:
            return self.before.output_enabled
        return self.command.output


def _require_pyserial() -> None:
    if _serial is None or _list_ports is None:
        detail = f" ({_SERIAL_IMPORT_ERROR})" if _SERIAL_IMPORT_ERROR else ""
        raise MDPDependencyError("pyserial is required; install it with: " f"python3 -m pip install pyserial{detail}")


def _xor(data: _Iterable[int]) -> int:
    return _reduce(_xor_operator, data, 0)


def _build_packet(packet_type: int, channel: int = 0xEE, payload: bytes = b"") -> bytes:
    """Build one host-to-MDP-M01 packet."""
    size = _HEADER_SIZE + len(payload)
    if not 0 <= packet_type <= 0xFF:
        raise MDPValidationError("packet type must fit in one byte")
    if not 0 <= channel <= 0xFF:
        raise MDPValidationError("channel must fit in one byte")
    if size > 0xFF:
        raise MDPValidationError("packet is too large")
    return _MAGIC + bytes((packet_type, size, channel, _xor(payload))) + payload


class _PacketParser:
    """Incremental parser that resynchronizes after noise or a corrupt packet."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[_Packet]:
        self._buffer.extend(data)
        packets: list[_Packet] = []

        while True:
            start = self._buffer.find(_MAGIC)
            if start < 0:
                # Retain a possible first header byte split across reads.
                self._buffer[:] = self._buffer[-1:] if self._buffer.endswith(b"Z") else b""
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                break

            size = self._buffer[3]
            if size < _HEADER_SIZE:
                del self._buffer[0]
                continue
            if len(self._buffer) < size:
                break

            raw = bytes(self._buffer[:size])
            if _xor(raw[_HEADER_SIZE:]) != raw[5]:
                del self._buffer[0]
                continue

            del self._buffer[:size]
            packets.append(_Packet(type=raw[2], channel=raw[4], payload=raw[6:]))

        return packets


def _u16le(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def _parse_status(packet: _Packet) -> Status:
    """Decode a synthesis/status packet emitted by the MDP-M01."""
    if packet.type != _PacketType.SYNTHESIZE:
        raise MDPProtocolError(f"expected synthesis packet, got type 0x{packet.type:02x}")
    if len(packet.payload) % _CHANNEL_COUNT:
        raise MDPProtocolError(f"status payload has invalid size {len(packet.payload)} " "(not divisible by 6)")

    record_size = len(packet.payload) // _CHANNEL_COUNT
    if record_size < _SYNTH_RECORD_MIN_SIZE:
        raise MDPProtocolError(f"status channel record is only {record_size} bytes; expected at least 24")

    channels: list[ChannelStatus] = []
    for index in range(_CHANNEL_COUNT):
        record = packet.payload[index * record_size : (index + 1) * record_size]
        voltage_mv = _u16le(record, 1)
        current_ma = _u16le(record, 3)
        machine_type = record[16]
        raw_mode = record[18]
        voltage = voltage_mv / 1000.0
        current = current_ma / 1000.0
        channels.append(
            ChannelStatus(
                channel=index + 1,
                number=record[0],
                machine_type=machine_type,
                machine=_MACHINE_NAMES.get(machine_type, f"unknown({machine_type})"),
                online=record[15] == 1,
                output_enabled=record[19] != 0,
                mode=_P906_MODE_NAMES.get(raw_mode, f"UNKNOWN({raw_mode})"),
                locked=record[17] == 1,
                error=record[23] == 1,
                voltage=voltage,
                current=current,
                power=voltage * current,
                set_voltage=_u16le(record, 9) / 1000.0,
                current_limit=_u16le(record, 11) / 1000.0,
                input_voltage=_u16le(record, 5) / 1000.0,
                input_current=_u16le(record, 7) / 1000.0,
                temperature_c=_u16le(record, 13) / 10.0,
            )
        )

    selected = packet.channel + 1 if packet.channel < _CHANNEL_COUNT else packet.channel
    return Status(selected_channel=selected, channels=tuple(channels))


def _find_mdp_port() -> str:
    """Find a Miniware MDP-M01 CDC port on Linux, macOS, or Windows."""
    _require_pyserial()
    try:
        ports = _list_ports.comports()
    except Exception as exc:
        raise MDPConnectionError(f"could not enumerate serial ports: {exc}") from exc

    candidates: list[str] = []
    for port in ports:
        fields = " ".join(
            str(value or "")
            for value in (
                port.manufacturer,
                port.product,
                port.serial_number,
                port.description,
                port.hwid,
            )
        ).lower()
        # Current M01 firmware identifies as 0416:dc01, with "Miniware" as
        # its USB serial string. Match the string first to avoid unrelated CDC
        # devices which happen to reuse the VID/PID.
        if "miniware" in fields or (port.vid == 0x0416 and port.pid == 0xDC01):
            candidates.append(port.device)

    if not candidates:
        raise MDPDeviceNotFoundError("no MDP-M01 serial port found; connect it by USB or pass --port")
    if len(candidates) > 1:
        joined = ", ".join(candidates)
        raise MDPDeviceNotFoundError(f"multiple possible MDP-M01 ports found ({joined}); select one with --port")
    return candidates[0]


class MDPController:
    """Python API for an MDP-M01 and its paired P906 channels."""

    def __init__(self, port: str | None = None, timeout: float = 3.0) -> None:
        if (
            isinstance(timeout, bool)
            or not isinstance(timeout, (int, float))
            or not _math.isfinite(timeout)
            or timeout <= 0
        ):
            raise MDPValidationError("timeout must be a finite value greater than zero")
        self.port = port or _find_mdp_port()
        self.timeout = timeout
        self._serial: _Any = None
        self._parser = _PacketParser()

    def __enter__(self) -> "MDPController":
        _require_pyserial()
        kwargs: dict[str, object] = {}
        if _os.name == "posix":
            kwargs["exclusive"] = True
        try:
            self._serial = _serial.Serial(
                self.port,
                _BAUD_RATE,
                bytesize=_serial.EIGHTBITS,
                parity=_serial.PARITY_NONE,
                stopbits=_serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                **kwargs,
            )
            self._serial.reset_input_buffer()
        except (OSError, _serial.SerialException) as exc:
            self._serial = None
            raise MDPConnectionError(f"could not open {self.port}: {exc}") from exc
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        _exception: BaseException | None,
        _traceback: object,
    ) -> None:
        serial_port = self._serial
        self._serial = None
        if serial_port is None:
            return
        try:
            serial_port.close()
        except (OSError, _serial.SerialException) as exc:
            if exception_type is None:
                raise MDPConnectionError(f"could not close {self.port}: {exc}") from exc

    @property
    def _serial_port(self) -> _Any:
        if self._serial is None:
            raise MDPConnectionError("MDPController must be open (normally with a 'with' statement)")
        return self._serial

    def _send(
        self,
        packet_type: _PacketType,
        channel: int = 0xEE,
        payload: bytes = b"",
    ) -> None:
        try:
            self._serial_port.write(_build_packet(packet_type, channel, payload))
            self._serial_port.flush()
        except (OSError, _serial.SerialException) as exc:
            raise MDPConnectionError(f"write to {self.port} failed: {exc}") from exc

    def _heartbeat(self) -> None:
        self._send(_PacketType.HEARTBEAT)

    def _packets_until(self, deadline: float) -> _Iterable[_Packet]:
        while _time.monotonic() < deadline:
            try:
                chunk = self._serial_port.read(4096)
            except (OSError, _serial.SerialException) as exc:
                raise MDPConnectionError(f"read from {self.port} failed: {exc}") from exc
            if chunk:
                yield from self._parser.feed(chunk)

    def read_status(
        self,
        timeout: float | None = None,
        predicate: _Callable[[Status], bool] | None = None,
    ) -> Status:
        """Request a snapshot, optionally waiting until a condition is true."""
        wait = self.timeout if timeout is None else timeout
        if isinstance(wait, bool) or not isinstance(wait, (int, float)) or not _math.isfinite(wait) or wait <= 0:
            raise MDPValidationError("timeout must be a finite value greater than zero")

        deadline = _time.monotonic() + wait
        self._heartbeat()
        next_heartbeat = _time.monotonic() + 1.0
        latest: Status | None = None

        for packet in self._packets_until(deadline):
            now = _time.monotonic()
            if now >= next_heartbeat:
                self._heartbeat()
                next_heartbeat = now + 1.0
            if packet.type != _PacketType.SYNTHESIZE:
                continue
            latest = _parse_status(packet)
            if predicate is None or predicate(latest):
                return latest

        if latest is None:
            raise MDPCommandTimeoutError(f"no valid status packet received from {self.port} within {wait:.1f}s")
        raise MDPCommandTimeoutError(f"the requested state was not confirmed within {wait:.1f}s")

    @staticmethod
    def _channel(status: Status, channel: int, *, allow_locked: bool = False) -> ChannelStatus:
        if isinstance(channel, bool) or not isinstance(channel, int):
            raise MDPValidationError("channel must be an integer between 1 and 6")
        if not 1 <= channel <= _CHANNEL_COUNT:
            raise MDPValidationError("channel must be between 1 and 6")
        state = status.channels[channel - 1]
        if not state.online:
            raise MDPDeviceStateError(f"channel {channel} is offline")
        if state.machine_type != 2:
            raise MDPDeviceStateError(f"channel {channel} contains {state.machine}, not an MDP-P906")
        if state.locked and not allow_locked:
            raise MDPDeviceStateError(f"channel {channel} is locked on the MDP-M01")
        return state

    @staticmethod
    def _millivalue(value: float, name: str, maximum: float) -> int:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise MDPValidationError(f"{name} must be a number")
        if not _math.isfinite(value) or not 0.0 <= value <= maximum:
            raise MDPValidationError(f"{name} must be a finite value between 0 and {maximum:g}")
        return int(value * 1000.0 + 0.5)

    def _resolve_commands(self, commands: _Iterable[ChannelCommand], before: Status) -> tuple[_ResolvedCommand, ...]:
        try:
            command_tuple = tuple(commands)
        except TypeError as exc:
            raise MDPValidationError("commands must be a ChannelCommand or an iterable of them") from exc
        if not command_tuple:
            raise MDPValidationError("at least one channel command is required")

        seen_channels: set[int] = set()
        resolved: list[_ResolvedCommand] = []
        for command in command_tuple:
            if not isinstance(command, ChannelCommand):
                raise MDPValidationError("commands must be ChannelCommand instances")
            off_only = command.output is False and command.voltage is None and command.current is None
            state = self._channel(before, command.channel, allow_locked=off_only)
            if command.channel in seen_channels:
                raise MDPValidationError(f"channel {command.channel} appears more than once in the batch")
            seen_channels.add(command.channel)
            if command.voltage is None and command.current is None and command.output is None:
                raise MDPValidationError(f"channel {command.channel} does not contain an action")
            if command.output is not None and not isinstance(command.output, bool):
                raise MDPValidationError("output must be True, False, or None")

            voltage = state.set_voltage if command.voltage is None else command.voltage
            current = state.current_limit if command.current is None else command.current
            voltage_mv = self._millivalue(voltage, "voltage", _P906_MAX_VOLTAGE)
            current_ma = self._millivalue(current, "current", _P906_MAX_CURRENT)
            if voltage_mv * current_ma > _P906_MAX_POWER_MILLI:
                raise MDPValidationError(f"channel {command.channel} requests more than the P906 300 W limit")
            resolved.append(_ResolvedCommand(command, state, voltage_mv, current_ma))

        return tuple(resolved)

    def _send_output(self, channel: int, enabled: bool) -> None:
        self._send(
            _PacketType.SET_OUTPUT,
            channel - 1,
            bytes((int(enabled),)),
        )

    def _send_setpoints(self, resolved: _ResolvedCommand) -> None:
        payload = resolved.voltage_mv.to_bytes(2, "little") + (resolved.current_ma.to_bytes(2, "little"))
        packet_types: list[_PacketType] = []
        if resolved.command.voltage is not None:
            packet_types.append(_PacketType.SET_VOLTAGE)
        if resolved.command.current is not None:
            packet_types.append(_PacketType.SET_CURRENT)

        for packet_type in packet_types:
            # Miniware's application repeats setpoint packets to improve the
            # reliability of the M01-to-P906 wireless hop.
            self._send(packet_type, resolved.command.channel - 1, payload)
            _time.sleep(0.03)
            self._send(packet_type, resolved.command.channel - 1, payload)
            _time.sleep(0.03)

    @staticmethod
    def _matches(status: Status, commands: tuple[_ResolvedCommand, ...]) -> bool:
        for resolved in commands:
            state = status.channels[resolved.command.channel - 1]
            if not state.online:
                return False
            if resolved.has_setpoints and (
                abs(state.set_voltage * 1000 - resolved.voltage_mv) >= 0.5
                or abs(state.current_limit * 1000 - resolved.current_ma) >= 0.5
            ):
                return False
            if state.output_enabled != resolved.target_output:
                return False
        return True

    def apply(self, commands: ChannelCommand | _Iterable[ChannelCommand]) -> Status:
        """Validate and apply one or more channel changes as a coordinated batch.

        All commands are validated before the first write. Requested-off
        channels are disabled first, then every setpoint is written, and only
        then are requested-on channels enabled. The serial/wireless protocol is
        not transactional, so a physical disconnect can still interrupt a
        partially applied batch.
        """
        command_iterable: _Iterable[ChannelCommand]
        if isinstance(commands, ChannelCommand):
            command_iterable = (commands,)
        else:
            command_iterable = commands

        before = self.read_status()
        resolved_commands = self._resolve_commands(command_iterable, before)
        pre_disabled: set[int] = set()

        # Safest ordering: disable first if OFF was requested, or if new
        # setpoints and ON were requested for a channel that is already live.
        for resolved in resolved_commands:
            needs_safe_cycle = (
                resolved.command.output is True and resolved.has_setpoints and resolved.before.output_enabled
            )
            needs_off = resolved.command.output is False and resolved.before.output_enabled
            if needs_safe_cycle or needs_off:
                self._send_output(resolved.command.channel, False)
                pre_disabled.add(resolved.command.channel)
        if pre_disabled:
            _time.sleep(0.05)

        for resolved in resolved_commands:
            if resolved.has_setpoints:
                self._send_setpoints(resolved)

        # Enable only after every channel's setpoints have been sent.
        for resolved in resolved_commands:
            should_enable = resolved.command.output is True and (
                not resolved.before.output_enabled or resolved.command.channel in pre_disabled
            )
            if should_enable:
                self._send_output(resolved.command.channel, True)

        changed = any(
            resolved.has_setpoints
            or (resolved.command.output is not None and resolved.command.output != resolved.before.output_enabled)
            for resolved in resolved_commands
        )
        if not changed:
            return before
        return self.read_status(predicate=lambda status: self._matches(status, resolved_commands))

    def set_voltage(self, channel: int, voltage: float) -> Status:
        """Set voltage while preserving the current limit and output state."""
        return self.apply(ChannelCommand(channel=channel, voltage=voltage))

    def set_current(self, channel: int, current: float) -> Status:
        """Set current limit while preserving voltage and output state."""
        return self.apply(ChannelCommand(channel=channel, current=current))

    def set_limits(self, channel: int, voltage: float, current: float) -> Status:
        """Set both limits while preserving output state."""
        return self.apply(ChannelCommand(channel=channel, voltage=voltage, current=current))

    def set_output(self, channel: int, enabled: bool) -> Status:
        """Enable or disable a channel while preserving its configured limits."""
        return self.apply(ChannelCommand(channel=channel, output=enabled))


def _status_dict(status: Status, port: str) -> dict[str, object]:
    return {
        "port": port,
        "selected_channel": status.selected_channel,
        "channels": [_asdict(channel) for channel in status.channels],
    }


def _print_status(status: Status, port: str, as_json: bool = False) -> None:
    if as_json:
        print(_json.dumps(_status_dict(status, port), indent=2))
        return

    print(f"MDP-M01: {port} (selected channel {status.selected_channel})")
    for channel in status.channels:
        if not channel.online:
            print(f"CH{channel.channel}: offline")
            continue
        output = "ON" if channel.output_enabled else "OFF"
        flags = []
        if channel.locked:
            flags.append("locked")
        if channel.error:
            flags.append("error")
        suffix = f" [{', '.join(flags)}]" if flags else ""
        print(
            f"CH{channel.channel}: {channel.machine} online, output {output}, "
            f"mode {channel.mode}{suffix}\n"
            f"  actual {channel.voltage:.3f} V / {channel.current:.3f} A "
            f"({channel.power:.3f} W); set {channel.set_voltage:.3f} V / "
            f"{channel.current_limit:.3f} A\n"
            f"  input {channel.input_voltage:.3f} V / "
            f"{channel.input_current:.3f} A; temperature "
            f"{channel.temperature_c:.1f} °C"
        )


def _parse_quantity(token: str) -> tuple[str, float] | None:
    match = _QUANTITY_TOKEN.fullmatch(token)
    if match is None:
        return None
    value = float(match.group(1))
    unit = match.group(2).lower()
    if unit.startswith("m"):
        value /= 1000.0
        unit = unit[1:]
    return unit, value


def _parse_channel_clauses(tokens: _Sequence[str]) -> tuple[ChannelCommand, ...]:
    if tokens and tokens[0].casefold() == "apply":
        tokens = tokens[1:]
    if not tokens:
        raise MDPValidationError("no channel commands were provided")

    commands: list[ChannelCommand] = []
    channel: int | None = None
    voltage: float | None = None
    current: float | None = None
    output: bool | None = None
    has_action = False

    def finish_clause() -> None:
        nonlocal channel, voltage, current, output, has_action
        if channel is None:
            return
        if not has_action:
            raise MDPValidationError(f"ch{channel} does not contain an action")
        commands.append(ChannelCommand(channel, voltage, current, output))

    for token in tokens:
        channel_match = _CHANNEL_TOKEN.fullmatch(token)
        if channel_match is not None:
            finish_clause()
            channel = int(channel_match.group(1))
            voltage = None
            current = None
            output = None
            has_action = False
            continue
        if channel is None:
            raise MDPValidationError(f"expected a channel such as ch1, got {token!r}")

        state = token.casefold()
        if state in {"on", "off"}:
            if output is not None:
                raise MDPValidationError(f"ch{channel} contains more than one output state")
            output = state == "on"
            has_action = True
            continue

        quantity = _parse_quantity(token)
        if quantity is None:
            raise MDPValidationError(
                f"unrecognized token {token!r} in ch{channel}; " "use values such as 9V, 750mA, and on/off"
            )
        unit, value = quantity
        if unit == "v":
            if voltage is not None:
                raise MDPValidationError(f"ch{channel} contains more than one voltage")
            voltage = value
        else:
            if current is not None:
                raise MDPValidationError(f"ch{channel} contains more than one current limit")
            current = value
        has_action = True

    finish_clause()
    duplicate = next(
        (
            command.channel
            for index, command in enumerate(commands)
            if command.channel in {item.channel for item in commands[:index]}
        ),
        None,
    )
    if duplicate is not None:
        raise MDPValidationError(f"ch{duplicate} appears more than once")
    return tuple(commands)


def _parse_legacy_command(tokens: _Sequence[str], channel: int) -> tuple[ChannelCommand, ...]:
    command = tokens[0].casefold()
    expected_lengths = {"set": 3, "voltage": 2, "current": 2, "output": 2}
    expected = expected_lengths[command]
    if len(tokens) != expected:
        raise MDPValidationError(f"legacy {command!r} syntax expects {expected - 1} argument(s)")
    try:
        if command == "set":
            result = ChannelCommand(channel, float(tokens[1]), float(tokens[2]))
        elif command == "voltage":
            result = ChannelCommand(channel, voltage=float(tokens[1]))
        elif command == "current":
            result = ChannelCommand(channel, current=float(tokens[1]))
        else:
            state = tokens[1].casefold()
            if state not in {"on", "off"}:
                raise MDPValidationError("output state must be on or off")
            result = ChannelCommand(channel, output=state == "on")
    except ValueError as exc:
        raise MDPValidationError(f"invalid numeric value: {exc}") from exc
    return (result,)


def _parse_cli_tokens(tokens: _Sequence[str], legacy_channel: int | None = None) -> tuple[ChannelCommand, ...] | None:
    if not tokens:
        raise MDPValidationError("a command is required")
    if tokens[0].casefold() == "status":
        if len(tokens) != 1:
            raise MDPValidationError("status does not accept channel actions")
        if legacy_channel is not None:
            raise MDPValidationError("--channel is not meaningful with status")
        return None

    legacy_commands = {"set", "voltage", "current", "output"}
    if tokens[0].casefold() in legacy_commands:
        return _parse_legacy_command(tokens, legacy_channel or 1)
    if legacy_channel is not None:
        raise MDPValidationError("--channel is only supported with the legacy command syntax")
    return _parse_channel_clauses(tokens)


def _make_argument_parser() -> _argparse.ArgumentParser:
    parser = _argparse.ArgumentParser(
        description=__doc__,
        formatter_class=_argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", help="serial port (auto-detected when omitted)")
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="response timeout in seconds (default: 3)",
    )
    parser.add_argument("--json", action="store_true", help="print status as JSON")
    # Retained so existing scripts using `--channel 1 set 5 0.5` keep working.
    parser.add_argument(
        "--channel",
        "-c",
        dest="legacy_channel",
        type=int,
        choices=range(1, _CHANNEL_COUNT + 1),
        help=_argparse.SUPPRESS,
    )
    parser.add_argument("command_tokens", nargs="+", metavar="COMMAND")
    return parser


def _run(argv: _Sequence[str] | None = None) -> int:
    parser = _make_argument_parser()
    args = parser.parse_args(argv)
    try:
        commands = _parse_cli_tokens(args.command_tokens, args.legacy_channel)
        if not _math.isfinite(args.timeout) or args.timeout <= 0:
            raise MDPValidationError("--timeout must be a finite value greater than zero")
    except MDPValidationError as exc:
        parser.error(str(exc))

    with MDPController(port=args.port, timeout=args.timeout) as controller:
        status = controller.read_status() if commands is None else controller.apply(commands)
        _print_status(status, controller.port, as_json=args.json)
    return 0


def _main(argv: _Sequence[str] | None = None) -> int:
    try:
        return _run(argv)
    except MDPError as exc:
        print(f"error: {exc}", file=_sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(_main())
