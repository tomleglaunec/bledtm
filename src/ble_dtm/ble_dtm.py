from __future__ import annotations
from abc import ABC
from typing import Optional
from enum import IntEnum
import serial


class Command(IntEnum):
    LE_TEST_SETUP = 0b00
    LE_RECEIVER_TEST = 0b01
    LE_TRANSMITTER_TEST = 0b10
    LE_TEST_END = 0b11


class PacketType(IntEnum):
    PRBS9 = 0b00
    HALF = 0b01
    ALT = 0b10
    VENDOR_SPECIFIC = 0b11


class Event(IntEnum):
    LE_TEST_STATUS = 0
    LE_PACKET_REPORT = 1


class Status(IntEnum):
    SUCCESS = 0
    ERROR = 1


class DirectTestMode:
    """Initialize Direct Test Mode over serial. Either a `serial.Serial` instance or a port and options should be provided. Passing both ser and port will raise a TypeError. Serial options are unused if a custom serial instance is provided."""

    def __init__(
        self,
        *,
        ser: Optional[serial.Serial] = None,
        port: Optional[str] = None,
        baudrate: int = 9600,
        timeout: float = 0.1,
    ):
        if (ser is None and port is None) or ser and port:
            raise TypeError("Either serial instance OR port must be specified.")

        if ser:
            if not isinstance(ser, serial.Serial):
                raise TypeError("ser must be an instance of type serial.Serial.")
            ser.open()
            self._serial = ser
        else:
            self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def _write_test(self, cmd: int, frequency: int, length: int, pkt: int) -> None:
        # See paragraph 3.3.2 of Bluetooth Core v5.4, Vol 6, Part F.
        message = ((cmd << 14) + (frequency << 8) + (length << 2) + pkt).to_bytes(2)
        n = self._serial.write(message)
        if not n == 2:
            raise RuntimeWarning(f"{n} byte(s) were sent, expected 2.")

    def _write_setup(self, cmd: int, control: int, parameter: int) -> None:
        # See paragraph 3.3.2 of Bluetooth Core v5.4, Vol 6, Part F.
        message = ((cmd << 14) + (control << 8) + parameter).to_bytes(2)
        n = self._serial.write(message)
        if not n == 2:
            raise RuntimeWarning(f"{n} byte(s) were sent, expected 2.")

    def _read(self) -> DUTResponse:
        return DUTResponse.from_raw(self._serial.read(2))


class DUTResponse(ABC):
    """Represents the DUT response. A response (LE_Status or LE_Packet_Report) consist of 2 bytes."""

    _event: Event

    @classmethod
    def from_raw(cls, raw: bytes) -> DUTResponse:
        """Factory method that returns a new instance of a DUTResponse child class, depending on the event type.
        The
        """
        if not isinstance(raw, bytes):
            raise TypeError(
                f"Arg raw must be of type bytes, {raw.__class__.__name__} passed."
            )
        if not (l := len(raw)) == 2:
            raise ValueError(f"Arg raw should be of length 2 (bytes), {l} instead.")

        packet = int.from_bytes(raw)

        event_type = Event(packet & (1 << 15))
        cls = LE_Test_Status if event_type == Event.LE_TEST_STATUS else LE_Packet_Report

        return cls(packet)

    @property
    def event(self) -> Event:
        return self._event


class LE_Test_Status(DUTResponse):

    _st: Status

    def __init__(self, response: int) -> None:
        self._event = Event.LE_TEST_STATUS
        self._st = Status(response & 1)

    @property
    def status(self) -> Status:
        """Return the status contained in the response from DUT."""
        return self._st

    def decode_setup(self, ctrl: int):
        """Decode Response field contained in LE_Test_Status event.
        As to Core Specification V5.4, this field has a meaning only in response to a LE_Test_Setup command with control parameters `0x05` and `0x09`.
        Raise ReservedForFutureUseError for other control parameters code, or when status is Error, as the response field is Reserved for future use.
        """
        if self.status == Status.ERROR:
            raise ReservedForFutureUseError(
                "Nothing to decode in Error status, Reserved for future use."
            )

        if ctrl == 0x05:
            return ...

        if ctrl == 0x09:
            return ...

        raise ReservedForFutureUseError(
            f"Invalid control parameter code: {hex(ctrl)}. Please check function documentation."
        )


class LE_Packet_Report(DUTResponse):

    _packet_count: int

    def __init__(self, response: int) -> None:
        self._event = Event.LE_PACKET_REPORT
        self._packet_count = response & 0x7FFF

    @property
    def packet_count(self) -> int:
        """Returns the number of packets received by the DUT (buffer of 15 bits, so up to 32767).
        Note: The DUT is not responsible for any overflow conditions of the packet count. That responsibility belongs with the RFPHY Tester or other auxiliary equipment.
        """
        return self._packet_count

    def __repr__(self) -> str:
        return f"LE_Packet_Report({self.packet_count} packet(s) received)"


class ReservedForFutureUseError(Exception):
    pass
