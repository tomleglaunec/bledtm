from __future__ import annotations
from abc import ABC
from typing import Optional, cast
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
    """Initialize Direct Test Mode over serial. Either a `serial.Serial` instance or a port and options should be provided.
    Passing both ser and port will raise a TypeError. Serial options are unused if a custom serial instance is provided.
    """

    def __init__(
        self,
        *,
        ser: Optional[serial.Serial] = None,
        port: Optional[str] = None,
        baudrate: int = 9600,
        timeout: float = 0.1,
        reset: bool = True,
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

        if reset:
            rc = self.reset()
            if rc.status == Status.ERROR:
                raise RuntimeWarning(f"DUT failed to reset: {rc}")

    def reset(self) -> LE_Test_Status:
        """Resets DUT by sending a Le_Test_Setup command with control and parameter set to 0x0."""
        self._write_setup(Command.LE_TEST_SETUP, 0, 0)
        return cast(LE_Test_Status, self._read())

    def stop_test(self) -> LE_Packet_Report:
        """Stops current test by sending a LE_Test_End command. Returns Packet report answered by
        DUT."""
        # Core Specification permits Parameter from value 0x00 to 0x03 but doesn't explicitly specify behavior (no effect?).
        # This implementation will only send Parameter of value 0.
        self._write_setup(Command.LE_TEST_END, 0, 0)
        return cast(LE_Packet_Report, self._read())

    def start_transmitter_test(self, frequency: int, length: int, pkt: PacketType) -> LE_Test_Status:
        """Starts receiver test by sending a LE_Receiver_Test command.
        Parameters:
            frequency: channel N selection from 0 to 39; (2N + 2402) MHz (No correspondance with LE channels denomiation!)
            length: length of packet payload in bits (limited to 63, if longer packet are needed see the setup command,
            for up to 255)
            pkt: packet type. `PacketType.VENDOR_SPECIFIC` will result in vendor specific packet type for LE Uncoded PHYs and
            11111111 for LE Coded PHY.
        """
        self._write_test(Command.LE_TRANSMITTER_TEST, frequency, length, pkt)
        return cast(LE_Test_Status, self._read())

    def start_receiver_test(self, frequency: int, length: int, pkt: PacketType) -> LE_Test_Status:
        """Starts receiver test by sending a LE_Receiver_Test command.
        Parameters:
            frequency: channel N selection from 0 to 39; (2N + 2402) MHz (No correspondance with LE channels denomiation!)
            length: length of packet payload in bits (limited to 63, if longer packet are needed see the setup command,
            for up to 255)
            pkt: packet type. `PacketType.VENDOR_SPECIFIC` will result in vendor specific packet type for LE Uncoded PHYs and
            11111111 for LE Coded PHY.
        """
        self._write_test(Command.LE_RECEIVER_TEST, frequency, length, pkt)
        return cast(LE_Test_Status, self._read())

    def _write_test(self, cmd: int, frequency: int, length: int, pkt: PacketType) -> None:
        if frequency < 0 or frequency > 0x27:
            raise ReservedForFutureUseError("Argument frequency must be in range 0 to 39.")

        if length < 0 or length > 0x3F:
            raise ValueError("Argument length must be in range 0 to 63 (0x3F).")

        if not isinstance(pkt, PacketType):
            raise TypeError("Argument pkt must be of type PacketType.")
        # See paragraph 3.3.2 of Bluetooth Core v5.4, Vol 6, Part F.
        message = (cmd << 14) + (frequency << 8) + (length << 2) + pkt.value
        n = self._serial.write(message.to_bytes(2))
        if not n == 2:
            raise RuntimeWarning(f"{n} byte(s) were sent, expected 2.")

    def _write_setup(self, cmd: int, control: int, parameter: int) -> None:
        # See paragraph 3.3.2 of Bluetooth Core v5.4, Vol 6, Part F.
        message = (cmd << 14) + (control << 8) + parameter
        n = self._serial.write(message.to_bytes(2))
        if not n == 2:
            raise RuntimeWarning(f"{n} byte(s) were sent, expected 2.")

    def _read(self) -> DUTResponse:
        return DUTResponse.from_raw(self._serial.read(2))


class DUTResponse(ABC):
    """Represents the DUT response.
    A response (LE_Status or LE_Packet_Report) consist of 2 bytes.
    """

    _event: Event

    @staticmethod
    def from_raw(raw: bytes) -> DUTResponse:
        """Factory method that returns a new instance of a DUTResponse child class,
        depending on the event type.
        """
        if not isinstance(raw, bytes):
            raise TypeError(f"Arg raw must be of type bytes, {raw.__class__.__name__} passed.")
        if not (length := len(raw)) == 2:
            raise ValueError(f"Arg raw should be of length 2 (bytes), {length} instead.")

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
        As to Core Specification V5.4, this field has a meaning only in response to a LE_Test_Setup command with control
        parameters `0x05` and `0x09`.
        Raise ReservedForFutureUseError for other control parameters code, or when status is Error, as the response field is
        Reserved for future use.
        """
        if self.status == Status.ERROR:
            raise ReservedForFutureUseError("Nothing to decode in Error status, Reserved for future use.")

        if ctrl == 0x05:
            return ...

        if ctrl == 0x09:
            return ...

        raise ReservedForFutureUseError(f"Invalid control parameter code: {hex(ctrl)}." "Please check function documentation.")


class LE_Packet_Report(DUTResponse):
    _packet_count: int

    def __init__(self, response: int) -> None:
        self._event = Event.LE_PACKET_REPORT
        self._packet_count = response & 0x7FFF

    @property
    def packet_count(self) -> int:
        """Returns the number of packets received by the DUT (buffer of 15 bits, so up to 32767).
        Note: The DUT is not responsible for any overflow conditions of the packet count. That responsibility belongs with the
        RFPHY Tester or other auxiliary equipment.
        """
        return self._packet_count

    def __repr__(self) -> str:
        return f"LE_Packet_Report({self.packet_count} packet(s) received)"


class ReservedForFutureUseError(Exception):
    pass
