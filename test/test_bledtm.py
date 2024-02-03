import pytest
import serial
from typing import cast
from bledtm import Event, DUTResponse, Status, LE_Test_Status, DirectTestMode


@pytest.mark.parametrize(
    "test_input, expected",
    [
        (0x0000, Event.LE_TEST_STATUS),
        (0x0001, Event.LE_TEST_STATUS),
        (0x7FFF, Event.LE_TEST_STATUS),
        (0x8000, Event.LE_PACKET_REPORT),
        (0x8001, Event.LE_PACKET_REPORT),
        (0xFFFF, Event.LE_PACKET_REPORT),
    ],
)
def test_event_type(test_input, expected):
    assert DUTResponse.from_raw(test_input.to_bytes(2)).event is expected


@pytest.mark.parametrize(
    "test_input, expected",
    [(0x0000, Status.SUCCESS), (0x7FFE, Status.SUCCESS), (0x0001, Status.ERROR), (0x7FFF, Status.ERROR)],
)
def test_status(test_input, expected):
    assert cast(LE_Test_Status, DUTResponse.from_raw(test_input.to_bytes(2))).status is expected


def test_factory():
    with pytest.raises(TypeError):
        DirectTestMode()

    with pytest.raises(TypeError):
        DirectTestMode(ser=serial.Serial(), port="/dev/null")
