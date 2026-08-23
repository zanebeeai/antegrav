import unittest
from unittest.mock import Mock, patch

from drive import pedal


class PedalReconnectTests(unittest.TestCase):
    def test_retries_initial_open_failure(self):
        serial_link = Mock()
        serial_link.read.return_value = b"0.500\n"
        logger = Mock()

        with patch.object(pedal.time, "monotonic", side_effect=[10.0, 10.5, 11.0, 11.0]), \
                patch.object(pedal.time, "monotonic_ns", return_value=123), \
                patch.object(
                    pedal.serial, "Serial",
                    side_effect=[pedal.serial.SerialException("missing"), serial_link],
                ) as open_port:
            link = pedal.PedalLink(logger)
            link.pump()
            self.assertIsNone(link._ser)
            link.pump()

        self.assertEqual(open_port.call_count, 2)
        self.assertEqual(link.frac, 0.5)
        self.assertEqual(link.sample_timestamp_ns, 123)

    def test_read_failure_is_inert_then_reconnects(self):
        failed_link = Mock()
        failed_link.read.side_effect = pedal.serial.SerialException("unplugged")
        recovered_link = Mock()
        recovered_link.read.return_value = b"0.750\n"
        logger = Mock()

        with patch.object(
                pedal.time, "monotonic",
                side_effect=[20.0, 20.5, 21.0, 21.0]), \
                patch.object(pedal.time, "monotonic_ns", return_value=456), \
                patch.object(
                    pedal.serial, "Serial",
                    side_effect=[failed_link, recovered_link],
                ):
            link = pedal.PedalLink(logger)
            link.pump()
            self.assertIsNone(link._ser)
            self.assertEqual(link.frac, 0.0)
            link.pump()
            self.assertIsNone(link._ser)
            link.pump()

        failed_link.close.assert_called_once_with()
        self.assertIs(link._ser, recovered_link)
        self.assertEqual(link.frac, 0.75)
        self.assertEqual(link.sample_timestamp_ns, 456)


if __name__ == "__main__":
    unittest.main()
