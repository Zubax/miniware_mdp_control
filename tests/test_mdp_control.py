import unittest
from dataclasses import replace
from io import StringIO
from unittest.mock import patch

import mdp_control
from mdp_control import (
    ChannelCommand,
    ChannelStatus,
    MDPCommandTimeoutError,
    MDPConnectionError,
    MDPController,
    MDPDependencyError,
    MDPDeviceNotFoundError,
    MDPDeviceStateError,
    MDPError,
    MDPProtocolError,
    MDPValidationError,
    Status,
)
from mdp_control import (
    _Packet,
    _PacketParser,
    _PacketType,
    _build_packet,
    _main,
    _make_argument_parser,
    _parse_cli_tokens,
    _parse_status,
)


def _channel_status(channel, *, online=True, output=False, voltage=24.0, current=1.0):
    return ChannelStatus(
        channel=channel,
        number=channel - 1,
        machine_type=2 if online else 0,
        machine="P906" if online else "unconfigured",
        online=online,
        output_enabled=output,
        mode="CV" if output else "OFF",
        locked=False,
        error=False,
        voltage=voltage if output else 0.0,
        current=0.0,
        power=0.0,
        set_voltage=voltage,
        current_limit=current,
        input_voltage=20.0,
        input_current=0.2,
        temperature_c=28.0,
    )


def _status(*channels):
    values = list(channels)
    while len(values) < 6:
        values.append(_channel_status(len(values) + 1, online=False, voltage=0, current=0))
    return Status(selected_channel=1, channels=tuple(values))


class _FakeController(MDPController):
    def __init__(self, before, after):
        super().__init__(port="fake")
        self.before = before
        self.after = after
        self.sent = []

    def read_status(self, timeout=None, predicate=None):
        status = self.before if predicate is None else self.after
        if predicate is not None and not predicate(status):
            raise MDPCommandTimeoutError("fake state did not match")
        return status

    def _send(self, packet_type, channel=0xEE, payload=b""):
        self.sent.append((packet_type, channel, payload))


class ErrorHierarchyTests(unittest.TestCase):
    def test_all_public_errors_share_one_base(self):
        error_types = (
            MDPCommandTimeoutError,
            MDPConnectionError,
            MDPDependencyError,
            MDPDeviceNotFoundError,
            MDPDeviceStateError,
            MDPProtocolError,
            MDPValidationError,
        )
        for error_type in error_types:
            with self.subTest(error_type=error_type):
                self.assertTrue(issubclass(error_type, MDPError))

    def test_public_surface_is_explicit(self):
        expected = {
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
        }
        self.assertEqual(set(mdp_control.__all__), expected)
        self.assertEqual(
            {name for name in vars(mdp_control) if not name.startswith("_")},
            expected,
        )

    def test_invalid_timeout_uses_public_validation_error(self):
        with self.assertRaises(MDPValidationError):
            MDPController(port="fake", timeout="3")


class PacketTests(unittest.TestCase):
    def test_build_packet(self):
        packet = _build_packet(_PacketType.SET_OUTPUT, channel=2, payload=b"\x01")
        self.assertEqual(packet, bytes.fromhex("5a5a1607020101"))

    def test_parser_handles_fragmentation_and_noise(self):
        expected = _build_packet(_PacketType.HEARTBEAT)
        parser = _PacketParser()
        self.assertEqual(parser.feed(b"noise\x5a"), [])
        self.assertEqual(parser.feed(expected[1:4]), [])
        packets = parser.feed(expected[4:])
        self.assertEqual(
            packets,
            [_Packet(type=_PacketType.HEARTBEAT, channel=0xEE, payload=b"")],
        )

    def test_parser_resynchronizes_after_bad_checksum(self):
        bad = bytearray(_build_packet(_PacketType.SET_OUTPUT, 0, b"\x01"))
        bad[5] ^= 1
        good = _build_packet(_PacketType.SET_OUTPUT, 0, b"\x00")
        packets = _PacketParser().feed(bytes(bad) + good)
        self.assertEqual(packets, [_Packet(_PacketType.SET_OUTPUT, 0, b"\x00")])


class StatusTests(unittest.TestCase):
    def test_parse_current_firmware_status(self):
        records = []
        for channel in range(6):
            record = bytearray(24)
            record[0] = channel
            if channel == 0:
                record[1:3] = (5001).to_bytes(2, "little")
                record[3:5] = (250).to_bytes(2, "little")
                record[5:7] = (24000).to_bytes(2, "little")
                record[7:9] = (300).to_bytes(2, "little")
                record[9:11] = (5000).to_bytes(2, "little")
                record[11:13] = (1000).to_bytes(2, "little")
                record[13:15] = (285).to_bytes(2, "little")
                record[15] = 1
                record[16] = 2
                record[18] = 2
                record[19] = 1
            records.append(record)

        packet = _Packet(_PacketType.SYNTHESIZE, 0, b"".join(records))
        status = _parse_status(packet)
        channel = status.channels[0]
        self.assertEqual(status.selected_channel, 1)
        self.assertEqual(channel.machine, "P906")
        self.assertTrue(channel.online)
        self.assertTrue(channel.output_enabled)
        self.assertEqual(channel.mode, "CV")
        self.assertAlmostEqual(channel.voltage, 5.001)
        self.assertAlmostEqual(channel.current, 0.250)
        self.assertAlmostEqual(channel.power, 1.25025)
        self.assertAlmostEqual(channel.set_voltage, 5.0)
        self.assertAlmostEqual(channel.current_limit, 1.0)
        self.assertAlmostEqual(channel.temperature_c, 28.5)
        self.assertFalse(status.channels[1].online)

    def test_status_requires_six_records(self):
        with self.assertRaisesRegex(MDPProtocolError, "invalid size"):
            _parse_status(_Packet(_PacketType.SYNTHESIZE, 0, b"\x00" * 145))


class CommandLineParsingTests(unittest.TestCase):
    def test_console_entry_point_formats_library_errors(self):
        stderr = StringIO()
        with patch("mdp_control._run", side_effect=MDPConnectionError("broken connection")):
            with patch("mdp_control._sys.stderr", stderr):
                self.assertEqual(_main(["status"]), 1)
        self.assertEqual(stderr.getvalue(), "error: broken connection\n")

    def test_help_uses_module_documentation_and_contains_examples(self):
        parser = _make_argument_parser()
        help_text = parser.format_help()
        self.assertIs(parser.description, mdp_control.__doc__)
        self.assertIn(mdp_control.__doc__.strip(), help_text)
        self.assertIn("mdp-control ch1 9V 0.75A on", help_text)
        self.assertIn(
            "mdp-control ch1 9V 750mA on ch2 5V 1A on ch3 off",
            help_text,
        )

    def test_channel_clauses_are_order_independent_and_case_insensitive(self):
        commands = _parse_cli_tokens(["CH1", "750mA", "9v", "ON", "channel2", "Off", "ch3", "0.5A"])
        self.assertEqual(
            commands,
            (
                ChannelCommand(1, voltage=9.0, current=0.75, output=True),
                ChannelCommand(2, output=False),
                ChannelCommand(3, current=0.5),
            ),
        )

    def test_on_and_off_reuse_existing_limits(self):
        self.assertEqual(_parse_cli_tokens(["ch1", "on"]), (ChannelCommand(1, output=True),))
        self.assertEqual(
            _parse_cli_tokens(["ch1", "off"]),
            (ChannelCommand(1, output=False),),
        )

    def test_duplicate_channel_is_rejected_before_execution(self):
        with self.assertRaisesRegex(MDPValidationError, "appears more than once"):
            _parse_cli_tokens(["ch1", "on", "CH1", "off"])

    def test_duplicate_quantity_is_rejected(self):
        with self.assertRaisesRegex(MDPValidationError, "more than one voltage"):
            _parse_cli_tokens(["ch1", "5V", "9V"])

    def test_legacy_syntax_remains_supported(self):
        self.assertEqual(
            _parse_cli_tokens(["set", "9", "0.75"], legacy_channel=2),
            (ChannelCommand(2, voltage=9.0, current=0.75),),
        )


class BatchApplicationTests(unittest.TestCase):
    def test_batch_disables_then_configures_then_enables(self):
        before_ch1 = _channel_status(1, output=False, voltage=24.0, current=1.0)
        before_ch2 = _channel_status(2, output=True, voltage=5.0, current=0.5)
        before = _status(before_ch1, before_ch2)
        after = _status(
            replace(before_ch1, output_enabled=True, set_voltage=9.0, current_limit=0.75),
            replace(before_ch2, output_enabled=False, set_voltage=3.3),
        )
        controller = _FakeController(before, after)

        with patch("mdp_control._time.sleep"):
            result = controller.apply(
                (
                    ChannelCommand(1, voltage=9.0, current=0.75, output=True),
                    ChannelCommand(2, voltage=3.3, output=False),
                )
            )

        self.assertIs(result, after)
        self.assertEqual(controller.sent[0], (_PacketType.SET_OUTPUT, 1, b"\x00"))
        self.assertEqual(
            [(packet_type, channel) for packet_type, channel, _ in controller.sent[1:-1]],
            [
                (_PacketType.SET_VOLTAGE, 0),
                (_PacketType.SET_VOLTAGE, 0),
                (_PacketType.SET_CURRENT, 0),
                (_PacketType.SET_CURRENT, 0),
                (_PacketType.SET_VOLTAGE, 1),
                (_PacketType.SET_VOLTAGE, 1),
            ],
        )
        self.assertEqual(controller.sent[-1], (_PacketType.SET_OUTPUT, 0, b"\x01"))

    def test_entire_batch_is_validated_before_first_write(self):
        before = _status(_channel_status(1))
        controller = _FakeController(before, before)
        with self.assertRaisesRegex(MDPDeviceStateError, "channel 3 is offline"):
            controller.apply((ChannelCommand(1, output=True), ChannelCommand(3, output=True)))
        self.assertEqual(controller.sent, [])

    def test_invalid_batch_container_uses_public_validation_error(self):
        before = _status(_channel_status(1))
        controller = _FakeController(before, before)
        with self.assertRaisesRegex(MDPValidationError, "iterable"):
            controller.apply(42)
        self.assertEqual(controller.sent, [])

    def test_invalid_channel_type_uses_public_validation_error(self):
        before = _status(_channel_status(1))
        controller = _FakeController(before, before)
        with self.assertRaisesRegex(MDPValidationError, "integer"):
            controller.apply(ChannelCommand([], output=True))
        self.assertEqual(controller.sent, [])


if __name__ == "__main__":
    unittest.main()
