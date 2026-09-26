"""使用真实 MotorControl、内存串口和虚拟时钟验证通信回归。"""

import struct
import unittest
from unittest.mock import patch

from motor import DM_Motor_Type, DM_variable, Motor, MotorControl


def frame(can_id, payload, cmd=0x11):
    return bytes([0xAA, cmd, 8]) + can_id.to_bytes(4, "little") + bytes(payload) + b"\x55"


class MemorySerial:
    def __init__(self):
        self.is_open = False
        self.received = b""
        self.written = []

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def write(self, data):
        self.written.append(bytes(data))
        return len(data)

    def read_all(self):
        data, self.received = self.received, b""
        return data


class ReplyingSerial(MemorySerial):
    """仅在收到命令时应答，保留 SDK 的真实读写顺序与等待时间。"""

    def __init__(self, ids):
        super().__init__()
        self.states = dict.fromkeys(ids, 0)
        self.reply_queries = True
        self.reply_disables = True
        self.reply_controls = True
        self.enable_replies = True
        self.enable_state = 1
        self.query_limit = None
        self.query_fault_after = None
        self.queries = 0
        self.controls = []
        self.saves = []
        self.param_values = {
            (sid, rid): value
            for sid in self.states
            for rid, value in (
                (DM_variable.CTRL_MODE, 1),
                (DM_variable.Damp, 1.0),
                (DM_variable.Gr, 1.0),
                (DM_variable.PMAX, 12.5),
                (DM_variable.VMAX, 45.0),
                (DM_variable.TMAX, 20.0),
            )
        }

    def status(self, sid):
        payload = [(self.states[sid] << 4) | sid, 0x80, 0, 0x7F, 0xF7, 0xFF, 25, 25]
        self.received += frame(0x20 + sid, payload)

    def write(self, data):
        super().write(data)
        can_id = int.from_bytes(data[13:15], "little")
        payload = data[21:29]
        if can_id == 0x7FF:
            sid = int.from_bytes(payload[:2], "little")
            command, rid = payload[2:4]
            if command == 0x33:
                value = self.param_values.get((sid, rid))
                if value is not None:
                    encoding = "<I" if rid == DM_variable.CTRL_MODE else "<f"
                    self.received += frame(0x20 + sid, payload[:4] + struct.pack(encoding, value))
            elif command == 0x55:
                encoding = "<I" if rid == DM_variable.CTRL_MODE else "<f"
                self.param_values[sid, rid] = struct.unpack(encoding, payload[4:8])[0]
                self.received += frame(0x20 + sid, payload)
            elif command == 0xAA:
                self.saves.append(sid)
                self.received += frame(0x20 + sid, payload)
            elif command == 0xCC:
                self.queries += 1
                if self.query_fault_after is not None and self.queries > self.query_fault_after:
                    self.states[sid] = 8
                if self.reply_queries and (self.query_limit is None or self.queries <= self.query_limit):
                    self.status(sid)
        else:
            sid = can_id & 0x0F
            if payload[:7] == b"\xFF" * 7:
                self.states[sid] = self.enable_state if payload[7] == 0xFC else 0
                if payload[7] == 0xFC and not self.enable_replies:
                    return len(data)
                if payload[7] == 0xFD and not self.reply_disables:
                    return len(data)
            else:
                self.controls.append(sid)
                if not self.reply_controls:
                    return len(data)
            self.status(sid)
        return len(data)


class TransportRegressions(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        for target, replacement in (
            ("motor.DM_CAN.monotonic", lambda: self.now),
            ("motor.DM_CAN.sleep", self.sleep),
        ):
            patcher = patch(target, replacement)
            patcher.start()
            self.addCleanup(patcher.stop)

    def sleep(self, seconds):
        self.now += seconds

    def make_sdk(self, slave_id=1, master_id=0x11):
        serial = MemorySerial()
        sdk = MotorControl(serial)
        motor = Motor(DM_Motor_Type.DM6006, slave_id, master_id)
        sdk.addMotor(motor)
        return sdk, motor, serial

    def test_enabled_and_fault_frames_survive_parameter_byte_collisions(self):
        for state in (1, 2, 8, 9, 10, 11, 12, 13, 14):
            for marker in (0x33, 0x55, 0xAA):
                with self.subTest(state=state, marker=marker):
                    sdk, motor, serial = self.make_sdk()
                    sdk._MotorControl__read_RID_param(motor, DM_variable.PMAX)
                    serial.received = frame(0x11, [(state << 4) | 1, 0, marker, 0x20, 7, 0xFF, 25, 25])
                    sdk.recv()
                    self.assertEqual(motor.getError(), state)
                    self.assertEqual(motor.feedback_count, 1)
                    self.assertEqual(sdk.dropped_param_replies, 0)

    def test_late_replies_for_all_declared_rids_never_update_feedback(self):
        for rid in DM_variable:
            for command in (0x33, 0x55):
                with self.subTest(rid=rid, command=command):
                    sdk, motor, serial = self.make_sdk()
                    motor.recv_data(1.25, 0, 0, 1)
                    timestamp = motor.last_feedback_at
                    sdk._MotorControl__read_RID_param(motor, rid)
                    self.sleep(2.0)
                    serial.received = frame(0x11, bytes([1, 0, command, rid]) + struct.pack("<f", 1.0))
                    sdk.recv()
                    self.assertEqual(motor.getPosition(), 1.25)
                    self.assertEqual(motor.feedback_count, 1)
                    self.assertEqual(motor.last_feedback_at, timestamp)
                    self.assertEqual(sdk.dropped_param_replies, 1)

    def test_full_slave_id_is_used_for_parameter_replies(self):
        sdk, motor, serial = self.make_sdk(slave_id=0x101)
        sdk._MotorControl__read_RID_param(motor, DM_variable.u_off)
        serial.received = frame(0x11, b"\x01\x01\x33\x32" + struct.pack("<f", 1.25))
        sdk.recv()
        self.assertEqual(motor.getParam(DM_variable.u_off), 1.25)

    def test_other_motors_can_id_cannot_complete_parameter_request(self):
        sdk, motor, serial = self.make_sdk()
        other = Motor(DM_Motor_Type.DM6006, 3, 0x13)
        sdk.addMotor(other)
        sdk._MotorControl__read_RID_param(motor, DM_variable.PMAX)
        payload = b"\x01\x00\x33\x15" + struct.pack("<f", 6.25)
        serial.received = frame(0x13, payload)
        sdk.recv()
        self.assertIsNone(motor.getParam(DM_variable.PMAX))
        self.assertIsNone(other.getParam(DM_variable.PMAX))
        serial.received = frame(0x11, payload)
        sdk.recv()
        self.assertEqual(motor.getParam(DM_variable.PMAX), 6.25)

    def test_read_reply_does_not_complete_write_request(self):
        sdk, motor, serial = self.make_sdk()
        sdk._MotorControl__write_motor_param(motor, DM_variable.CTRL_MODE, 3)
        serial.received = frame(0x11, b"\x01\x00\x33\x0A" + struct.pack("<I", 2))
        sdk.recv()
        self.assertIsNone(motor.getParam(DM_variable.CTRL_MODE))
        serial.received = frame(0x11, b"\x01\x00\x55\x0A" + struct.pack("<I", 3))
        sdk.recv()
        self.assertEqual(motor.getParam(DM_variable.CTRL_MODE), 3)

    def test_fragmented_parameter_and_status_frames_share_one_buffer(self):
        sdk, motor, serial = self.make_sdk()
        sdk._MotorControl__read_RID_param(motor, DM_variable.u_off)
        stream = frame(0x11, b"\x01\x00\x33\x32" + struct.pack("<f", 1.25))
        stream += frame(0x11, [0x11, 0x80, 0, 0x7F, 0xF7, 0xFF, 25, 25])
        for byte in stream:
            serial.received = bytes([byte])
            sdk.recv_set_param_data()
        self.assertEqual(motor.getParam(DM_variable.u_off), 1.25)
        self.assertEqual(motor.feedback_count, 1)
        self.assertEqual(sdk.data_save, b"")

    def test_noise_buffer_is_bounded_and_preserves_a_split_frame(self):
        sdk, motor, serial = self.make_sdk()
        for _ in range(10):
            serial.received = bytes(1024)
            sdk.recv()
            self.assertLess(len(sdk.data_save), 16)
        packet = frame(0x11, [0x11, 0x80, 0, 0x7F, 0xF7, 0xFF, 25, 25])
        serial.received = bytes(1024) + packet[:7]
        sdk.recv()
        serial.received = packet[7:]
        sdk.recv()
        self.assertEqual(motor.feedback_count, 1)
        self.assertEqual(sdk.data_save, b"")

    def test_ambiguous_disabled_frame_does_not_refresh_state(self):
        sdk, motor, serial = self.make_sdk()
        self.now = 2.0
        serial.received = frame(0x11, [1, 0, 0xAA, 0x7F, 0xF8, 0, 25, 25])
        sdk.recv()
        self.assertEqual(motor.feedback_count, 0)
        self.assertIsNone(motor.last_feedback_at)

    def test_hardware_enable_reply_is_not_dropped(self):
        """实机使能帧的 data[2] 是 0xAA，但它不是参数应答。"""
        sdk, motor, serial = self.make_sdk()
        serial.received = frame(0x11, [0x11, 0x71, 0xAA, 0x7F, 0xF8, 0, 0x21, 0x1E])
        sdk.recv()
        self.assertEqual(motor.getError(), 0x01)
        self.assertEqual(motor.feedback_count, 1)
        self.assertEqual(sdk.dropped_param_replies, 0)
        self.assertAlmostEqual(motor.getPosition(), -1.3998, delta=0.001)

    def test_adapter_frames_are_counted(self):
        sdk, motor, serial = self.make_sdk()
        serial.received = frame(0, bytes(8), cmd=0xEE)
        sdk.recv()
        self.assertEqual(sdk.ignored_frames, 1)
        self.assertEqual(motor.feedback_count, 0)

    def test_read_motor_param_accepts_matching_reply(self):
        sdk, motor, serial = self.make_sdk()
        reply = frame(0x11, [1, 0, 0x33, 10, 1, 0, 0, 0])
        with patch.object(serial, "read_all", side_effect=[b"", reply]):
            value = sdk.read_motor_param(motor, DM_variable.CTRL_MODE)
        self.assertEqual(value, 1)
        self.assertEqual(motor.getParam(DM_variable.CTRL_MODE), 1)
        self.assertEqual(motor.feedback_count, 0)
        self.assertEqual(sdk.dropped_param_replies, 0)

    def test_read_motor_param_keeps_retrying_until_reply(self):
        sdk, motor, serial = self.make_sdk()
        reply = frame(0x11, [1, 0, 0x33, 10, 2, 0, 0, 0])
        with patch.object(serial, "read_all", side_effect=[b"", b"", reply]):
            value = sdk.read_motor_param(motor, DM_variable.CTRL_MODE)
        self.assertEqual(value, 2)

    def test_save_reply_is_swallowed_without_parsing(self):
        sdk, motor, serial = self.make_sdk()
        motor.state_q = 1.25
        sdk.save_motor_param(motor)
        serial.received = frame(0x11, [1, 0, 0xAA, 1, 0, 0, 0, 0])
        sdk.recv()
        self.assertIsNone(sdk._param_request)
        self.assertEqual(sdk.dropped_param_replies, 0)
        self.assertEqual(motor.temp_param_dict, {})
        self.assertEqual(motor.getPosition(), 1.25)
        self.assertEqual(motor.feedback_count, 0)

    def test_edge_stationary_feedback_is_not_dropped(self):
        sdk, motor, serial = self.make_sdk()
        serial.received = frame(0x11, [0x11, 0, 0xAA, 0x7F, 0xF8, 0, 0x21, 0x1E])
        sdk.recv()
        self.assertEqual(motor.feedback_count, 1)
        self.assertEqual(sdk.dropped_param_replies, 0)
        self.assertEqual(motor.getError(), 0x01)
        self.assertAlmostEqual(motor.getPosition(), -12.44, delta=0.05)

    def test_feedback_is_still_parsed_during_param_window(self):
        sdk, motor, serial = self.make_sdk()
        sdk.save_motor_param(motor)
        serial.received = frame(0x11, [0x11, 0x80, 0, 0x80, 8, 0, 25, 25])
        sdk.recv()
        self.assertEqual(motor.getError(), 0x01)
        self.assertIsNotNone(sdk._param_request)
        self.assertEqual(sdk.dropped_param_replies, 0)


if __name__ == "__main__":
    unittest.main()
