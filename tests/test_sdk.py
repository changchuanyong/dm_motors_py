"""直接 SDK 调用和示例退出路径的无硬件回归测试。"""

import io
import runpy
import struct
import unittest
from contextlib import redirect_stdout, redirect_stderr
from pathlib import Path
from unittest.mock import patch

from serial import SerialTimeoutException

from motor import Control_Type, DM_Motor_Type, DM_variable, Motor, MotorControl
from test_transport_regressions import MemorySerial, ReplyingSerial, frame


EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "DM_Motor_Example.py"


class SDKTests(unittest.TestCase):
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

    def make_sdk(self, mode=Control_Type.VEL, count=1, ready=True):
        port = ReplyingSerial(range(count))
        sdk = MotorControl(port)
        self.addCleanup(port.close)
        for sid in port.states:
            motor = Motor(DM_Motor_Type.DM6006, sid, 0x20 + sid)
            sdk.addMotor(motor)
            if ready:
                sdk.sync_motor_limits(motor)
                self.assertTrue(sdk.switchControlMode(motor, mode))
                sdk.enable(motor)
        if ready:
            sdk.refresh_status()
        return sdk, sdk.motors[0], port

    def test_id_collisions_and_foreign_objects_are_rejected(self):
        sdk, motor, port = self.make_sdk()
        for sid, mid in ((0, 0x22), (2, 0x20), (0x20, 0x22)):
            with self.subTest(slave=sid, master=mid), self.assertRaises(ValueError):
                sdk.addMotor(Motor(DM_Motor_Type.DM6006, sid, mid))
        before = len(port.written)
        with self.assertRaises(ValueError):
            sdk.control_Vel(Motor(DM_Motor_Type.DM6006, 0, 0x20), 0.1)
        self.assertEqual(len(port.written), before)
        self.assertEqual(list(sdk.motors), [motor])

    def test_measured_limits_are_used_without_changing_other_controllers(self):
        sdk, motor, port = self.make_sdk(ready=False)
        port.param_values[(0, DM_variable.PMAX)] = 6.25
        port.param_values[(0, DM_variable.TMAX)] = 12.0
        sdk.sync_motor_limits(motor)
        self.assertEqual(tuple(motor.limits), (6.25, 45.0, 12.0))
        self.assertEqual(tuple(sdk.Limit_Param[DM_Motor_Type.DM6006]), motor.limits)
        other = MotorControl(MemorySerial())
        self.addCleanup(other.serial_.close)
        self.assertEqual(other.Limit_Param[DM_Motor_Type.DM6006], [12.5, 45, 20])
        sdk.switchControlMode(motor, Control_Type.POS_VEL)
        sdk.enable(motor)
        sdk.control_Pos_Vel(motor, 6.25, 0.1)
        before = len(port.written)
        with self.assertRaises(ValueError):
            sdk.control_Pos_Vel(motor, 6.251, 0.1)
        self.assertEqual(len(port.written), before)

    def test_unreadable_or_conflicting_limits_are_rejected(self):
        sdk, first, port = self.make_sdk(count=2, ready=False)
        port.param_values[(0, DM_variable.TMAX)] = None
        with self.assertRaises(RuntimeError):
            sdk.sync_motor_limits(first)
        self.assertIsNone(first.limits)
        port.param_values[(0, DM_variable.TMAX)] = 20.0
        sdk.sync_motor_limits(first)
        second = sdk.motors[1]
        port.param_values[(1, DM_variable.PMAX)] = 6.25
        with self.assertRaises(RuntimeError):
            sdk.sync_motor_limits(second)
        self.assertEqual(tuple(sdk.Limit_Param[DM_Motor_Type.DM6006]), first.limits)

    def test_limit_changes_invalidate_previously_measured_limits(self):
        for rid, value in (
            (DM_variable.PMAX, 6.25),
            (DM_variable.VMAX, 30.0),
            (DM_variable.TMAX, 10.0),
        ):
            for replies in (True, False):
                with self.subTest(rid=rid, replies=replies):
                    sdk, motor, port = self.make_sdk()
                    if not replies:
                        port.write = lambda data: MemorySerial.write(port, data)
                    self.assertEqual(sdk.change_motor_param(motor, rid, value), replies)
                    self.assertIsNone(motor.limits)
                    before = len(port.written)
                    with self.assertRaises(RuntimeError):
                        sdk.control_Vel(motor, 0.1)
                    self.assertEqual(len(port.written), before)
                    if replies:
                        sdk.sync_motor_limits(motor)
                        sdk.refresh_status()
                        sdk.control_Vel(motor, 0.1)
        sdk, motor, _ = self.make_sdk(count=2)
        measured = motor.limits
        sdk.change_limit_param(motor.MotorType, *measured)
        self.assertTrue(all(item.limits == measured for item in sdk.motors))
        sdk.change_limit_param(motor.MotorType, 6.25, 45, 20)
        self.assertTrue(all(item.limits is None for item in sdk.motors))

    def test_four_modes_encode_updated_numeric_targets(self):
        cases = (
            (
                Control_Type.MIT,
                "controlMIT",
                0,
                ((500, 0, 0, 100, -100), (0, 5, 12.5, -100, 100)),
                (
                    b"\x7F\xFF\xFF\xFF\xFF\x00\x00\x00",
                    b"\xFF\xFF\x00\x00\x00\xFF\xFF\xFF",
                ),
            ),
            (
                Control_Type.POS_VEL,
                "control_Pos_Vel",
                0x100,
                ((0.25, 0.5), (-0.5, 1.25)),
                (struct.pack("<ff", 0.25, 0.5), struct.pack("<ff", -0.5, 1.25)),
            ),
            (
                Control_Type.VEL,
                "control_Vel",
                0x200,
                ((0.25,), (-0.5,)),
                (struct.pack("<fI", 0.25, 0), struct.pack("<fI", -0.5, 0)),
            ),
            (
                Control_Type.Torque_Pos,
                "control_pos_force",
                0x300,
                ((0.25, 100, 3000), (-0.5, 65535, 10000)),
                (
                    struct.pack("<fHH", 0.25, 100, 3000),
                    struct.pack("<fHH", -0.5, 65535, 10000),
                ),
            ),
        )
        for mode, method, can_id, commands, payloads in cases:
            with self.subTest(mode=mode):
                sdk, motor, port = self.make_sdk(mode)
                for command, payload in zip(commands, payloads):
                    getattr(sdk, method)(motor, *command)
                    self.assertEqual(
                        int.from_bytes(port.written[-1][13:15], "little"), can_id
                    )
                    self.assertEqual(port.written[-1][21:29], payload)
                self.assertEqual(port.controls, [0, 0])

    def test_invalid_targets_never_reach_the_wire(self):
        cases = (
            (
                Control_Type.MIT,
                "controlMIT",
                (
                    (-1, 0, 0, 0, 0),
                    (501, 0, 0, 0, 0),
                    (0, -1, 0, 0, 0),
                    (0, 6, 0, 0, 0),
                    (0, 0, 13, 0, 0),
                    (0, 0, 0, float("nan"), 0),
                    (0, 0, 0, 0, float("inf")),
                ),
            ),
            (
                Control_Type.POS_VEL,
                "control_Pos_Vel",
                ((13, 0), (None, 0), (0, float("nan"))),
            ),
            (Control_Type.VEL, "control_Vel", ((float("inf"),),)),
            (
                Control_Type.Torque_Pos,
                "control_pos_force",
                (
                    (13, 0, 0),
                    (0, -1, 0),
                    (0, 65536, 0),
                    (0, 1.0, 0),
                    (0, True, 0),
                    (0, 0, -1),
                    (0, 0, 10001),
                ),
            ),
        )
        for mode, method, commands in cases:
            sdk, motor, port = self.make_sdk(mode)
            for command in commands:
                with self.subTest(mode=mode, command=command):
                    before = len(port.written)
                    with self.assertRaises((ValueError, TypeError)):
                        getattr(sdk, method)(motor, *command)
                    self.assertEqual(len(port.written), before)

    def test_float32_overflow_is_rejected_before_control_writes(self):
        cases = (
            (Control_Type.POS_VEL, "control_Pos_Vel", (0.0,), 25),
            (Control_Type.VEL, "control_Vel", (), 21),
            (Control_Type.POS_VEL_CSP, "control_Pos_Vel_CSP", (0.0,), 25),
            (Control_Type.VEL_CSP, "control_Vel_CSP", (), 21),
            (Control_Type.Torque_CSP, "control_Tor_CSP", (), 21),
        )
        maximum = struct.unpack("<f", b"\xFF\xFF\x7F\x7F")[0]
        for mode, method, prefix, offset in cases:
            sdk, motor, port = self.make_sdk(mode)
            for value in (1e40, -1e40):
                with self.subTest(mode=mode, invalid=value):
                    before = len(port.written)
                    with self.assertRaises(ValueError):
                        getattr(sdk, method)(motor, *prefix, value)
                    self.assertEqual(len(port.written), before)
            for value in (maximum, -maximum):
                with self.subTest(mode=mode, valid=value):
                    getattr(sdk, method)(motor, *prefix, value)
                    encoded = port.written[-1][offset : offset + 4]
                    self.assertEqual(struct.unpack("<f", encoded)[0], value)

    def test_invalid_parameter_values_are_rejected_before_writing(self):
        sdk, motor, port = self.make_sdk(ready=False)
        cases = (
            (
                DM_variable.Damp,
                (float("nan"), float("inf"), -float("inf"), 1e40, -1e40),
            ),
            (DM_variable.PMAX, (-1.0, 0.0, 1e-50)),
            (DM_variable.VMAX, (-1.0, 0.0, 1e-50)),
            (DM_variable.TMAX, (-1.0, 0.0, 1e-50)),
            (DM_variable.TIMEOUT, (1.9, -1, 0x100000000)),
        )
        for rid, values in cases:
            for value in values:
                with self.subTest(rid=rid, value=value):
                    before = len(port.written)
                    with self.assertRaises((ValueError, TypeError)):
                        sdk.change_motor_param(motor, rid, value)
                    self.assertEqual(len(port.written), before)
        for rid, value, encoding in (
            (DM_variable.Damp, 1.5, "<f"),
            (DM_variable.PMAX, 6.25, "<f"),
            (DM_variable.TIMEOUT, 100, "<I"),
        ):
            with self.subTest(valid_rid=rid):
                self.assertTrue(sdk.change_motor_param(motor, rid, value))
                self.assertEqual(
                    port.written[-1][25:29], struct.pack(encoding, value)
                )

    def test_serial_write_timeout_must_be_finite_and_positive(self):
        for value in (None, 0, -0.1, float("nan"), float("inf")):
            with self.subTest(write_timeout=value):
                port = MemorySerial()
                port.write_timeout = value
                with self.assertRaises(ValueError):
                    MotorControl(port)
                self.assertEqual(port.written, [])

    def test_short_writes_and_serial_timeouts_abort_without_retry(self):
        for failure in ("zero", "partial", "timeout"):
            with self.subTest(failure=failure):
                sdk, motor, port = self.make_sdk()
                error = SerialTimeoutException("write timed out")

                def fail_write(data):
                    MemorySerial.write(port, data)
                    if failure == "timeout":
                        raise error
                    return 0 if failure == "zero" else len(data) - 1

                port.write = fail_write
                before = len(port.written)
                if failure == "timeout":
                    with self.assertRaises(SerialTimeoutException) as caught:
                        sdk.control_Vel(motor, 0.1)
                    self.assertIs(caught.exception, error)
                else:
                    with self.assertRaises(OSError):
                        sdk.control_Vel(motor, 0.1)
                self.assertEqual(len(port.written), before + 1)

    def test_commands_require_measured_limits_and_confirmed_mode(self):
        sdk, motor, port = self.make_sdk(ready=False)
        sdk.enable(motor)
        with self.assertRaises(RuntimeError):
            sdk.control_Vel(motor, 0.1)
        sdk.switchControlMode(motor, Control_Type.VEL)
        with self.assertRaises(RuntimeError):
            sdk.control_Vel(motor, 0.1)
        sdk.sync_motor_limits(motor)
        sdk.switchControlMode(motor, Control_Type.POS_VEL)
        sdk.refresh_status()
        with self.assertRaises(RuntimeError):
            sdk.control_Vel(motor, 0.1)
        self.assertEqual(port.controls, [])

    def test_missing_unknown_disabled_fault_and_stale_feedback_stop_commands(self):
        for state in (None, 0, 2, 8, 9, 10, 11, 12, 13, 14, "stale"):
            with self.subTest(state=state):
                sdk, motor, port = self.make_sdk()
                motor.recv_data(0, 0, 0, 1 if state in (None, "stale") else state)
                if state is None:
                    motor.last_feedback_at = None
                elif state == "stale":
                    self.sleep(0.201)
                before = len(port.written)
                with self.assertRaises(RuntimeError):
                    sdk.control_Vel(motor, 0.1)
                self.assertEqual(len(port.written), before)
        # 已到达串口的新反馈必须先收取，不能仅凭旧缓存时间误停。
        sdk, motor, port = self.make_sdk()
        self.sleep(0.201)
        port.status(0)
        sdk.control_Vel(motor, 0.1)
        self.assertEqual(port.controls, [0])

    def test_parameter_operations_do_not_succeed_from_old_cached_values(self):
        port = MemorySerial()
        sdk = MotorControl(port)
        self.addCleanup(port.close)
        motor = Motor(DM_Motor_Type.DM6006, 0, 0x20)
        sdk.addMotor(motor)
        motor.temp_param_dict[DM_variable.PMAX] = 12.5
        port.received = frame(0x20, b"\x00\x00\x33\x15" + struct.pack("<f", 12.5))
        self.assertIsNone(sdk.read_motor_param(motor, DM_variable.PMAX))
        motor.temp_param_dict[DM_variable.CTRL_MODE] = Control_Type.VEL
        self.assertFalse(sdk.switchControlMode(motor, Control_Type.VEL))
        self.assertIsNone(motor.NowControlMode)
        motor.temp_param_dict[DM_variable.Damp] = 1.0
        self.assertFalse(sdk.change_motor_param(motor, DM_variable.Damp, 1.0))

    def test_enable_requires_fresh_query_and_fresh_enabled_reply(self):
        for failure in ("no_query", "no_enable_reply", 0, 2, 8):
            with self.subTest(failure=failure):
                sdk, motor, port = self.make_sdk()
                sdk.disable(motor)  # 旧失能应答仍留在接收缓存。
                if failure == "no_query":
                    port.reply_queries = False
                elif failure == "no_enable_reply":
                    port.enable_replies = False
                else:
                    port.enable_state = failure
                with self.assertRaises(RuntimeError):
                    sdk.enable(motor)
                self.assertEqual(port.states[0], 0)
                self.assertEqual(port.controls, [])

    def test_legacy_reenable_failure_disables_instead_of_reusing_old_state(self):
        sdk, motor, port = self.make_sdk(Control_Type.MIT)
        sdk.enable_old(motor, Control_Type.MIT)
        port.enable_replies = False
        with self.assertRaises(RuntimeError):
            sdk.enable_old(motor, Control_Type.MIT)
        self.assertFalse(motor.isEnable)
        self.assertEqual(port.states[0], 0)
        self.assertEqual(port.controls, [])

    def test_all_motors_need_new_feedback_during_refresh(self):
        sdk, _, port = self.make_sdk(count=4)
        port.query_limit = port.queries + 3
        with self.assertRaises(RuntimeError):
            sdk.refresh_status()
        self.assertEqual(port.controls, [])

    def test_one_four_sixteen_motors_repeat_enable_control_and_disable(self):
        for count in (1, 4, 16):
            with self.subTest(count=count):
                sdk, _, port = self.make_sdk(count=count, ready=False)
                for motor in sdk.motors:
                    sdk.sync_motor_limits(motor)
                    sdk.switchControlMode(motor, Control_Type.VEL)
                for _ in range(2):
                    for motor in sdk.motors:
                        sdk.enable(motor)
                        sdk.enable(motor)
                    sdk.refresh_status()
                    sdk.check_status()
                    for motor in sdk.motors:
                        sdk.control_Vel(motor, 0.1)
                    for motor in sdk.motors:
                        sdk.disable(motor)
                self.assertEqual(port.controls, list(range(count)) * 2)
                self.assertTrue(all(state == 0 for state in port.states.values()))

    def make_example(self):
        with patch("serial.Serial") as connect:
            namespace = runpy.run_path(str(EXAMPLE), run_name="example_import")
            connect.assert_not_called()
        main = namespace["main"]
        main.__globals__.update(
            SLAVE_ID=0,
            MASTER_ID=0x20,
            RUN_SECONDS=0.025,
            CONTROL_PERIOD=0.01,
            STATUS_INTERVAL=1.0,
            TARGET_POSITION=0.1,
            TARGET_VELOCITY=0.2,
        )
        port = ReplyingSerial([0])
        for target, replacement in (
            ("serial.Serial", lambda *args, **kwargs: port),
            ("time.monotonic", lambda: self.now),
            ("time.sleep", self.sleep),
        ):
            patcher = patch(target, replacement)
            patcher.start()
            self.addCleanup(patcher.stop)
        return main, port

    def test_example_import_is_passive_and_loop_accepts_updated_targets(self):
        main, port = self.make_example()
        write = port.write

        def update_after_write(data):
            result = write(data)
            if port.controls:
                main.__globals__.update(TARGET_POSITION=-0.3, TARGET_VELOCITY=0.4)
            return result

        port.write = update_after_write
        with redirect_stdout(io.StringIO()):
            main()
        commands = [
            data[21:29]
            for data in port.written
            if int.from_bytes(data[13:15], "little") == 0x100
        ]
        self.assertGreaterEqual(len(commands), 2)
        self.assertEqual(commands[0], struct.pack("<ff", 0.1, 0.2))
        self.assertTrue(
            all(data == struct.pack("<ff", -0.3, 0.4) for data in commands[1:])
        )
        self.assertFalse(port.is_open)
        self.assertEqual(port.states[0], 0)

    def test_example_none_position_holds_initial_feedback(self):
        main, port = self.make_example()
        main.__globals__["TARGET_POSITION"] = None

        def moving_feedback(sid):
            position = 0x8000 + 100 * len(port.controls)
            port.received += frame(
                0x20 + sid,
                [
                    (port.states[sid] << 4) | sid,
                    position >> 8,
                    position & 0xFF,
                    0x7F,
                    0xF7,
                    0xFF,
                    25,
                    25,
                ],
            )

        port.status = moving_feedback
        with redirect_stdout(io.StringIO()):
            main()
        positions = [
            struct.unpack("<f", data[21:25])[0]
            for data in port.written
            if int.from_bytes(data[13:15], "little") == 0x100
        ]
        self.assertGreaterEqual(len(positions), 2)
        self.assertEqual(len(set(positions)), 1)
        self.assertAlmostEqual(positions[0], 12.5 / 65535, places=6)

    def test_example_bounds_writes_and_attempts_cleanup_after_write_failure(self):
        for failure in ("zero", "partial", "timeout"):
            with self.subTest(failure=failure):
                main, port = self.make_example()
                write = port.write
                error = SerialTimeoutException("write timed out")
                failed = False

                def fail_during_control(data):
                    nonlocal failed
                    if failed or int.from_bytes(data[13:15], "little") == 0x100:
                        failed = True
                        MemorySerial.write(port, data)
                        if failure == "timeout":
                            raise error
                        return 0 if failure == "zero" else len(data) - 1
                    return write(data)

                port.write = fail_during_control
                with patch("serial.Serial", return_value=port) as connect:
                    with redirect_stdout(io.StringIO()), redirect_stderr(
                        io.StringIO()
                    ):
                        with self.assertRaises(OSError) as caught:
                            main()
                if failure == "timeout":
                    self.assertIs(caught.exception, error)
                self.assertEqual(connect.call_args.kwargs["write_timeout"], 0.1)
                self.assertEqual(main.__globals__["SERIAL_WRITE_TIMEOUT"], 0.1)
                self.assertEqual(
                    sum(
                        int.from_bytes(data[13:15], "little") == 0x100
                        for data in port.written
                    ),
                    1,
                )
                self.assertEqual(port.written[-1][21:29], b"\xFF" * 7 + b"\xFD")
                self.assertFalse(port.is_open)

    def test_example_disables_and_closes_after_fault_timeout_bad_target_or_interrupt(
        self,
    ):
        for failure in ("fault", "timeout", "target", "interrupt"):
            with self.subTest(failure=failure):
                main, port = self.make_example()
                main.__globals__["RUN_SECONDS"] = 0.5
                write = port.write

                def fail_after_command(data):
                    result = write(data)
                    if int.from_bytes(data[13:15], "little") == 0x100:
                        if failure == "fault":
                            port.states[0] = 8
                            port.status(0)
                        elif failure == "timeout":
                            port.reply_controls = False
                        elif failure == "target":
                            main.__globals__["TARGET_POSITION"] = 13.0
                        elif failure == "interrupt":
                            raise KeyboardInterrupt
                    return result

                port.write = fail_after_command
                with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                    if failure == "interrupt":
                        main()
                    else:
                        with self.assertRaises((RuntimeError, ValueError)):
                            main()
                self.assertFalse(port.is_open)
                self.assertEqual(port.states[0], 0)


if __name__ == "__main__":
    unittest.main()
