#!/usr/bin/env python3
"""
Plain-text serial <-> ROS 2 bridge for the ESP32 (replaces micro-ROS).

Protocol (matches esp32_firmware/arduino_sketch/esp32_firmware/esp32_firmware.ino):
  ESP32 -> Pi:
    I,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>\n
    D,<loop_hz>,<sbus_ok 0|1>,<armed 0|1>,<autonomous_mode 0|1>,<estop_ok 0|1>\n
      (last two fields appended 2026-09-07 - estop_ok is 1 when the hardware
      e-stop circuit is intact/closed, 0 when tripped or the wire is cut)
    S,<ch0>,<ch1>,<ch2>,<ch3>,<ch4>,<ch5>,<leftSpeedDac>,<rightSpeedDac>,<leftReverse 0|1>,<rightReverse 0|1>\n
      (leftSpeedDac/rightSpeedDac are 0-255 DAC values, not microsecond pulses -
      the drivetrain moved to analog-voltage speed control, see CLAUDE.md)
  Pi -> ESP32:
    C,<linear>,<angular>\n
    A,<pan>,<tilt>,<fire>\n   (autonomous aim+fire - only honored by the ESP32
      when SwB=autonomous AND SwC is in an aim position; published from
      /esp32/aim_cmd, e.g. by aim_tracker.py)
"""
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__('esp32_serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.imu_pub = self.create_publisher(Imu, '/esp32/imu', 10)
        self.diag_pub = self.create_publisher(Float32MultiArray, '/esp32/diagnostics', 10)
        self.sbus_pub = self.create_publisher(Float32MultiArray, '/esp32/sbus_channels', 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, '/esp32/motor_pwm', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32MultiArray, '/esp32/aim_cmd', self.aim_cmd_callback, 10)

        # Runtime speed limiting. First powered movement of an untested
        # drivetrain should happen at a fraction of full command, and the
        # operator needs to raise that without editing firmware and reflashing.
        # These scale /cmd_vel on its way to the ESP32; they cannot make the
        # vehicle faster than the commanded value, only slower.
        self.declare_parameter('speed_scale', 0.25)   # deliberately low default
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 1.0)
        self.create_subscription(Float32MultiArray, '/tuning/drive', self.tuning_callback, 10)
        self.tuning_pub = self.create_publisher(Float32MultiArray, '/tuning/drive_state', 10)
        self.create_timer(1.0, self.publish_tuning_state)
        self.get_logger().info(
            f'speed_scale={self.get_parameter("speed_scale").value} '
            f'(cmd_vel is scaled by this before reaching the motors)'
        )

        self._port = port
        self._baud = baud
        self.ser = None
        self._ser_lock = threading.Lock()
        self._connected = False

        # Opening is retried in the background rather than blocking __init__
        # (or crashing the node) if the device isn't present yet - a systemd
        # unit that restarts this node in a tight loop every time
        # /dev/esp32 is briefly missing is a lot noisier and slower to
        # recover than a node that just keeps retrying internally while
        # staying up and reporting "disconnected" the whole time.
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def _open_serial(self) -> bool:
        """Attempt to (re)open the port. Returns True on success. Never
        raises - callers just check the return value and retry later."""
        try:
            ser = serial.Serial(self._port, self._baud, timeout=1.0)
            # The DevKitC wires RTS->EN and DTR->GPIO0 through the auto-reset
            # transistor pair. pyserial asserts BOTH lines by default on
            # open, which holds EN low and parks the ESP32 in reset - the
            # port opens cleanly and then goes completely silent, which
            # looks exactly like dead hardware. Drive them explicitly to the
            # run state instead. Verified on this vehicle: dtr=True,
            # rts=False is the combination that produces the firmware's
            # READY banner.
            ser.dtr = True
            ser.rts = False
            time.sleep(0.2)
            ser.reset_input_buffer()
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'could not open {self._port}: {e}', throttle_duration_sec=5.0)
            return False
        with self._ser_lock:
            self.ser = ser
        self._connected = True
        self.get_logger().info(f'opened {self._port} @ {self._baud}')
        return True

    def _write_line(self, line: str):
        """Best-effort write. Silently drops the line if not currently
        connected - callers (cmd_vel, aim_cmd) already fail safe on the
        ESP32 side via CMD_VEL_TIMEOUT_MS/staleness checks when nothing
        arrives, so there is nothing useful to retry here."""
        with self._ser_lock:
            ser = self.ser
        if ser is None:
            return
        try:
            ser.write(line.encode('ascii'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'write failed: {e}')

    def cmd_vel_callback(self, msg: Twist):
        scale = float(self.get_parameter('speed_scale').value)
        max_lin = float(self.get_parameter('max_linear').value)
        max_ang = float(self.get_parameter('max_angular').value)

        linear = max(-max_lin, min(max_lin, msg.linear.x * scale))
        angular = max(-max_ang, min(max_ang, msg.angular.z * scale))

        self._write_line(f'C,{linear:.4f},{angular:.4f}\n')

    def tuning_callback(self, msg: Float32MultiArray):
        """[speed_scale, max_linear, max_angular] - any value < 0 leaves that
        setting unchanged, so the dashboard can adjust one slider at a time."""
        values = list(msg.data)
        names = ('speed_scale', 'max_linear', 'max_angular')
        for name, value in zip(names, values):
            if value < 0:
                continue
            # Hard ceiling of 1.0: this control exists to limit the vehicle,
            # and a typo in a web form must never be able to amplify a command.
            self.set_parameters([rclpy.parameter.Parameter(
                name, rclpy.Parameter.Type.DOUBLE, float(min(value, 1.0))
            )])
        self.get_logger().info(
            f'tuning updated: speed_scale={self.get_parameter("speed_scale").value:.2f} '
            f'max_linear={self.get_parameter("max_linear").value:.2f} '
            f'max_angular={self.get_parameter("max_angular").value:.2f}'
        )

    def publish_tuning_state(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.get_parameter('speed_scale').value),
            float(self.get_parameter('max_linear').value),
            float(self.get_parameter('max_angular').value),
        ]
        self.tuning_pub.publish(msg)

    def aim_cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 3:
            self.get_logger().warn(f'aim_cmd expected [pan, tilt, fire], got {msg.data}')
            return
        pan, tilt, fire = msg.data
        self._write_line(f'A,{pan:.4f},{tilt:.4f},{1.0 if fire > 0.5 else 0.0:.1f}\n')

    def _handle_disconnect(self, reason: str):
        """Drop the dead handle and fall back to the reconnect loop. Never
        raises - this runs from inside an except block in read_loop."""
        self.get_logger().warning(f'{self._port} disconnected ({reason}) - will keep retrying')
        self._connected = False
        with self._ser_lock:
            ser, self.ser = self.ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def read_loop(self):
        RECONNECT_BACKOFF_S = 2.0
        while rclpy.ok():
            if not self._connected:
                if not self._open_serial():
                    time.sleep(RECONNECT_BACKOFF_S)
                    continue

            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self._handle_disconnect(str(e))
                continue
            if not raw:
                continue
            try:
                line = raw.decode('ascii', errors='replace').strip()
            except UnicodeDecodeError:
                continue
            if not line:
                continue
            try:
                self.handle_line(line)
            except Exception:
                import traceback
                self.get_logger().error('exception in handle_line:\n' + traceback.format_exc())

    def handle_line(self, line: str):
        parts = line.split(',')
        if parts[0] == 'I' and len(parts) == 11:
            try:
                qw, qx, qy, qz, gx, gy, gz, ax, ay, az = (float(p) for p in parts[1:])
            except ValueError:
                self.get_logger().warn(f'bad IMU line: {line}')
                return
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'esp32_imu'
            msg.orientation.w = qw
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            self.imu_pub.publish(msg)
        elif parts[0] == 'D' and len(parts) in (4, 6):
            # 4 parts = old firmware (loop_hz, sbus_ok, armed); 6 parts adds
            # autonomous_mode and estop_ok (2026-09-07, purely additive on
            # the ESP32 side) - accept both so a firmware/bridge version
            # mismatch degrades to "missing fields", not "diagnostics silently
            # stop publishing entirely".
            try:
                values = [float(p) for p in parts[1:]]
            except ValueError:
                self.get_logger().warn(f'bad DIAG line: {line}')
                return
            msg = Float32MultiArray()
            msg.data = values
            self.diag_pub.publish(msg)
        elif parts[0] == 'S' and len(parts) == 11:
            try:
                values = [float(p) for p in parts[1:]]
            except ValueError:
                self.get_logger().warn(f'bad SBUS/motor line: {line}')
                return
            sbus_msg = Float32MultiArray()
            sbus_msg.data = values[0:6]
            self.sbus_pub.publish(sbus_msg)
            motor_msg = Float32MultiArray()
            motor_msg.data = values[6:10]
            self.motor_pub.publish(motor_msg)
        elif parts[0] == 'READY':
            self.get_logger().info('ESP32 reported READY')
        else:
            # Anything else (boot messages, garbage) - just log it, don't crash.
            self.get_logger().debug(f'unrecognized line: {line}')


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
