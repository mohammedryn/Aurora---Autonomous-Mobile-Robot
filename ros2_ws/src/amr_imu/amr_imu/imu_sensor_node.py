"""
ISM330DHCX IMU driver node.

Reads accel + gyro via SPI on /dev/spidev0.0 (Pi 5 SPI0, CE0).
Publishes sensor_msgs/Imu on /imu/data_raw at 100 Hz.

Wiring (Pi 5 40-pin header):
  Pin 17 (3.3V)  → VCC
  Pin 20 (GND)   → GND
  Pin 19 (MOSI)  → SDA/SDI
  Pin 21 (MISO)  → SDO
  Pin 23 (SCLK)  → SCL
  Pin 24 (CE0)   → CS/CSB
"""

import struct
import time
import spidev
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Register addresses
_WHO_AM_I  = 0x0F   # expected: 0x6B
_CTRL1_XL  = 0x10   # accel: 104 Hz, ±4 g
_CTRL2_G   = 0x11   # gyro:  104 Hz, ±2000 dps
_CTRL3_C   = 0x12   # BDU, IF_INC
_OUTX_L_G  = 0x22   # first of 6 gyro bytes
_OUTX_L_A  = 0x28   # first of 6 accel bytes

# Sensitivity constants
_ACCEL_SENS = 0.000122 * 9.80665   # ±4 g, LSB → m/s²
_GYRO_SENS  = 0.070 * (3.14159265358979 / 180.0)  # ±2000 dps, LSB → rad/s

_READ_FLAG = 0x80


class ISM330DHCX:
    """Thin SPI wrapper for ISM330DHCX."""

    def __init__(self, bus: int = 0, device: int = 0):
        self._spi = spidev.SpiDev()
        self._spi.open(bus, device)
        self._spi.max_speed_hz = 100_000  # 100kHz: jumper wiring drops reads at
        # 500kHz (signal integrity) — measured 18/30 ok @500k vs 30/30 @100k.
        # IMU only needs ~100Hz data so 100kHz SPI is far more than enough.
        self._spi.mode = 0               # CPOL=0 CPHA=0 — ISM330DHCX SPI mode 0

    def init(self, retries: int = 10) -> bool:
        # WHO_AM_I can return garbage (e.g. 0x7f) on the first reads right after
        # power-up / SPI bus settling. Retry before giving up — a loose wire
        # reads 0x7f every time, but a settling glitch clears within a few reads.
        who = 0
        for attempt in range(retries):
            who = self._read_reg(_WHO_AM_I)
            if who == 0x6B:
                break
            print(f'[ISM330DHCX] WHO_AM_I returned {hex(who)} '
                  f'(attempt {attempt + 1}/{retries}), expected 0x6b')
            time.sleep(0.1)
        if who != 0x6B:
            return False
        # BDU=1, IF_INC=1 (auto-increment register address on multi-byte read)
        self._write_reg(_CTRL3_C, 0x44)
        # Accel: 104 Hz, ±4 g (CTRL1_XL = 0x4A)
        self._write_reg(_CTRL1_XL, 0x4A)
        # Gyro:  104 Hz, ±2000 dps (CTRL2_G = 0x4C)
        self._write_reg(_CTRL2_G, 0x4C)
        return True

    def read(self):
        """Return (accel_xyz m/s², gyro_xyz rad/s) as two 3-tuples."""
        raw_g = self._read_6(_OUTX_L_G)
        raw_a = self._read_6(_OUTX_L_A)
        accel = tuple(v * _ACCEL_SENS for v in raw_a)
        gyro  = tuple(v * _GYRO_SENS  for v in raw_g)
        return accel, gyro

    def _read_reg(self, addr: int) -> int:
        return self._spi.xfer2([addr | _READ_FLAG, 0x00])[1]

    def _write_reg(self, addr: int, val: int):
        self._spi.xfer2([addr & 0x7F, val])

    def _read_6(self, addr: int):
        rx = self._spi.xfer2([addr | _READ_FLAG] + [0x00] * 6)
        return struct.unpack_from('<3h', bytes(rx[1:]))  # 6 bytes → 3 × int16


class ImuSensorNode(Node):
    def __init__(self):
        super().__init__('imu_sensor_node')
        self.declare_parameter('spi_bus',    0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('frame_id',   'imu_link')
        self.declare_parameter('rate_hz',    100.0)

        bus    = self.get_parameter('spi_bus').value
        device = self.get_parameter('spi_device').value
        self._frame_id = self.get_parameter('frame_id').value
        rate   = self.get_parameter('rate_hz').value

        self._imu = ISM330DHCX(bus, device)
        if not self._imu.init():
            self.get_logger().fatal(
                f'ISM330DHCX WHO_AM_I mismatch — check SPI wiring on /dev/spidev{bus}.{device}'
            )
            raise RuntimeError('ISM330DHCX init failed')

        self.get_logger().info(
            f'ISM330DHCX initialised on /dev/spidev{bus}.{device} at {rate:.0f} Hz'
        )

        self._pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.create_timer(1.0 / rate, self._timer_cb)

        # Covariance from ISM330DHCX datasheet noise densities:
        #   accel: 70 μg/√Hz → ~0.021 m/s² stddev at 104 Hz
        #   gyro:  4 mdps/√Hz → ~0.009 rad/s stddev at 104 Hz
        self._accel_cov = [0.021**2, 0, 0, 0, 0.021**2, 0, 0, 0, 0.021**2]
        self._gyro_cov  = [0.009**2, 0, 0, 0, 0.009**2, 0, 0, 0, 0.009**2]
        # Orientation unknown until madgwick runs — mark as unknown
        self._orient_cov = [-1.0] + [0.0] * 8

    def _timer_cb(self):
        try:
            accel, gyro = self._imu.read()
        except Exception as exc:
            self.get_logger().error(f'SPI read error: {exc}', throttle_duration_sec=5.0)
            return

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.linear_acceleration_covariance = self._accel_cov

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance = self._gyro_cov

        # Orientation is not provided by raw data — madgwick will compute it
        msg.orientation_covariance = self._orient_cov

        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ImuSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
