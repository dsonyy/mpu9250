import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Temperature

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250


class MPU9250Node(Node):
    def __init__(self):
        super().__init__("mpu9250_node")
        self.mpu = None
        self.imu_pub = self.create_publisher(Imu, "imu/raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)
        self.temp_pub = self.create_publisher(Temperature, "imu/temp", 10)

        self.rate = 10
        self.timer = self.create_timer(1.0 / self.rate, self.publish)

    def init_mpu(self):
        self.mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            address_mpu_master=MPU9050_ADDRESS_68,
            address_mpu_slave=None,
            bus=1,
            gfs=GFS_1000,
            afs=AFS_8G,
            mfs=AK8963_BIT_16,
            mode=AK8963_MODE_C100HZ)

        self.mpu.configure()  # Apply the settings to the registers.

    def calibrate(self):
        self.mpu.calibrate()  # Calibrate sensors
        self.mpu.configure()  # The calibration resets the sensors, so you need to reconfigure them

    def get_calibration(self):
        abias = self.mpu.abias  # Get the master accelerometer biases
        gbias = self.mpu.gbias  # Get the master gyroscope biases
        magScale = self.mpu.magScale  # Get magnetometer soft iron distortion
        mbias = self.mpu.mbias  # Get magnetometer hard iron distortion
        return abias, gbias, magScale, mbias

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        frame_id = "base_link"

        accel = self.mpu.readAccelerometerMaster()
        gyro = self.mpu.readGyroscopeMaster()
        mag = self.mpu.readMagnetometerMaster()
        temp = self.mpu.readTemperatureMaster()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = frame_id
        imu_msg.linear_acceleration.x = gyro[0]
        imu_msg.linear_acceleration.y = gyro[1]
        imu_msg.linear_acceleration.z = gyro[2]
        imu_msg.angular_velocity.x = accel[0]
        imu_msg.angular_velocity.y = accel[1]
        imu_msg.angular_velocity.z = accel[2]

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = frame_id
        mag_msg.magnetic_field.x = mag[0]
        mag_msg.magnetic_field.y = mag[1]
        mag_msg.magnetic_field.z = mag[2]

        temp_msg = Temperature()
        temp_msg.header.stamp = stamp
        temp_msg.header.frame_id = frame_id
        temp_msg.temperature = temp

        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)
        self.temp_pub.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    mpu9250_node = MPU9250Node()
    rclpy.spin(mpu9250_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
