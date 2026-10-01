import rospy
import serial
import time
from inspire_hand.msg import get_force_act_1, get_angle_act_1, set_force_1, set_angle_1, set_speed_1


class InspireHandController:
    def __init__(self, serial_port, baud_rate, hand_id):
        self.ser = self.openSerial(serial_port, baud_rate)
        self.hand_id = hand_id
        self.force_publisher = rospy.Publisher("force_data", get_force_act_1, queue_size=10)
        self.angle_publisher = rospy.Publisher("angle_data", get_angle_act_1, queue_size=10)
        rospy.Subscriber("set_angle_data", set_angle_1, self.angle_callback)
        rospy.Subscriber("set_force_data", set_force_1, self.force_callback)
        rospy.Subscriber("set_speed_data", set_speed_1, self.speed_callback)

    def openSerial(self, SERIAL_PORT, BAUD_RATE):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            if not ser.isOpen():
                rospy.loginfo("Serial port was not open, opening now.")
                ser.open()  # 仅在未打开时才打开
            rospy.loginfo("Serial port open: %s" % SERIAL_PORT)
            return ser
        except serial.SerialException as e:
            rospy.logerr("Failed to open serial port: %s" % e)
            return None

    # 定义力传感器数据地址和手指 ID
    FORCE_SENSOR_RANGES = {
        1: (1582,),  # Pinky
        2: (1584,),  # Ring Finger
        3: (1586,),  # Middle Finger
        4: (1588,),  # Index Finger
        5: (1590,),  # Thumb Flexion
        6: (1592,),  # Thumb Abduction
    }

    # 定义角度传感器数据地址和手指 ID
    ANGLE_ACT_RANGES = {
        1: (1546,),  # Pinky
        2: (1548,),  # Ring Finger
        3: (1550,),  # Middle Finger
        4: (1552,),  # Index Finger
        5: (1554,),  # Thumb Flexion
        6: (1556,),  # Thumb Abduction
    }

    # 定义设置角度的地址和手指 ID
    ANGLE_SET_RANGES = {
        1: (1486,),  # Pinky
        2: (1488,),  # Ring Finger
        3: (1490,),  # Middle Finger
        4: (1492,),  # Index Finger
        5: (1494,),  # Thumb Flexion
        6: (1496,),  # Thumb Abduction
    }

    # 定义设置力的地址和手指 ID
    FORCE_SET_RANGES = {
        1: (1498,),  # Pinky
        2: (1500,),  # Ring Finger
        3: (1502,),  # Middle Finger
        4: (1504,),  # Index Finger
        5: (1506,),  # Thumb Flexion
        6: (1508,),  # Thumb Abduction
    }

    # 定义设置速度的地址和手指 ID
    SPEED_SET_RANGES = {
        1: (1522,),  # Pinky
        2: (1524,),  # Ring Finger
        3: (1526,),  # Middle Finger
        4: (1528,),  # Index Finger
        5: (1530,),  # Thumb Flexion
        6: (1532,),  # Thumb Abduction
    }

    #创建字典映射手指ID到名称
    FINGER_NAMES = {
        1: "Pinky",
        2: "Ring Finger",
        3: "Middle Finger",
        4: "Index Finger",
        5: "Thumb Flexion",
        6: "Thumb Abduction",
    }

    def readRegister(self, address, length):
        if address < 0 or length <= 0:
            rospy.logerr("Invalid address or length for reading register.")
            return []

        bytes = [0xEB, 0x90]  # 帧头
        bytes.append(self.hand_id)  # id
        bytes.append(0x04)  # 固定字节
        bytes.append(0x11)  # cmd 读寄存器命令标志
        bytes.append(address & 0xFF)  # 寄存器起始地址低八位
        bytes.append((address >> 8) & 0xFF)  # 寄存器起始地址高八位
        bytes.append(length)
        checksum = sum(bytes[2:]) & 0xFF  # 计算校验和
        bytes.append(checksum)  # 低八位校验和

        rospy.loginfo("Sending to serial: %s", [hex(b) for b in bytes])
        self.ser.write(bytes)  # 向串口写入数据
        time.sleep(0.01)  # 延时10ms
        recv = self.ser.read_all()  # 从端口读字节数据

        if len(recv) == 0:  # 如果返回的数据长度为0，直接返回
            return []

        num = (recv[3] & 0xFF) - 3  # 寄存器数据所返回的数量
        if num <= 0 or len(recv) < 7 + num:  # 检查数量有效性和接收数据长度
            rospy.logerr("Invalid data length or count.")
            return []

        val = [recv[7 + i] for i in range(num)]  # 提取寄存器值

        # 处理有符号整数
        val_act = []
        for i in range(6):
            if 2 * i + 1 < len(val):  # 确保不会超出范围
                value_act = (val[2 * i] & 0xFF) + ((val[1 + 2 * i] & 0xFF) << 8)
                if value_act > 32767:
                    value_act -= 65536
                val_act.append(value_act)
            else:
                rospy.logwarn(f"数据不足，无法处理索引 {i} 的值")
                break

        rospy.loginfo('读到的值依次为：%s', ' '.join(map(str, val_act)))
        return val_act

    def writeRegister(self, address, val):
        bytes = [0xEB, 0x90]  # 帧头
        bytes.append(self.hand_id)  # id
        bytes.append(12 + 3)  # len
        bytes.append(0x12)  # cmd 写寄存器命令标志
        bytes.append(address & 0xFF)  # 寄存器起始地址低八位
        bytes.append((address >> 8) & 0xFF)  # 寄存器起始地址高八位
        bytes.extend(val)  # 添加要写入的值
        checksum = sum(bytes[2:]) & 0xFF  # 计算校验和
        bytes.append(checksum)  # 低八位校验和
        rospy.loginfo("Sending to serial: %s", [hex(b) for b in bytes])
        self.ser.write(bytes)  # 向串口写入数据
        time.sleep(0.01)  # 延时10ms
        self.ser.read_all()  # 把返回帧读掉，不处理

    def publish_data(self):
        start_time = time.time()

        # 读取实际受力并发布数据
        if self.force_publisher.get_num_connections() > 0:
            force_data_msg = get_force_act_1()
            force_data_msg.finger_ids = []
            force_data_msg.force_values = []
            force_data_msg.finger_names = []

            for finger_id, (address,) in self.FORCE_SENSOR_RANGES.items():
                force_values = self.readRegister(address, 12)

                if isinstance(force_values, list) and len(force_values) == 6 and all(
                        isinstance(v, int) for v in force_values):
                    force_data_msg.finger_ids.extend(range(finger_id, finger_id + 6))
                    force_data_msg.force_values.extend(force_values)
                    force_data_msg.finger_names.extend(
                        [self.FINGER_NAMES.get(fid, "Unknown Finger") for fid in range(finger_id, finger_id + 6)])
                    break
                else:
                    rospy.logerr("读取的值不是有效的整数，无法发布数据。")

            self.force_publisher.publish(force_data_msg)
            rospy.loginfo(f"已发布力数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

        # 读取实际角度并发布数据
        if self.angle_publisher.get_num_connections() > 0:
            angle_data_msg = get_angle_act_1()
            angle_data_msg.finger_ids = []
            angle_data_msg.angle_values = []
            angle_data_msg.finger_names = []

            for finger_id, (address,) in self.ANGLE_ACT_RANGES.items():
                angle_values = self.readRegister(address, 12)

                if isinstance(angle_values, list) and len(angle_values) == 6 and all(
                        isinstance(v, int) for v in angle_values):
                    angle_data_msg.finger_ids.extend(range(finger_id, finger_id + 6))
                    angle_data_msg.angle_values.extend(angle_values)
                    angle_data_msg.finger_names.extend(
                        [self.FINGER_NAMES.get(fid, "Unknown Finger") for fid in range(finger_id, finger_id + 6)])
                    break
                else:
                    rospy.logerr("读取的值不是有效的整数，无法发布数据。")

            self.angle_publisher.publish(angle_data_msg)
            rospy.loginfo(f"已发布角度数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

    def angle_callback(self, msg):
        """接收设置角度的消息并写入相应的寄存器。"""
        values_to_write = []
        
        for finger_id, angle in zip(msg.finger_ids, msg.angles):
            if 0 <= angle <= 1000:  # 确保角度在0到1000之间
                low_byte = angle & 0xFF          # 低字节
                high_byte = (angle >> 8) & 0xFF   # 高字节
                values_to_write.extend([low_byte, high_byte]) 
            else:
                rospy.logwarn(f"未找到手指 ID {finger_id} 的地址")
                    
        first_finger_id, (first_address,)  = next(iter(self.ANGLE_SET_RANGES.items()))

        if first_address is not None:
        	self.writeRegister(first_address, values_to_write)

    def force_callback(self, msg):
        """接收设置力阈值的消息并写入相应的寄存器。"""
        values_to_write = []

        for finger_id, force in zip(msg.finger_ids, msg.forces):
            if 0 <= force <= 1000:  # 确保力阈值在0到1000之间
                low_byte = force & 0xFF  # 低字节
                high_byte = (force >> 8) & 0xFF  # 高字节
                values_to_write.extend([low_byte, high_byte])
            else:
                rospy.logwarn(f"未找到手指 ID {finger_id} 的地址")

        first_finger_id, (first_address,) = next(iter(self.FORCE_SET_RANGES.items()))

        if first_address is not None:
            self.writeRegister(first_address, values_to_write)

    def speed_callback(self, msg):
        """接收设置速度的消息并写入相应的寄存器。"""
        values_to_write = []

        for finger_id, speed in zip(msg.finger_ids, msg.speeds):
            if 0 <= speed <= 1000:  # 确保速度在0到1000之间
                low_byte = speed & 0xFF  # 低字节
                high_byte = (speed >> 8) & 0xFF  # 高字节
                values_to_write.extend([low_byte, high_byte])
            else:
                rospy.logwarn(f"未找到手指 ID {finger_id} 的地址")

        first_finger_id, (first_address,) = next(iter(self.SPEED_SET_RANGES.items()))

        if first_address is not None:
            self.writeRegister(first_address, values_to_write)

def main():
    rospy.init_node('data_publisher_node', anonymous=True)
    controller = InspireHandController('/dev/ttyUSB0', 115200, 1)

    rate = rospy.Rate(50)
    try:
        while not rospy.is_shutdown():
            controller.publish_data()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("数据发布节点被手动停止")
    finally:
        controller.ser.close()


if __name__ == "__main__":
    main()
