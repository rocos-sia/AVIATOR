#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from std_msgs.msg import String
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse

# 定义 Modbus TCP 相关参数
MODBUS_IP = "192.168.11.210"
MODBUS_PORT = 6000

# 定义各部分数据地址范围
TOUCH_SENSOR_RANGES = {
    "小拇指": (3000, 3369),
    "无名指": (3370, 3739),
    "中指": (3740, 4109),
    "食指": (4110, 4479),
    "大拇指": (4480, 4899),
    "掌心": (4900, 5123),
}

# Modbus 每次最多读取寄存器的数量
MAX_REGISTERS_PER_READ = 125


def read_register_range(client, start_addr, end_addr):
    """
    批量读取指定地址范围内的寄存器数据。
    """
    register_values = []

    # 分段读取寄存器
    for addr in range(start_addr, end_addr + 1, MAX_REGISTERS_PER_READ * 2):
        # 确定当前段的寄存器数量
        current_count = min(MAX_REGISTERS_PER_READ, (end_addr - addr) // 2 + 1)

        # 批量读取寄存器
        response = client.read_holding_registers(address=addr, count=current_count)

        if isinstance(response, ExceptionResponse) or response.isError():
            rospy.logerr(f"读取寄存器 {addr} 失败: {response}")
            register_values.extend([0] * current_count)  # 用 0 填充失败的数据
        else:
            register_values.extend(response.registers)  # 添加成功读取的数据

    return register_values


def main():
    rospy.init_node("handcontrol_publisher", anonymous=True)

    # 创建发布者
    publisher = rospy.Publisher("touch_data", String, queue_size=10)

    # 创建 Modbus TCP 客户端
    modbus_client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    if not modbus_client.connect():
        rospy.logerr("无法连接到 Modbus 服务器，请检查 IP 和端口配置")
        return

    rospy.loginfo("Modbus 连接成功")

    # 发布频率
    rate = rospy.Rate(50)  # 50 Hz

    try:
        while not rospy.is_shutdown():
            start_time = time.time()

            touch_data = {}
            for part_name, (start_addr, end_addr) in TOUCH_SENSOR_RANGES.items():
                values = read_register_range(modbus_client, start_addr, end_addr)
                touch_data[part_name] = ", ".join(map(str, values))

            output = "\n".join(
                [f"{name}数据：{data}" for name, data in touch_data.items()]
            )

            # 发布消息
            msg = String()
            msg.data = output
            publisher.publish(msg)

            # 打印读取频率
            end_time = time.time()
            frequency = 1 / (end_time - start_time)
            rospy.loginfo(f"读取频率：{frequency:.2f} Hz")

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Modbus 发布节点被手动停止")
    finally:
        modbus_client.close()


if __name__ == "__main__":
    main()

