#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import rospy
from pymodbus.client import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse
from inspire_hand_modbus.msg import get_force_act_1, get_angle_act_1, get_touch_act_1, set_angle_1, set_force_1, set_speed_1

# 定义 Modbus TCP 相关参数
MODBUS_IP = "192.168.11.210"
MODBUS_PORT = 6000

# 定义力传感器数据地址和手指 ID
FORCE_SENSOR_RANGES = {
    1: (1582,),  # Pinky
    2: (1584,),  # Ring Finger
    3: (1586,),  # Middle Finger
    4: (1588,),  # Index Finger
    5: (1590,),  # Thumb Flexion
    6: (1592,),  # Thumb Abduction
}

# 定义触觉传感器数据地址范围
TOUCH_ACT_RANGES = {
    1: (3000, 3369),  # Pinky
    2: (3370, 3739),  # Ring Finger
    3: (3740, 4109),  # Middle Finger
    4: (4110, 4479),  # Index Finger
    5: (4480, 4899),  # Thumb
    7: (4900, 5123),  # Palm
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

# 创建字典映射手指 ID 到名称
FINGER_NAMES = {
    1: "Pinky",
    2: "Ring Finger",
    3: "Middle Finger",
    4: "Index Finger",
    5: "Thumb Flexion",
    6: "Thumb Abduction",
    7: "Palm",
}

def read_signed_register(client, address):
    """读取单个寄存器并返回有符号整型值。"""
    response = client.read_holding_registers(address=address, count=1)
    
    if isinstance(response, ExceptionResponse) or response.isError():
        rospy.logerr(f"读取寄存器 {address} 失败: {response}")
        return 0  # 读取失败时返回 0
    else:
        value = response.registers[0]
        if value > 32767:  
            value -= 65536  
        return value

def read_register_range(client, start_addr, end_addr):
    """批量读取指定地址范围内的寄存器数据。"""
    register_values = []
    # 分段读取寄存器
    for addr in range(start_addr, end_addr + 1, 125 * 2):
        current_count = min(125, (end_addr - addr) // 2 + 1)
        response = client.read_holding_registers(address=addr, count=current_count)

        if isinstance(response, ExceptionResponse) or response.isError():
            rospy.logerr(f"读取寄存器 {addr} 失败: {response}")
            register_values.extend([0] * current_count)  # 填充读取失败的寄存器数据
        else:
            register_values.extend(response.registers)  # 添加成功读取的寄存器数据

    return register_values

def read_touch_data(client):
    """读取触觉数据并返回字典 (finger_id: touch_values)。"""
    touch_data = {}

    for finger_id, (start_addr, end_addr) in TOUCH_ACT_RANGES.items():
        touch_values = read_register_range(client, start_addr, end_addr)
        touch_data[finger_id] = []

        for i in range(0, len(touch_values), 2):
            if i < len(touch_values):  
                combined_value = int(touch_values[i])  
                touch_data[finger_id].append(combined_value)

        touch_data[finger_id] = [int(value) for value in touch_data[finger_id]]

        rospy.loginfo(f"Finger ID: {finger_id}, Touch Values: {touch_data[finger_id]}")

    return touch_data

def publish_data():
    """读取数据并发布力、角度和触觉数据。"""
    start_time = time.time()  

    # 检查连接情况并发布数据
    if force_publisher.get_num_connections() > 0:
        # 发布力数据
        force_data_msg = get_force_act_1() 
        force_data_msg.finger_ids = []  
        force_data_msg.force_values = []  
        force_data_msg.finger_names = []  

        for finger_id, (start_addr,) in FORCE_SENSOR_RANGES.items():
            force_value = read_signed_register(modbus_client, start_addr)
            force_data_msg.finger_ids.append(finger_id)
            force_data_msg.force_values.append(force_value)
            force_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

        force_publisher.publish(force_data_msg)
        rospy.loginfo(f"已发布力数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

    if angle_publisher.get_num_connections() > 0:
        # 发布角度数据
        angle_data_msg = get_angle_act_1()  
        angle_data_msg.finger_ids = []  
        angle_data_msg.angles = []  
        angle_data_msg.finger_names = []  

        for finger_id, (start_addr,) in ANGLE_ACT_RANGES.items():
            angle_value = read_signed_register(modbus_client, start_addr)  
            angle_data_msg.finger_ids.append(finger_id)
            angle_data_msg.angles.append(angle_value)  
            angle_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

        angle_publisher.publish(angle_data_msg)
        rospy.loginfo(f"已发布角度数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

    if touch_publisher.get_num_connections() > 0:
        # 发布触觉数据
        touch_data = read_touch_data(modbus_client)
        touch_data_msg = get_touch_act_1()
        touch_data_msg.finger_ids = list(touch_data.keys())
        touch_data_msg.touch_values = []
        touch_data_msg.finger_names = []

        for finger_id in touch_data_msg.finger_ids:
            touch_data_msg.touch_values.extend(touch_data[finger_id])  # 添加每个手指的触觉数组
            touch_data_msg.finger_names.append(FINGER_NAMES.get(finger_id, "Unknown Finger"))

        touch_publisher.publish(touch_data_msg)
        rospy.loginfo(f"已发布触觉数据，读取频率: {1 / (time.time() - start_time):.2f} Hz")

def angle_callback(msg):
    """接收设置角度的消息并写入相应的寄存器。"""
    for finger_id, angle in zip(msg.finger_ids, msg.angles):
        if 0 <= angle <= 1000:  # 确保角度在0到1000之间
            address = ANGLE_SET_RANGES.get(finger_id, (None,))[0]
            if address is not None:
                write_signed_register(modbus_client, address, angle)
            else:
                rospy.logwarn(f"未找到手指 ID {finger_id} 的地址")

def force_callback(msg):
    """接收设置力的消息并写入相应的寄存器。"""
    for finger_id, force in zip(msg.finger_ids, msg.forces):
        if 0 <= force <= 3000:  # 确保力在0到3000之间
            address = FORCE_SET_RANGES.get(finger_id, (None,))[0]
            if address is not None:
                write_signed_register(modbus_client, address, force)
            else:
                rospy.logwarn(f"未找到手指 ID {finger_id} 的地址")

def speed_callback(msg):
    """接收设置速度的消息并写入相应的寄存器。"""
    for finger_id, speed in zip(msg.finger_ids, msg.speeds):
        if 0 <= speed <= 1000:  # 确保速度在0到1000之间
            address = SPEED_SET_RANGES.get(finger_id, (None,))[0]
            if address is not None:
                write_signed_register(modbus_client, address, speed)
            else:
                rospy.logwarn(f"未找到手指 ID {finger_id} 的地址")

def write_signed_register(client, address, value):
    """写入一个有符号整型值到寄存器。"""
    if value < 0:
        value += 65536  # 转换为无符号值
    response = client.write_register(address, value)
    
    if isinstance(response, ExceptionResponse) or response.isError():
        rospy.logerr(f"写入寄存器 {address} 失败: {response}")

def data_reading_thread():
    """数据读取的线程。"""
    while not rospy.is_shutdown():
        publish_data()  # 读取并发布数据

def main():
    rospy.init_node("sensor_data_publisher", anonymous=True)

    # 创建发布者
    global force_publisher, angle_publisher, touch_publisher
    force_publisher = rospy.Publisher("force_data", get_force_act_1, queue_size=10)  
    angle_publisher = rospy.Publisher("angle_data", get_angle_act_1, queue_size=10)  
    touch_publisher = rospy.Publisher("touch_data", get_touch_act_1, queue_size=10)  
    rospy.Subscriber("set_angle_data", set_angle_1, angle_callback)  
    rospy.Subscriber("set_force_data", set_force_1, force_callback)  
    rospy.Subscriber("set_speed_data", set_speed_1, speed_callback)  

    # 创建 Modbus TCP 客户端
    global modbus_client
    modbus_client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    if not modbus_client.connect():
        rospy.logerr("无法连接到 Modbus 服务器，请检查 IP 和端口配置")
        return

    rospy.loginfo("Modbus 连接成功")

    # 启动数据读取线程
    read_thread = threading.Thread(target=data_reading_thread)
    read_thread.start()

    try:
        rospy.spin()  # 保持节点活动
    except rospy.ROSInterruptException:
        rospy.loginfo("数据发布节点被手动停止")
    finally:
        modbus_client.close()

if __name__ == "__main__":
    main()

