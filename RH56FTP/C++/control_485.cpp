#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <iomanip>
#include <thread> // 添加线程库用于延时操作

using namespace std;
using namespace boost::asio;

// 寄存器字典
const struct {
    const char* name;
    int address;
} regdict[] = {
    {"angleSet", 1486},
    {"forceSet", 1498},
    {"speedSet", 1522},
    {"angleAct", 1546},
    {"forceAct", 1582}
};

class SerialPort {
public:
    SerialPort(const string& port, unsigned int baudrate) : io(), serial(io, port) {
        serial.set_option(serial_port_base::baud_rate(baudrate));
        serial.set_option(serial_port_base::character_size(8)); // 数据位
        serial.set_option(serial_port_base::parity(serial_port_base::parity::none)); // 无校验位
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one)); // 1个停止位
        cout << "串口 " << port << " 已打开" << endl;
    }

    void write(const vector<unsigned char>& data) {
        boost::asio::write(serial, buffer(data));
    }

    vector<unsigned char> read(size_t length) {
        vector<unsigned char> buffer(length);
        boost::asio::read(serial, boost::asio::buffer(buffer.data(), length));
        return buffer;  
    }

private:
    io_context io;
    serial_port serial;
};

// 根据寄存器名称获取地址
int getAddressByName(const string& reg_name) {
    for (const auto& reg : regdict) {
        if (reg.name == reg_name) {
            return reg.address;  // 返回匹配的地址
        }
    }
    return -1;  // 未找到寄存器名称，返回 -1
}

// 写寄存器函数
void write_register(SerialPort& serial, int address_decimal, const string& id_value, const vector<int>& values_to_write) {
    if (values_to_write.size() > 6) {
        cerr << "超过允许的值数量，最多只能写入6个值！" << endl;
        return;
    }

    // 构建并发送数据
    vector<unsigned char> send_buffer;
    send_buffer.push_back(0xEB); 
    send_buffer.push_back(0x90);
    send_buffer.push_back(static_cast<unsigned char>(stoi(id_value)));  // 将 ID 转换为 unsigned char
    size_t num_values = values_to_write.size();
    unsigned char data_length = static_cast<unsigned char>(num_values * 2 + 3); 
    send_buffer.push_back(data_length);  
    send_buffer.push_back(0x12);  // 写寄存器命令
    send_buffer.push_back(static_cast<unsigned char>(address_decimal & 0xFF));  // 地址低八位
    send_buffer.push_back(static_cast<unsigned char>((address_decimal >> 8) & 0xFF));  // 地址高八位

    // 将要写入的值添加到数据包中
    for (int value : values_to_write) {
        send_buffer.push_back(static_cast<unsigned char>(value & 0xFF));  // 写入值低八位
        send_buffer.push_back(static_cast<unsigned char>((value >> 8) & 0xFF));  // 写入值高八位
    }

    // 计算校验和，从 ID 部分开始
    unsigned char checksum = 0;
    for (size_t k = 2; k < send_buffer.size(); ++k) {
        checksum += send_buffer[k];
    }

    send_buffer.push_back(checksum & 0xFF);  // 添加校验和

    // 输出发送的数据
    cout << "发送的写入数据: ";
    for (auto byte : send_buffer) {
        cout << hex << setw(2) << setfill('0') << (int)byte << " ";
    }
    cout << endl;

    // 写入数据
    serial.write(send_buffer);

    // 读取并丢弃写入后的响应
    try {
        vector<unsigned char> discard_buffer = serial.read(9); 
        cout << "写操作后的响应数据已丢弃，字节数: " << discard_buffer.size() << endl;
    } catch (const std::exception& e) {
        cerr << "写操作后的响应数据读取失败: " << e.what() << endl;
    }
}

// 读寄存器函数
void read_register(SerialPort& serial, int address_decimal, const string& id_value, size_t length_to_read) {
    vector<int> parsed_values;

    // 构建并发送请求数据
    vector<unsigned char> send_buffer;
    send_buffer.push_back(0xEB); 
    send_buffer.push_back(0x90);
    send_buffer.push_back(static_cast<unsigned char>(stoi(id_value)));  // 将 ID 转换为 unsigned char
    send_buffer.push_back(0x04);
    send_buffer.push_back(0x11);
    send_buffer.push_back(static_cast<unsigned char>(address_decimal & 0xFF));  // 地址低八位
    send_buffer.push_back(static_cast<unsigned char>((address_decimal >> 8) & 0xFF));  // 地址高八位
    send_buffer.push_back(0x0C); // 读取寄存器长度

    // 计算校验和
    unsigned char checksum = 0;
    for (size_t k = 2; k < send_buffer.size(); ++k) {
        checksum += send_buffer[k];
    }

    send_buffer.push_back(checksum & 0xFF);

    cout << "发送的完整数据: ";
    for (auto byte : send_buffer) {
        cout << hex << setw(2) << setfill('0') << (int)byte << " ";
    }
    cout << endl;

    serial.write(send_buffer);  // 写入串口

    // 读取响应数据
    vector<unsigned char> received_data = serial.read(length_to_read);

    // 打印接收到的原始响应数据
    cout << "接收到的原始响应数据: ";
    for (auto byte : received_data) {
        cout << hex << setw(2) << setfill('0') << (int)byte << " ";
    }
    cout << endl;

    // 解析响应
    for (size_t j = 7; j < 19; j += 2) {
        if (j + 1 < received_data.size()) {
            int value = (received_data[j + 1] << 8) | received_data[j];
            if (value > 6000) {
                value = 0;
            }
            parsed_values.push_back(value);
        }
    }

    // 输出参数
    cout << "参数: ";
    for (int value : parsed_values) {
        cout << dec << value << " ";
    }
    cout << endl;
}

// 主函数
int main() {
    string port = "/dev/ttyUSB0";  // 根据实际情况修改串口名
    SerialPort serial(port, 115200);  // 打开串口

    string id_value = "01";

    // 写入寄存器示例
    write_register(serial, getAddressByName("speedSet"), id_value, {1000, 1000, 1000, 1000, 1000, 1000});
    this_thread::sleep_for(std::chrono::seconds(1));

    write_register(serial, getAddressByName("angleSet"), id_value, {0, 1000, 1000, 1000, 1000, 0});
    this_thread::sleep_for(std::chrono::seconds(1));

    write_register(serial, getAddressByName("forceSet"), id_value, {1000, 1000, 1000, 1000, 1000, 1000});
    this_thread::sleep_for(std::chrono::seconds(1));

    write_register(serial, getAddressByName("angleSet"), id_value, {1000, 1000, 1000, 1000, 1000, 1000});
    this_thread::sleep_for(std::chrono::seconds(1));

    // 读取寄存器示例
    read_register(serial, getAddressByName("angleSet"), id_value, 20);
    this_thread::sleep_for(std::chrono::seconds(1));
    read_register(serial, getAddressByName("angleAct"), id_value, 20);
    this_thread::sleep_for(std::chrono::seconds(1));
    read_register(serial, getAddressByName("forceAct"), id_value, 20);
    this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}

