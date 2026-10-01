#include <iostream>
#include <bitset>
#include <sstream>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iomanip>
#include <thread>

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
    SerialPort(const string& port, unsigned int baudrate)
        : io(), serial(io, port), timer(io), timeout(false) {
        serial.set_option(serial_port_base::baud_rate(baudrate));
        serial.set_option(serial_port_base::character_size(8)); // 数据位
        serial.set_option(serial_port_base::parity(serial_port_base::parity::none)); // 无校验位
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one)); // 1个停止位
        cout << "串口 " << port << " 已打开" << endl;
    }

    void write(const vector<unsigned char>& data) {
        boost::asio::write(serial, buffer(data));
    }

    vector<unsigned char> read(size_t length, int timeout_ms = 500) {
        vector<unsigned char> buffer(length);
        timeout = false;

        timer.expires_after(std::chrono::milliseconds(timeout_ms));
        timer.async_wait(boost::bind(&SerialPort::on_timeout, this, boost::asio::placeholders::error));

        boost::system::error_code ec;
        size_t bytes_transferred = 0;
        serial.async_read_some(boost::asio::buffer(buffer),
                               boost::bind(&SerialPort::on_read_complete, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred, &ec, &bytes_transferred));

        io.run();
        io.reset();

        if (timeout) {
            throw std::runtime_error("串口读取超时");
        }

        if (ec) {
            throw boost::system::system_error(ec);
        }

        buffer.resize(bytes_transferred);
        return buffer;
    }

private:
    void on_timeout(const boost::system::error_code& ec) {
        if (!ec) {
            timeout = true;
            serial.cancel(); 
        }
    }

    void on_read_complete(const boost::system::error_code& ec, size_t bytes_transferred, boost::system::error_code* out_ec, size_t* out_bytes_transferred) {
        *out_ec = ec;
        *out_bytes_transferred = bytes_transferred;
        timer.cancel(); 
    }

    io_context io;
    serial_port serial;
    steady_timer timer;
    bool timeout;
};

int getAddressByName(const string& reg_name) {
    for (const auto& reg : regdict) {
        if (reg.name == reg_name) {
            return reg.address;  
        }
    }
    return -1;  
}

// 地址转换和二进制转十六进制函数
string convertAddressToBinary_read(int address_dec, const string& id_value) {
    string address_bin = bitset<32>(address_dec).to_string();
    size_t first_one = address_bin.find('1');

    if (first_one != string::npos) {
        address_bin = address_bin.substr(first_one);
    } else {
        address_bin = "0"; 
    }

    string formatted_output = "0000000" + address_bin + "000000000000" + id_value;
    return formatted_output;
}

string convertAddressToBinary_write(int address_dec, const string& id_value) {
    string address_bin = bitset<32>(address_dec).to_string();
    size_t first_one = address_bin.find('1');

    if (first_one != string::npos) {
        address_bin = address_bin.substr(first_one);
    } else {
        address_bin = "0"; 
    }

    string formatted_output = "0000010" + address_bin + "000000000000" + id_value;
    return formatted_output;
}

string binaryToHex(const string& binary_string) {
    unsigned long decimal_value = stoul(binary_string, nullptr, 2);
    stringstream ss;
    ss << hex << uppercase << decimal_value;
    return ss.str();
}

// 处理十六进制数据的函数
vector<unsigned char> processHexData(const string& hex_data) {
    string padded_hex = hex_data;
    if (padded_hex.length() < 8) {
        padded_hex = string(8 - padded_hex.length(), '0') + padded_hex;
    }

    vector<unsigned char> processed_data;

    for (size_t i = padded_hex.length(); i > 0; i -= 2) {
        string byte_str = padded_hex.substr(i - 2, 2);
        unsigned char byte_value = static_cast<unsigned char>(stoi(byte_str, nullptr, 16));
        processed_data.push_back(byte_value);
    }

    return processed_data;
}

void write_register(SerialPort& serial, int address_decimal, const string& id_value, const vector<int>& values_to_write) {
    if (values_to_write.size() > 6) {
        cerr << "超过允许的值数量，最多只能写入6个值！" << endl;
        return;
    }

    vector<int> first_chunk(values_to_write.begin(), values_to_write.begin() + min(4, (int)values_to_write.size()));
    vector<int> second_chunk(values_to_write.begin() + first_chunk.size(), values_to_write.end());

    // 构造并发送第一段数据
    auto construct_and_send = [&](const vector<int>& chunk, int addr) {
        // 转换地址为二进制->十六进制
        string result_bin = convertAddressToBinary_write(addr, id_value);
        string result_hex = binaryToHex(result_bin);
        vector<unsigned char> processed_data = processHexData(result_hex);

        // 构造数据帧
        vector<unsigned char> send_buffer;
        send_buffer.push_back(0xAA);
        send_buffer.push_back(0xAA);
        send_buffer.insert(send_buffer.end(), processed_data.begin(), processed_data.end());

        // 写入值
        for (int value : chunk) {
            send_buffer.push_back(value & 0xFF);  // 低八位
            send_buffer.push_back((value >> 8) & 0xFF);  // 高八位
        }

        while (send_buffer.size() < 14) {
            send_buffer.push_back(0x00);
        }

        // 添加固定字节
        send_buffer.push_back(chunk.size() * 2);  // 数据长度
        send_buffer.push_back(0x00);
        send_buffer.push_back(0x01);
        send_buffer.push_back(0x00);

        // 计算校验和
        unsigned char checksum = 0;
        for (size_t k = 2; k < send_buffer.size(); ++k) {
            checksum += send_buffer[k];
        }
        send_buffer.push_back(checksum & 0xFF);
        send_buffer.push_back(0x55);
        send_buffer.push_back(0x55);

        // 打印并发送
        cout << "发送的完整数据: ";
        for (auto byte : send_buffer) {
            cout << hex << setw(2) << setfill('0') << (int)byte << " ";
        }
        cout << endl;

        serial.write(send_buffer);

        // 读掉响应数据，不处理
        try {
            vector<unsigned char> discard_buffer = serial.read(32, 500); 
        } catch (const std::exception& e) {
            cerr << "写操作后的响应数据读取失败: " << e.what() << endl;
        }
    };

    // 发送第一段
    construct_and_send(first_chunk, address_decimal);

    // 如果有第二段值，发送第二段
    if (!second_chunk.empty()) {
        construct_and_send(second_chunk, address_decimal + 8);  // 地址后移 8
    }
}



void read_register(SerialPort& serial, int address_decimal, const string& id_value, size_t length_to_read) {
    vector<int> parsed_values1; // 存储第一段解析的值
    vector<int> parsed_values2; // 存储第二段解析的值

    vector<unsigned char> send_buffer; 

    string result_bin = convertAddressToBinary_read(address_decimal, id_value);
    string result_hex = binaryToHex(result_bin);  
    vector<unsigned char> processed_data = processHexData(result_hex);

    send_buffer.push_back(0xAA); 
    send_buffer.push_back(0xAA);
    send_buffer.insert(send_buffer.end(), processed_data.begin(), processed_data.end());
    send_buffer.push_back(0x08); 
    send_buffer.insert(send_buffer.end(), 7, 0x00); 
    send_buffer.push_back(0x01);
    send_buffer.push_back(0x00);
    send_buffer.push_back(0x01);
    send_buffer.push_back(0x00);

    unsigned char checksum = 0;
    for (size_t k = 2; k < send_buffer.size(); ++k) {
        checksum += send_buffer[k];
    }
    send_buffer.push_back(checksum & 0xFF); // 添加校验和
    send_buffer.push_back(0x55);
    send_buffer.push_back(0x55);

    // 打印发送的数据
    cout << "发送的完整数据: ";
    for (auto byte : send_buffer) {
        cout << hex << setw(2) << setfill('0') << (int)byte << " ";
    }
    cout << endl;

    serial.write(send_buffer); 
    // 读取响应数据
    vector<unsigned char> received_data = serial.read(length_to_read);

    // 打印接收到的原始响应数据
    cout << "接收到的原始数据: ";
    for (auto byte : received_data) {
        cout << hex << setw(2) << setfill('0') << (int)byte << " ";
    }
    cout << endl;

    // 找到第一个 0xAA 的位置
    size_t start_pos = 0;
    while (start_pos < received_data.size() && received_data[start_pos] != 0xAA) {
        start_pos++;
    }

    // 从第一个 0xAA 开始解析数据
    for (size_t j = start_pos + 6; j < start_pos + 14; j += 2) {
        if (j + 1 < received_data.size()) {
            int value = (received_data[j + 1] << 8) | received_data[j];
            if (value > 6000) { 
                value = 0;
            }
            parsed_values1.push_back(value);
        }
    }

    address_decimal += 8;  // 地址后移
    result_bin = convertAddressToBinary_read(address_decimal, id_value);
    result_hex = binaryToHex(result_bin);  // 将二进制字符串转换为十六进制
    processed_data = processHexData(result_hex);


    send_buffer.push_back(0xAA); 
    send_buffer.push_back(0xAA);
    send_buffer.insert(send_buffer.end(), processed_data.begin(), processed_data.end());
    send_buffer.push_back(0x04); 
    send_buffer.insert(send_buffer.end(), 7, 0x00); 
    send_buffer.push_back(0x01);
    send_buffer.push_back(0x00);
    send_buffer.push_back(0x01);
    send_buffer.push_back(0x00);

    checksum = 0;
    for (size_t k = 2; k < send_buffer.size(); ++k) {
        checksum += send_buffer[k];
    }
    send_buffer.push_back(checksum & 0xFF); // 添加校验和
    send_buffer.push_back(0x55);
    send_buffer.push_back(0x55);

    cout << "发送的完整数据（后半段）: ";
    for (auto byte : send_buffer) {
        cout << hex << setw(2) << setfill('0') << (int)byte << " ";
    }
    cout << endl;

    serial.write(send_buffer); 

    received_data = serial.read(length_to_read);

    // 打印第二段接收到的原始响应数据
    cout << "接收到的原始数据（后半段）: ";
    for (auto byte : received_data) {
        cout << hex << setw(2) << setfill('0') << (int)byte << " ";
    }
    cout << endl;

    // 找到第一个 0xAA 的位置
    start_pos = 0;
    while (start_pos < received_data.size() && received_data[start_pos] != 0xAA) {
        start_pos++;
    }

    // 从第一个 0xAA 开始解析数据
    for (size_t j = start_pos + 6; j < start_pos + 14; j += 2) {
        if (j + 1 < received_data.size()) {
            int value = (received_data[j + 1] << 8) | received_data[j];
            if (value > 6000) { 
                value = 0;
            }
            parsed_values2.push_back(value);
        }
    }

    // 输出结果
    cout << "参数： ";
    // 输出第一段解析的值
    for (int value : parsed_values1) {
        cout << dec << value << " ";
    }

    // 输出第二段解析的前两个值
    for (size_t i = 0; i < 2 && i < parsed_values2.size(); ++i) {
        cout << dec << parsed_values2[i] << " ";
    }
    cout << endl;
}

int main() {
    string port = "/dev/ttyUSB1";  // 根据实际情况修改串口名
    SerialPort serial(port, 115200);

    string id_value = "01";

    // 写入寄存器示例
    write_register(serial, getAddressByName("speedSet"), id_value, {1000, 1000, 1000, 1000, 1000, 1000});
    this_thread::sleep_for(std::chrono::seconds(1));

    write_register(serial, getAddressByName("angleSet"), id_value, {1000, 1000, 1000, 1000, 1000, 0});
    this_thread::sleep_for(std::chrono::seconds(1));

    write_register(serial, getAddressByName("forceSet"), id_value, {1000, 1000, 1000, 1000, 1000, 1000});
    this_thread::sleep_for(std::chrono::seconds(1));

    write_register(serial, getAddressByName("angleSet"), id_value, {1000, 1000, 1000, 1000, 1000, 1000});
    this_thread::sleep_for(std::chrono::seconds(1));

    read_register(serial, getAddressByName("angleSet"), id_value, 23);
    this_thread::sleep_for(std::chrono::seconds(1));
    read_register(serial, getAddressByName("angleAct"), id_value, 23);
    this_thread::sleep_for(std::chrono::seconds(1));
    read_register(serial, getAddressByName("forceAct"), id_value, 23);
    this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
