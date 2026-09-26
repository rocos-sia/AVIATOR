// 独立回归测试：复用实际服务序列化代码，无需打开 HTTP 端口。
#define main joystick_web_main
#include "../src/main.cpp"
#undef main
int main() {
    joystick::State s;
    s.name = "中文手柄";
    s.rumble_error = "震动不可用：需要读写权限";
    if (quote(s.name) != "\"中文手柄\"") return 1;
    if (quote("\"\\\n") != "\"\\\"\\\\\\u000a\"") return 2;
    std::cout << json(s) << '\n';
}
