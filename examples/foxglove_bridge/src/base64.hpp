// Minimal RFC 4648 base64 encoder (dependency-free).
//
// Used to carry the JPEG bytes inside the `foxglove.CompressedImage` JSON
// payload, whose `data` field is declared `contentEncoding: base64`.

#pragma once

#include <cstdint>
#include <string>

namespace aviator {

inline std::string base64_encode(const unsigned char* data, std::size_t len) {
  static const char tbl[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((len + 2) / 3) * 4);
  for (std::size_t i = 0; i < len; i += 3) {
    std::uint32_t n = static_cast<std::uint32_t>(data[i]) << 16;
    if (i + 1 < len) n |= static_cast<std::uint32_t>(data[i + 1]) << 8;
    if (i + 2 < len) n |= static_cast<std::uint32_t>(data[i + 2]);
    out.push_back(tbl[(n >> 18) & 0x3F]);
    out.push_back(tbl[(n >> 12) & 0x3F]);
    out.push_back((i + 1 < len) ? tbl[(n >> 6) & 0x3F] : '=');
    out.push_back((i + 2 < len) ? tbl[n & 0x3F] : '=');
  }
  return out;
}

inline std::string base64_encode(const std::string& s) {
  return base64_encode(reinterpret_cast<const unsigned char*>(s.data()),
                       s.size());
}

}  // namespace aviator
