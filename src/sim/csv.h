#pragma once
#include <cstdio>
#include <string>
#include <vector>
#include <stdexcept>

inline std::string to_string_fixed(double v, int digits) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
  return std::string(buf);
}

class CSVWriter {
  FILE* f;

public:
  explicit CSVWriter(const std::string& path) : f(nullptr) {
    f = std::fopen(path.c_str(), "wb");
    if (!f) throw std::runtime_error("CSVWriter: failed to open " + path);
  }

  ~CSVWriter() {
    if (f) std::fclose(f);
    f = nullptr;
  }

  void write_row(const std::vector<std::string>& cols) {
    for (size_t i = 0; i < cols.size(); i++) {
      if (i) std::fputc(',', f);
      const std::string& s = cols[i];
      bool need_q = false;
      for (char c : s) {
        if (c == ',' || c == '"' || c == '\n' || c == '\r') { need_q = true; break; }
      }
      if (!need_q) {
        std::fwrite(s.data(), 1, s.size(), f);
      } else {
        std::fputc('"', f);
        for (char c : s) {
          if (c == '"') std::fputc('"', f);
          std::fputc(c, f);
        }
        std::fputc('"', f);
      }
    }
    std::fputc('\n', f);
  }
};
