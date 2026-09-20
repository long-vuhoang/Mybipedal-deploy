#pragma once

// Flight-recorder kiểu ring buffer để debug bám quỹ đạo (joint_cmd vs joint_states).
//
//  - Chỉ 1 writer thread cho mỗi RingTrace (không lock, không cấp phát trên hot path).
//  - Giữ lại N dòng MỚI NHẤT (ring) -> nếu robot đổ, đoạn cuối vẫn còn.
//  - Dump ra CSV 1 lần khi kết thúc (Dump() tự "đóng băng" buffer trước khi đọc).
//  - Header-only, chỉ phụ thuộc thư viện chuẩn -> test độc lập được.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

namespace mybipedal_deploy::rl_control_module {

class RingTrace {
 public:
  // col_names: tên các cột dữ liệu (float). Cột thời gian `t_s` được tự thêm vào đầu.
  void Init(std::vector<std::string> col_names, size_t capacity_rows) {
    cols_ = std::move(col_names);
    ncols_ = cols_.size();
    cap_ = capacity_rows;
    enabled_ = cap_ > 0 && ncols_ > 0;
    if (!enabled_) return;
    t_ns_.assign(cap_, 0);
    data_.assign(cap_ * ncols_, 0.0f);
    head_.store(0, std::memory_order_relaxed);
    frozen_.store(false, std::memory_order_relaxed);
  }

  bool enabled() const { return enabled_; }
  size_t ncols() const { return ncols_; }

  // Hot path. vals trỏ tới đúng ncols() float. Chỉ gọi từ 1 thread.
  void Push(int64_t t_ns, const float* vals) {
    if (!enabled_ || frozen_.load(std::memory_order_relaxed)) return;
    const uint64_t h = head_.load(std::memory_order_relaxed);
    const size_t i = static_cast<size_t>(h % cap_);
    t_ns_[i] = t_ns;
    std::memcpy(&data_[i * ncols_], vals, ncols_ * sizeof(float));
    head_.store(h + 1, std::memory_order_release);
  }

  // Đóng băng + ghi CSV theo thứ tự thời gian. origin_ns: mốc 0 của cột t_s.
  // Trả về số dòng đã ghi (0 nếu lỗi/không có dữ liệu).
  size_t Dump(const std::string& path, int64_t origin_ns) {
    if (!enabled_) return 0;
    frozen_.store(true, std::memory_order_release);
    // cho writer đang dở dang kịp push xong dòng cuối (tránh dòng bị xé)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const uint64_t n = head_.load(std::memory_order_acquire);
    const size_t count = static_cast<size_t>(n < cap_ ? n : cap_);
    const size_t start = static_cast<size_t>(n > cap_ ? (n % cap_) : 0);

    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return 0;
    std::fputs("t_s", f);
    for (const auto& c : cols_) std::fprintf(f, ",%s", c.c_str());
    std::fputc('\n', f);
    for (size_t k = 0; k < count; ++k) {
      const size_t i = (start + k) % cap_;
      std::fprintf(f, "%.6f", static_cast<double>(t_ns_[i] - origin_ns) * 1e-9);
      const float* row = &data_[i * ncols_];
      for (size_t c = 0; c < ncols_; ++c) std::fprintf(f, ",%.7g", static_cast<double>(row[c]));
      std::fputc('\n', f);
    }
    std::fclose(f);
    return count;
  }

  uint64_t total_pushed() const { return head_.load(std::memory_order_acquire); }
  size_t capacity() const { return cap_; }

 private:
  bool enabled_{false};
  std::vector<std::string> cols_;
  size_t ncols_{0};
  size_t cap_{0};
  std::vector<int64_t> t_ns_;
  std::vector<float> data_;
  std::atomic<uint64_t> head_{0};
  std::atomic<bool> frozen_{false};
};

inline int64_t SteadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace mybipedal_deploy::rl_control_module
