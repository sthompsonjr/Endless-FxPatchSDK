#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

template<typename T, size_t Size>
class CircularBuffer {
    static_assert((Size & (Size - 1)) == 0, "CircularBuffer Size must be a power of 2");
    static_assert(Size > 0, "CircularBuffer Size must be greater than 0");

public:
    void reset() {
        buffer_.fill(T{});
        writeHead_ = 0;
    }

    void write(T sample) {
        buffer_[writeHead_ & kMask] = sample;
        ++writeHead_;
    }

    [[nodiscard]] T read(int delaySamples) const {
        return buffer_[(writeHead_ - 1 - delaySamples) & kMask];
    }

    [[nodiscard]] T readLinear(float delaySamples) const {
        int intPart = static_cast<int>(delaySamples);
        float frac = delaySamples - static_cast<float>(intPart);

        T a = read(intPart);
        T b = read(intPart + 1);
        return a + frac * (b - a);
    }

    [[nodiscard]] T readHermite(float delaySamples) const {
        int intPart = static_cast<int>(delaySamples);
        float frac = delaySamples - static_cast<float>(intPart);

        T xm1 = read(intPart - 1);
        T x0  = read(intPart);
        T x1  = read(intPart + 1);
        T x2  = read(intPart + 2);

        // Catmull-Rom / Hermite 4-point interpolation
        T c0 = x0;
        T c1 = 0.5f * (x1 - xm1);
        T c2 = xm1 - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
        T c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);

        return ((c3 * frac + c2) * frac + c1) * frac + c0;
    }

    [[nodiscard]] size_t size() const { return Size; }

private:
    static constexpr size_t kMask = Size - 1;
    std::array<T, Size> buffer_{};
    size_t writeHead_ = 0;
};
