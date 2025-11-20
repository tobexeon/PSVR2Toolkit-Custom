#pragma once

#include <cstdint>
#include <corecrt_math.h>
#include <corecrt_math_defines.h>
#include <cmath>
#include <set>
#include <stdexcept>

#include <openvr_driver.h>

namespace psvr2_toolkit {
    static double Clamp(double value, double min, double max)
    {
        return value < min ? min : value > max ? max : value;
    }

    static int8_t ClampedAdd(int8_t a, int8_t b)
    {
        // The result is clamped to the range of int8_t
        int16_t result = static_cast<int16_t>(a) + static_cast<int16_t>(b);
        if (result > INT8_MAX)
            return INT8_MAX;
        else if (result < INT8_MIN)
            return INT8_MIN;
        else
            return static_cast<int8_t>(result);
    }
    
    // 把这个放在 math_helpers 里合适的位置（替代或并存）
    static int8_t ApplyCombinedHapticsGain(int8_t sample, float gain) {
        if (gain == 1.0f || sample == 0) return sample;
    
        // 把原始样本转换为 float，范围 -127..127
        float s = static_cast<float>(sample);
    
        if (gain < 1.0f) {
            // 线性减少（简单直接）
            float out = s * gain;
            // 四舍五入并 clamp
            int outInt = static_cast<int>(std::round(out));
            if (outInt > INT8_MAX) outInt = INT8_MAX;
            if (outInt < INT8_MIN) outInt = INT8_MIN;
            return static_cast<int8_t>(outInt);
        } else {
            // gain > 1：先线性放大，然后应用“温和的指数曲线”，最后做软削峰
            // 参数选择：exponent = 1.0 + (gain - 1.0) * 0.5  （这个比例可调）
            float linear = s * gain;
    
            float normalized = std::abs(linear) / 127.0f;
            if (normalized > 1.0f) normalized = 1.0f;
    
            float exponent = 1.0f + (gain - 1.0f) * 0.5f; // 缓和增长，避免指数爆炸
            float curved = std::pow(normalized, exponent) * 127.0f;
    
            // 软削峰：使用 tanh 做柔和限制（比硬截断更平滑）
            float soft = std::tanh(curved / 127.0f) * 127.0f;
    
            float result = soft;
            if (linear < 0.0f) result = -result;
    
            int outInt = static_cast<int>(std::round(result));
            if (outInt > INT8_MAX) outInt = INT8_MAX;
            if (outInt < INT8_MIN) outInt = INT8_MIN;
            return static_cast<int8_t>(outInt);
        }
    }

    static int8_t CosineToByte(uint32_t position, double max, double amp, double overdrive)
    {
        double cosResult = Clamp(cos((position / max) * 2 * M_PI) * overdrive, -1.0, 1.0) * amp;
        
        int8_t out = static_cast<int8_t>(cosResult);

        return out;
    }
}
