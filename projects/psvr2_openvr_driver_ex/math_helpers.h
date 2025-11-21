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
            // gain > 1：使用指数曲线 y = 1 - (1-x)^K
            float linear = s * gain;
            
            // 归一化到[0,1]范围
            float normalized = std::abs(linear) / 127.0f;
            if (normalized > 1.0f) normalized = 1.0f;
            
            // 应用指数曲线：y = 1 - (1-x)^K
            // 在x=0处斜率为K，在x=1处斜率为0
            float curvedNormalized = 1.0f - std::pow(1.0f - normalized, gain);
            
            // 反归一化并确保不超过127
            float curved = curvedNormalized * 127.0f;
            curved = std::min(curved, 127.0f);
            
            // 应用符号
            float result = (linear < 0.0f) ? -curved : curved;
            
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
