#pragma once

#include <cstdint>
#include <corecrt_math.h>
#include <corecrt_math_defines.h>
#include <cmath>
#include <set>
#include <stdexcept>
#include <array>
#include <vector>
#include <mutex>    // for std::once_flag / std::call_once
#include <atomic>
#include <algorithm>


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
    
    static constexpr int GAIN_LUT_STEPS = 100;     // 可按需调整
    static constexpr float GAIN_MIN = 0.1f;
    static constexpr float GAIN_MAX = 10.0f;
    
    class GainLUT
    {
    public:
        static const int8_t* Get(float gain)
        {
            Init();
    
            // clamp gain range
            if (gain < GAIN_MIN) gain = GAIN_MIN;
            if (gain > GAIN_MAX) gain = GAIN_MAX;
    
            int idx = static_cast<int>(((gain - GAIN_MIN) / (GAIN_MAX - GAIN_MIN)) * (GAIN_LUT_STEPS - 1) + 0.5f);
            return tables[idx].data();
        }
    
    private:
        static void Init()
        {
            static std::once_flag once;
            std::call_once(once, []()
            {
                tables.resize(GAIN_LUT_STEPS);
    
                for (int g = 0; g < GAIN_LUT_STEPS; ++g)
                {
                    float gain = GAIN_MIN + (GAIN_MAX - GAIN_MIN) * (float(g) / (GAIN_LUT_STEPS - 1));
    
                    for (int s = 0; s < 128; ++s)
                    {
                        // 保持与原实现完全一致的数学步骤：
                        // 先把样本乘 gain（linear），再归一化到 [0,1]，然后根据 gain 决定是否使用指数曲线，最后 *127 并 round
                        float sample = static_cast<float>(s); // 绝对值输入 0..127
    
                        float linear = sample * gain;
                        float normalized = linear / 127.0f;
                        if (normalized < 0.0f) normalized = 0.0f;
                        if (normalized > 1.0f) normalized = 1.0f;
    
                        float curvedNormalized;
                        if (gain <= 1.0f)
                        {
                            curvedNormalized = normalized * gain; // 对应原代码在 gain<=1 的行为
                        }
                        else
                        {
                            curvedNormalized = 1.0f - std::pow(1.0f - normalized, gain);
                        }
    
                        float curved = curvedNormalized * 127.0f;
                        if (curved > 127.0f) curved = 127.0f;
    
                        tables[g][s] = static_cast<int8_t>(std::round(curved)); // round 保持语义一致
                    }
                }
            });
        }
    
        static inline std::vector<std::array<int8_t, 128>> tables;
    };
    
    static inline int8_t ApplyCombinedHapticsGain(int8_t sample, float gain)
    {
        if (gain == 1.0f || sample == 0) return sample;
    
        const int8_t* lut = GainLUT::Get(gain);
    
        int absS = std::abs(sample);
        if (absS > 127) absS = 127;
    
        int8_t out = lut[absS];
        return (sample < 0) ? -out : out;
    }

    static int8_t CosineToByte(uint32_t position, double max, double amp, double overdrive)
    {
        double cosResult = Clamp(cos((position / max) * 2 * M_PI) * overdrive, -1.0, 1.0) * amp;
        
        int8_t out = static_cast<int8_t>(cosResult);

        return out;
    }
}
