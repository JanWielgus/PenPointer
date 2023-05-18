#include <cstdint>
#include <Fusion/FusionMath.h>


/// @brief Offset class dynamically calculates offset of input value
/// (how much it is displaced from 0 when it is in threshold).
class Offset
{
    float threshold;

    float filterCoefficient; // how fast value converge to 0 when within threshold
    uint32_t samplesToStartConverge; // samples to be within threshold to start converging
    uint32_t convergeSamplesCounter; // increased when within threshold
    FusionVector offset; // current offset

public:

    /// @brief Given delta time (dT_s),
    /// how fast the offset will converge to 0 (convergeRate),
    /// when the value is below (threshold) for at least (convergeDelay_s).
    /// @param dT_s Delta time [seconds].
    /// @param convergeRate How fast value will converge to 0 when within threshold and time elapsed.
    /// @param convergeDelay_s Time that has to elapse when value is withing threshold to start converging to 0 [seconds].
    /// @param threshold Min/max range that value has to be in to start converging.
    Offset(float dT_s, float convergeRate, float convergeDelay_s, float threshold)
    {
        this->threshold = threshold;

        filterCoefficient = convergeRate * dT_s;
        float sampleRate = (1.f / dT_s);
        samplesToStartConverge = convergeDelay_s * sampleRate;
        convergeSamplesCounter = 0;
        offset = {};
    }

    /// @brief Calculate offset and get value with applied offset.
    FusionVector updateValue(FusionVector newValue)
    {
        // Apply current offset
        newValue = FusionVectorSubtract(newValue, offset);

        // Reset timer if value not stationary
        if ((fabs(newValue.axis.x) > threshold) ||
            (fabs(newValue.axis.y) > threshold) ||
            (fabs(newValue.axis.z) > threshold))
        {
            convergeSamplesCounter = 0;
            return newValue;
        }

        // Increment timer while value stationary
        if (convergeSamplesCounter < samplesToStartConverge)
        {
            convergeSamplesCounter++;
            return newValue;
        }

        // Adjust offset if timer has elapsed
        FusionVector offsetChange = FusionVectorMultiplyScalar(newValue, filterCoefficient);
        offset = FusionVectorAdd(offset, offsetChange); // update offset
        return FusionVectorSubtract(newValue, offsetChange); // apply new offset to newValue
    }

    /// @brief Get current offset. 
    FusionVector getOffset()
    {
        return offset;
    }
};
