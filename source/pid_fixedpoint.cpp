module;

#include <limits>
#include <concepts>
#include <type_traits>

export module pid_fixedpoint;

// 1. Primary Template (Fallback / Catch-all)
template <typename T, typename = void>
struct dsp_traits {
    // If T is a standard floating point (like float), use standard numeric_limits
    static constexpr T max() { 
        static_assert(std::is_floating_point_v<T>, "Type not supported by dsp_traits!");
        return std::numeric_limits<T>::max(); 
    }
};

// 2. Advanced Specialization: Automatically matches ANY type that defines a 'storage_type'
template <typename T>
struct dsp_traits<T, std::void_t<typename T::storage_type>> {
    static constexpr T max() {
        // 1. Extract the underlying raw primitive (e.g., int16_t for Q15, int32_t for Q31)
        using RawStorage = typename T::storage_type;
        
        // 2. Get the maximum possible raw integer value (e.g., 0x7FFF for int16_t)
        // 3. Construct and return the matching ARM type wrapped around that value
        return T(std::numeric_limits<RawStorage>::max());
    }
};

export namespace pid_fixedpoint {  

template <typename T>
concept SupportsSelfMultiplication = requires(T a, T b) {
    { a * b } -> std::same_as<T>;
};

template<typename T> class pid {
   public:
    pid(T kp, T ki, T kd) : kp(kp), ki(ki), kd(kd) {
        // Provide a clear message to display if compilation fails
        static_assert(SupportsSelfMultiplication<T>,
            "Error: This class doesn't support template types that can't multiply themselves");        
    }

    // static constexpr  T out_max {std::numeric_limits<typename T::value_type>::max()};
    static constexpr  T out_max {dsp_traits<T>::max()}; // Use dsp_traits to determine the maximum value for type T
    static constexpr  T out_min {0};        // Minimum allowable output

    void init(T kp, T ki, T kd, bool reset = true) {
        kp = kp;
        ki = ki;
        kd = kd;
        
        if (reset) {
            reset_state();
        }
    }
        
    T compute(T setpoint, T feedback) {
        T error;
        T p_term, i_term, d_term;
        T output;
        
        // 1. Calculate the current error (e = r - y)
        error = setpoint - feedback;
        
        // 2. Proportional Term: P = kp * error
        p_term = kp * error;
        
        // 3. Integral Term with anti-windup: I = I_prev + (ki * error)
        integral = integral + ki * error;
          
        // Anti-windup: Clamp the integral accumulator to prevent runway saturation
        if (integral > out_max) {
            integral = out_max;
        } else if (integral < out_min) {
            integral = out_min;
        }
        
        i_term = integral;
            
        // 4. Derivative Term: D = kd * (error - prev_error)
        d_term = kd * (error - previous_error);
           
        // 5. Total Output: Control = P + I + D
        output = p_term + i_term + d_term;
          
        // 6. Save current error for the next execution cycle's derivative calculation
        previous_error = error;
            
        // 7. Final Output Saturation: Clamp the overall control signal
        if (output > out_max) {
            output = out_max;
        } else if (output < out_min) {
            output = out_min;
        }
            
        return output;    
    }
        
private:
    T kp;
    T ki;
    T kd;
    T integral {0};  // Initialize integral term to zero
    T previous_error {0};  // Initialize previous error to zero

    inline void reset_state() {
        integral.v = 0;
        previous_error.v = 0;
    }
};
    
} // namespace pid_fixedpoint