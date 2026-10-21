module;

#include <limits>
#include <concepts>
#include <type_traits>

export module pid_fixedpoint;


// to avoid that PID saturates the output, it's neccessary to know the max allowed value
// different approach is needed for base types and classes / structs, 
// because for base types we can use std::numeric_limits<T>::max() 
// but for classes / structs we need to extract the internal type and 
// then use std::numeric_limits<InternalType>::max() to get the max value of the internal type.

// Primary template: Matches primitives (when std::is_class is false)
template <typename T, typename Enable = void>
struct automated_traits {
    static constexpr T max() { 
        return std::numeric_limits<T>::max(); 
    }
};

// Specialization: Automatically matches ANY class or struct
template <typename T>
struct automated_traits<T, typename std::enable_if<std::is_class<T>::value>::type> {
    static constexpr T max() {
        return T(std::numeric_limits<typename T::value_type>::max());
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
    static constexpr  T out_max {automated_traits<T>::max()}; // Use dsp_traits to determine the maximum value for type T
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