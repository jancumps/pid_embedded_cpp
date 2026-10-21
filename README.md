# pid\_embedded\_cpp





C++ PID class, with template support.

You define what data type it uses.  
For embedded controllers without FPU, it can use the ARM DSP++ fixed floating point types.

Also works with floats and doubles, better suited for controllers and MCUs with floating point support.



Example code:



```cpp
using fp_t = arm_cmsis_dsp::Q15;
using pid_controller_t = pid_fixedpoint::pid<fp_t>;

fp_t feedback;
fp_t target_command = fp_t::f(0.66f); // Example target command in Q15 per-unit (0.0 to 1.0)
fp_t output_pid {0}; // Initialize output_pid to 0.0 in Q15 per-unit

import pid_fixedpoint;

pid_controller_t controller(fp_t::f(0.1f), fp_t::f(0.01f), fp_t::f(0.05f)); // Example PID gains for voltage control

```

Example project: [PID controller without floating point for ARM controllers using DSP C++ fixed point API](https://community.element14.com/technologies/embedded/b/blog/posts/pid-controller-without-floating-point-for-arm-controllers-using-dsp-c-fixed-point-api)

