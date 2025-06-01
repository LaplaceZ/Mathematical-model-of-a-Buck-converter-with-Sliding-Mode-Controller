#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --------------------------------------------------
// Buck parameters
// --------------------------------------------------
#define VIN  50    // แรงดันอินพุต (V)
#define L    15e-3  // ค่าตัวเหนี่ยวนำ (H) - 100uH
#define C    1000e-6  // ค่าตัวเก็บประจุ (F) - 470uF
#define R    10.0    // ค่าความต้านทานโหลด (Ohm)

// พารามิเตอร์ของตัวควบคุม (จากที่คุณให้มา)
#define a    8.2575
#define b    79.5011
#define m    4918.0 // ใช้เป็น double เพื่อหลีกเลี่ยงการคำนวณจำนวนเต็ม
#define K    49596.0 // ใช้เป็น double เพื่อหลีกเลี่ยงการคำนวณจำนวนเต็ม
#define VREF 20.0    // แรงดันอ้างอิง (V)

// --------------------------------------------------
// SMC parameter
// --------------------------------------------------
#define T_SIM   0.5      // เวลาจำลองทั้งหมด (s) - อาจจะต้องนานขึ้นเพื่อให้ระบบเข้าสู่ steady-state
#define DT      1e-6     // ขั้นตอนเวลา (s) - 1us
#define N_STEPS (int)(T_SIM / DT) // จำนวนขั้นตอนการจำลอง
#define PRINT_INTERVAL 1 // พิมพ์ผลลัพธ์ทุกๆ X ขั้นตอน (เพื่อไม่ให้ไฟล์ใหญ่เกินไป)

// --------------------------------------------------
// State-Space (dy/dt = f(t, y))
// y is [iL, vC]
// f_vector[0] = di_L/dt
// f_vector[1] = dv_C/dt
// --------------------------------------------------
void buck_converter_equations(double iL, double vC, double D_val, double* f_vector) {
    f_vector[0] = (D_val * VIN - vC) / L;       // di_L/dt
    f_vector[1] = (iL / C) - (vC / (R * C)); // dv_C/dt
}

int main() {
    // --------------------------------------------------
    // ค่าเริ่มต้นของตัวแปรสถานะ
    // --------------------------------------------------
    double iL_current = 0.0; // กระแสตัวเหนี่ยวนำเริ่มต้น (A)
    double vC_current = 0.0; // แรงดันตัวเก็บประจุเริ่มต้น (V)

    // --------------------------------------------------
    // Initial value of state variables
    // --------------------------------------------------
    double d_states[2]; // d_states[0] = di_L/dt, d_states[1] = dv_C/dt
    double current_D;   // Duty Cycle 

    // --------------------------------------------------
    // Calculate the coefficients x1, x2, x3
    // --------------------------------------------------
    double x1, x2, x3;
    x1 = (a * R * C + b * L + a * L * K + m * R * L * C * (-K - 1)) / (a * R * C);
    x2 = (b * L * R + a * L * K * R + m * R * L * C) / (a * R * C);
    x3 = (m * R * L * C * (K + 1)) / (a * R * C);

    printf("สัมประสิทธิ์ x1: %.6f, x2: %.6f, x3: %.6f\n", x1, x2, x3);

    // --------------------------------------------------
    //Save the results. (CSV file)
    // --------------------------------------------------
    FILE *fp = fopen("buck_controlled_simulation_results.csv", "w");
    if (fp == NULL) {
        perror("ไม่สามารถเปิดไฟล์ได้");
        return EXIT_FAILURE;
    }
    
    printf("Start buck simulation with controller...\n");
    printf("Parameter: Vin=%.1fV, L=%.1fuH, C=%.1fuF, R=%.1fOhm, Vref=%.1fV\n",VIN, L*1e6, C*1e6, R, VREF);
    // --------------------------------------------------
    // Simulation loop using Euler's method
    // --------------------------------------------------
    for (int i = 0; i < N_STEPS; i++) {
        double current_time = i * DT;

        // 1. Calculate Duty Cycle 'D' at the current time
        // Vdc in your equation should be vC_current (current output voltage)
        // IL in your equation should be iL_current (current inductor current)
        current_D = (vC_current / VIN) * x1 - (iL_current / VIN) * x2 + (VREF / VIN) * x3;

        // **Caution:** Duty Cycle must be between 0 and 1
        // If the calculation gives a value of D outside this range, there may be a problem in the simulation
        // Clamping should be applied
        if (current_D > 1.0) {
            current_D = 1.0;
        } else if (current_D < 0.0) {
            current_D = 0.0;
        }

        // 2. Calculate the derivative at the current time point using the calculated D.
        buck_converter_equations(iL_current, vC_current, current_D, d_states);

        // 3. Update state variables for next step (Euler's Method)
        iL_current += d_states[0] * DT;
        vC_current += d_states[1] * DT;

        // 4. Save the results.
        if (i % PRINT_INTERVAL == 0) {
            fprintf(fp, "%.6f\n", vC_current);
        }
    }
    // Save the final result
    fprintf(fp, "%.6f\n", T_SIM,vC_current);
    fclose(fp);
    printf("Simulation completed! Results saved in buck_controlled_simulation_results.csv\n");
    printf("\nExpected Steady-State Value:\n");
    printf("  Output voltage (Vout): %.3f V \n", VREF);
    printf("  vC: %.3f V\n", vC_current);
    printf("  iL: %.3f A\n", iL_current);

    return EXIT_SUCCESS;
}
