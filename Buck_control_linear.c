#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --------------------------------------------------
// กำหนดพารามิเตอร์ของวงจรและตัวควบคุม
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
// พารามิเตอร์การจำลอง
// --------------------------------------------------
#define T_SIM   0.5      // เวลาจำลองทั้งหมด (s) - อาจจะต้องนานขึ้นเพื่อให้ระบบเข้าสู่ steady-state
#define DT      1e-6     // ขั้นตอนเวลา (s) - 1us
#define N_STEPS (int)(T_SIM / DT) // จำนวนขั้นตอนการจำลอง
#define PRINT_INTERVAL 1 // พิมพ์ผลลัพธ์ทุกๆ X ขั้นตอน (เพื่อไม่ให้ไฟล์ใหญ่เกินไป)

// --------------------------------------------------
// ฟังก์ชันที่แทนสมการ State-Space (dy/dt = f(t, y))
// ในที่นี้ y คือ [iL, vC]
// f_vector[0] = di_L/dt
// f_vector[1] = dv_C/dt
// D คือ Duty Cycle ที่คำนวณจากภายนอกฟังก์ชันนี้
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
    // ตัวแปรสำหรับอนุพันธ์และ Duty Cycle
    // --------------------------------------------------
    double d_states[2]; // d_states[0] = di_L/dt, d_states[1] = dv_C/dt
    double current_D;   // ค่า Duty Cycle ที่จะคำนวณในแต่ละขั้นตอน

    // --------------------------------------------------
    // คำนวณสัมประสิทธิ์ x1, x2, x3 ครั้งเดียวเนื่องจากเป็นค่าคงที่
    // --------------------------------------------------
    double x1, x2, x3;
    x1 = (a * R * C + b * L + a * L * K + m * R * L * C * (-K - 1)) / (a * R * C);
    x2 = (b * L * R + a * L * K * R + m * R * L * C) / (a * R * C);
    x3 = (m * R * L * C * (K + 1)) / (a * R * C);

    printf("สัมประสิทธิ์ x1: %.6f, x2: %.6f, x3: %.6f\n", x1, x2, x3);

    // --------------------------------------------------
    // เปิดไฟล์สำหรับบันทึกผลลัพธ์
    // --------------------------------------------------
    FILE *fp = fopen("buck_controlled_simulation_results.csv", "w");
    if (fp == NULL) {
        perror("ไม่สามารถเปิดไฟล์ได้");
        return EXIT_FAILURE;
    }
    
    //fprintf(fp, "Time(s),Inductor_Current(A),Capacitor_Voltage(V),Duty_Cycle(D)\n");

    printf("เริ่มการจำลองวงจรบัคพร้อมตัวควบคุม...\n");
    printf("พารามิเตอร์: Vin=%.1fV, L=%.1fuH, C=%.1fuF, R=%.1fOhm, Vref=%.1fV\n",
           VIN, L*1e6, C*1e6, R, VREF);
    printf("เวลาจำลอง: %.3fs, ขั้นตอนเวลา: %.1fus, จำนวนขั้นตอน: %d\n",
           T_SIM, DT*1e6, N_STEPS);

    // --------------------------------------------------
    // ลูปการจำลองโดยใช้วิธีของออยเลอร์
    // --------------------------------------------------
    for (int i = 0; i < N_STEPS; i++) {
        double current_time = i * DT;

        // 1. คำนวณ Duty Cycle 'D' ณ เวลาปัจจุบัน
        // Vdc ในสมการของคุณน่าจะเป็น vC_current (แรงดันเอาต์พุตปัจจุบัน)
        // IL ในสมการของคุณน่าจะเป็น iL_current (กระแสตัวเหนี่ยวนำปัจจุบัน)
        current_D = (vC_current / VIN) * x1 - (iL_current / VIN) * x2 + (VREF / VIN) * x3;

        // **ข้อควรระวัง:** Duty Cycle ต้องอยู่ระหว่าง 0 ถึง 1
        // หากการคำนวณให้ค่า D นอกช่วงนี้ อาจเกิดปัญหาในการจำลอง
        // ควรมีการจำกัดค่า (Clamping)
        if (current_D > 1.0) {
            current_D = 1.0;
        } else if (current_D < 0.0) {
            current_D = 0.0;
        }

        // 2. คำนวณอนุพันธ์ที่จุดเวลาปัจจุบันโดยใช้ D ที่คำนวณได้
        buck_converter_equations(iL_current, vC_current, current_D, d_states);

        // 3. อัปเดตตัวแปรสถานะสำหรับขั้นตอนถัดไป (Euler's Method)
        iL_current += d_states[0] * DT;
        vC_current += d_states[1] * DT;

        // 4. บันทึกผลลัพธ์
        if (i % PRINT_INTERVAL == 0) {
            fprintf(fp, "%.6f\n", vC_current);
        }
    }

    // บันทึกผลลัพธ์สุดท้าย
    fprintf(fp, "%.6f\n", T_SIM,vC_current);

    fclose(fp);
    printf("การจำลองเสร็จสิ้น! ผลลัพธ์ถูกบันทึกใน buck_controlled_simulation_results.csv\n");

    printf("\nค่า Steady-State ที่คาดหวัง:\n");
    printf("  แรงดันเอาต์พุต (Vout): %.3f V (ควรเข้าใกล้ VREF หากตัวควบคุมทำงานได้ดี)\n", VREF);
    printf("  กระแสตัวเหนี่ยวนำ (IL): %.3f A (VREF / R)\n", VREF / R);
    printf("  ค่าที่ได้จากการจำลอง (สุดท้าย):\n");
    printf("  แรงดันเอาต์พุต (vC): %.3f V\n", vC_current);
    printf("  กระแสตัวเหนี่ยวนำ (iL): %.3f A\n", iL_current);
    printf("  Duty Cycle (สุดท้าย): %.3f\n", current_D);


    return EXIT_SUCCESS;
}