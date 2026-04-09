/*
 * cpu_benchmark.c - Impedance Control Pipeline CPU Benchmark
 * Compile: gcc -O2 -lm -o cpu_benchmark cpu_benchmark.c
 * Run: ./cpu_benchmark
 *
 * Measures actual execution time of the same algorithm as FPGA.
 * Results can be scaled to ARM A53 @ 1.333 GHz by IPC ratio.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <windows.h>

#define N_JOINTS 6
#define N_TESTS 10
#define N_ITER 1000000

/* DH Parameters (meters, radians) */
static const double DH_a[6]     = {0.055, 0.200, 0.056, 0.0,    0.0,   0.0};
static const double DH_alpha[6] = {M_PI/2, 0.0, M_PI/2, -M_PI/2, M_PI/2, 0.0};
static const double DH_d[6]     = {0.166, 0.0,  0.0,    0.192,  0.0,   0.055};

/* 4x4 matrix type */
typedef double Mat4[4][4];
typedef double Vec6[6];

/* Test configurations (radians) */
static const double test_q[N_TESTS][6] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
    {0.7854, 0.7854, 0.7854, 0.7854, 0.7854, 0.7854},
    {1.5708, 0.0, 1.5708, 0.0, 1.5708, 0.0},
    {0.5, -0.25, 1.0, -0.5, 0.25, -1.0},
    {3.0, 0.0, 0.0, 0.0, 0.0, 3.0},
    {-0.7854, -1.5708, 0.7854, -0.7854, 1.5708, -0.7854},
    {2.0, -1.0, 1.5, -2.0, 1.0, -0.5},
    {0.0, 1.5708, -1.5708, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 1.5708, 0.0},
};

static inline void mat4_identity(Mat4 m) {
    memset(m, 0, sizeof(Mat4));
    m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0;
}

static inline void mat4_mult(Mat4 C, const Mat4 A, const Mat4 B) {
    int i, j, k;
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++) {
            double s = 0.0;
            for (k = 0; k < 4; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

static void dh_matrix(Mat4 T, double a, double alpha, double d, double theta) {
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    T[0][0] = ct;  T[0][1] = -st*ca; T[0][2] = st*sa;  T[0][3] = a*ct;
    T[1][0] = st;  T[1][1] = ct*ca;  T[1][2] = -ct*sa; T[1][3] = a*st;
    T[2][0] = 0;   T[2][1] = sa;     T[2][2] = ca;      T[2][3] = d;
    T[3][0] = 0;   T[3][1] = 0;      T[3][2] = 0;       T[3][3] = 1;
}

/* Forward Kinematics: returns pose[6] = {x,y,z,rx,ry,rz} */
static void forward_kinematics(const double q[6], double pose[6], Mat4 T_all[7]) {
    int i;
    Mat4 Ti, Tmp;
    mat4_identity(T_all[0]);
    for (i = 0; i < 6; i++) {
        dh_matrix(Ti, DH_a[i], DH_alpha[i], DH_d[i], q[i]);
        mat4_mult(Tmp, T_all[i], Ti);
        memcpy(T_all[i+1], Tmp, sizeof(Mat4));
    }
    pose[0] = T_all[6][0][3];
    pose[1] = T_all[6][1][3];
    pose[2] = T_all[6][2][3];
    pose[5] = atan2(T_all[6][1][0], T_all[6][0][0]);
    pose[4] = atan2(-T_all[6][2][0], sqrt(T_all[6][0][0]*T_all[6][0][0] + T_all[6][1][0]*T_all[6][1][0]));
    pose[3] = atan2(T_all[6][2][1], T_all[6][2][2]);
}

/* Analytical Jacobian 6x6 */
static void analytical_jacobian(const Mat4 T_all[7], double J[6][6]) {
    int i;
    double pe[3] = {T_all[6][0][3], T_all[6][1][3], T_all[6][2][3]};
    for (i = 0; i < 6; i++) {
        double z[3] = {T_all[i][0][2], T_all[i][1][2], T_all[i][2][2]};
        double p[3] = {T_all[i][0][3], T_all[i][1][3], T_all[i][2][3]};
        double d[3] = {pe[0]-p[0], pe[1]-p[1], pe[2]-p[2]};
        /* cross product z × d */
        J[0][i] = z[1]*d[2] - z[2]*d[1];
        J[1][i] = z[2]*d[0] - z[0]*d[2];
        J[2][i] = z[0]*d[1] - z[1]*d[0];
        J[3][i] = z[0];
        J[4][i] = z[1];
        J[5][i] = z[2];
    }
}

/* Velocity estimator (simplified) */
static void velocity_estimator(const double pose[6], const double pose_prev[6],
                                double vel[6], double inv_dt, double alpha) {
    int i;
    for (i = 0; i < 6; i++) {
        double raw = (pose[i] - pose_prev[i]) * inv_dt;
        vel[i] = alpha * raw; /* + (1-alpha)*vel_prev simplified */
    }
}

/* Impedance control */
static void impedance_control(const double xd[6], const double x[6],
                               const double xd_dot[6], const double x_dot[6],
                               const double Ks[6], const double Ds[6],
                               double F[6]) {
    int i;
    for (i = 0; i < 6; i++)
        F[i] = Ks[i] * (xd[i] - x[i]) + Ds[i] * (xd_dot[i] - x_dot[i]);
}

/* J^T force mapping: tau = J^T * F */
static void jt_force_mapping(const double J[6][6], const double F[6], double tau[6]) {
    int i, j;
    for (i = 0; i < 6; i++) {
        double s = 0.0;
        for (j = 0; j < 6; j++)
            s += J[j][i] * F[j];
        tau[i] = s;
    }
}

/* Torque to Iq */
static void torque_to_iq(const double tau[6], double Kt, short iq[6]) {
    int i;
    for (i = 0; i < 6; i++) {
        double v = tau[i] / Kt;
        if (v > 32767) v = 32767;
        if (v < -32767) v = -32767;
        iq[i] = (short)v;
    }
}

/* Full pipeline */
static void full_pipeline(const double q[6]) {
    Mat4 T_all[7];
    double pose[6], J[6][6], vel[6], F[6], tau[6];
    short iq[6];
    double xd[6] = {0.3, 0.0, 0.35, 0.0, 0.0, 0.0};
    double xd_dot[6] = {0};
    double Ks[6] = {100,100,100,10,10,10};
    double Ds[6] = {10,10,10,1,1,1};
    double pose_prev[6] = {0};

    forward_kinematics(q, pose, T_all);
    analytical_jacobian(T_all, J);
    velocity_estimator(pose, pose_prev, vel, 10000.0, 0.3);
    impedance_control(xd, pose, xd_dot, vel, Ks, Ds, F);
    jt_force_mapping(J, F, tau);
    torque_to_iq(tau, 0.1, iq);
}

static double get_time_us(void) {
    static LARGE_INTEGER freq = {0};
    LARGE_INTEGER cnt;
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&cnt);
    return (double)cnt.QuadPart / (double)freq.QuadPart * 1e6;
}

int main(void) {
    int i, t;
    double t0, t1, elapsed;
    double fk_us, jac_us, full_us;

    printf("================================================================\n");
    printf("  CPU Benchmark: 6-DOF Impedance Control Pipeline (C, -O2)\n");
    printf("  FPGA baseline: 395 cycles = 3.95 us @ 100 MHz\n");
    printf("  Iterations: %d\n", N_ITER);
    printf("================================================================\n\n");

    /* Warmup */
    for (i = 0; i < 10000; i++)
        full_pipeline(test_q[0]);

    /* --- Benchmark FK only --- */
    {
        Mat4 T_all[7];
        double pose[6];
        t0 = get_time_us();
        for (i = 0; i < N_ITER; i++)
            for (t = 0; t < N_TESTS; t++)
                forward_kinematics(test_q[t], pose, T_all);
        t1 = get_time_us();
        fk_us = (t1 - t0) / (N_ITER * N_TESTS);
        printf("FK only:          %.3f us/call\n", fk_us);
    }

    /* --- Benchmark Jacobian only --- */
    {
        Mat4 T_all[7];
        double pose[6], J[6][6];
        forward_kinematics(test_q[0], pose, T_all);
        t0 = get_time_us();
        for (i = 0; i < N_ITER; i++)
            for (t = 0; t < N_TESTS; t++)
                analytical_jacobian(T_all, J);
        t1 = get_time_us();
        jac_us = (t1 - t0) / (N_ITER * N_TESTS);
        printf("Jacobian only:    %.3f us/call\n", jac_us);
    }

    /* --- Benchmark full pipeline --- */
    t0 = get_time_us();
    for (i = 0; i < N_ITER; i++)
        for (t = 0; t < N_TESTS; t++)
            full_pipeline(test_q[t]);
    t1 = get_time_us();
    full_us = (t1 - t0) / (N_ITER * N_TESTS);
    printf("Full pipeline:    %.3f us/call\n", full_us);

    printf("\n================================================================\n");
    printf("  COMPARISON\n");
    printf("================================================================\n");
    printf("  %-35s %10s %10s\n", "Platform", "Latency", "vs FPGA");
    printf("  %-35s %10s %10s\n", "-----------------------------------", "----------", "----------");
    printf("  %-35s %10s %10s\n", "FPGA @ 100 MHz (Vivado sim)", "3.95 us", "1.0x");
    printf("  %-35s %8.2f us %8.1fx\n", "This PC (C, -O2, measured)", full_us, full_us / 3.95);

    /* Scale to ARM A53 @ 1.333 GHz */
    /* Rough IPC ratio: x86 OoO ~3-4 IPC, A53 in-order ~0.8-1.2 IPC */
    /* Plus A53 has weaker FPU, slower trig in libm */
    /* Conservative: A53 is ~4-6x slower than modern x86 per-clock, */
    /* but runs at lower freq. Scaling factor ~8-15x total */
    double a53_factor = 10.0; /* conservative estimate */
    double a53_us = full_us * a53_factor;
    printf("  %-35s %8.2f us %8.1fx\n", "ARM A53 @ 1.333 GHz (scaled)", a53_us, a53_us / 3.95);

    /* STM32H7 estimate: ~3-5x slower than A53 (lower freq, in-order M7, no NEON) */
    double stm32_us = a53_us * 3.0;
    printf("  %-35s %8.2f us %8.1fx\n", "STM32H7 @ 480 MHz (scaled)", stm32_us, stm32_us / 3.95);

    printf("================================================================\n");

    /* Save CSV */
    FILE *fp = fopen("D:/papercode/data/csv/cpu_benchmark_c.csv", "w");
    if (fp) {
        fprintf(fp, "platform,latency_us,speedup_vs_fpga\n");
        fprintf(fp, "FPGA_100MHz,3.95,1.0\n");
        fprintf(fp, "ThisPC_C_O2,%.3f,%.1f\n", full_us, full_us/3.95);
        fprintf(fp, "ARM_A53_1333MHz_est,%.2f,%.1f\n", a53_us, a53_us/3.95);
        fprintf(fp, "STM32H7_480MHz_est,%.2f,%.1f\n", stm32_us, stm32_us/3.95);
        fclose(fp);
        printf("\nResults saved to D:/papercode/data/csv/cpu_benchmark_c.csv\n");
    }

    return 0;
}
