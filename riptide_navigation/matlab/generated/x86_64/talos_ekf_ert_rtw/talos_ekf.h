//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: talos_ekf.h
//
// Code generated for Simulink model 'talos_ekf'.
//
// Model version                  : 1.6
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Fri Oct  2 19:15:52 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_talos_ekf_h_
#define RTW_HEADER_talos_ekf_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "talos_ekf_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model talos_ekf
class talos_ekf final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    real_T P_i[256];                   // '<S1>/DataStoreMemory - P'
    real_T x[16];                      // '<S1>/DataStoreMemory - x'
    boolean_T blockOrdering_p;         // '<S4>/Correct'
    boolean_T blockOrdering_n;         // '<S3>/Correct'
    boolean_T blockOrdering_k;         // '<S2>/Correct'
  };

  // Constant parameters (default storage)
  struct ConstP {
    // Expression: p.InitialCovariance
    //  Referenced by: '<S1>/DataStoreMemory - P'

    real_T DataStoreMemoryP_InitialValue[256];

    // Expression: p.InitialState
    //  Referenced by: '<S1>/DataStoreMemory - x'

    real_T DataStoreMemoryx_InitialValue[16];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    real_T Q[256];                     // '<Root>/Q'
    real_T dt;                         // '<Root>/dt'
    boolean_T enable_imu;              // '<Root>/enable_imu'
    real_T imu_measurement[9];         // '<Root>/imu_measurement'
    real_T R_imu[81];                  // '<Root>/R_imu'
    boolean_T enable_fog;              // '<Root>/enable_fog'
    real_T fog_measurement[3];         // '<Root>/fog_measurement'
    real_T R_fog[9];                   // '<Root>/R_fog'
    boolean_T enable_dvl;              // '<Root>/enable_dvl'
    real_T dvl_measurement[3];         // '<Root>/dvl_measurement'
    real_T R_dvl[9];                   // '<Root>/R_dvl'
    boolean_T enable_depth;            // '<Root>/enable_depth'
    real_T depth_measurement;          // '<Root>/depth_measurement'
    real_T R_depth;                    // '<Root>/R_depth'
    boolean_T enable_reset;            // '<Root>/enable_reset'
    real_T reset_state[16];            // '<Root>/reset_state'
    real_T R_reset[256];               // '<Root>/R_reset'
    real_T dvl_context[6];             // '<Root>/dvl_context'
    real_T imu_context[12];            // '<Root>/imu_context'
    real_T fog_mask[3];                // '<Root>/fog_mask'
    real_T depth_mask;                 // '<Root>/depth_mask'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real_T state[16];                  // '<Root>/state'
    real_T covariance[256];            // '<Root>/covariance'
  };

  // Real-time Model Data Structure
  struct RT_MODEL {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  talos_ekf(talos_ekf const&) = delete;

  // Assignment Operator
  talos_ekf& operator= (talos_ekf const&) & = delete;

  // Move Constructor
  talos_ekf(talos_ekf &&) = delete;

  // Move Assignment Operator
  talos_ekf& operator= (talos_ekf &&) = delete;

  // Real-Time Model get method
  talos_ekf::RT_MODEL * getRTM();

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  talos_ekf();

  // Destructor
  ~talos_ekf();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // private member function(s) for subsystem '<Root>'
  real_T xnrm2(int32_T n, const real_T x[81], int32_T ix0);
  real_T xdotc(int32_T n, const real_T x[81], int32_T ix0, const real_T y[81],
               int32_T iy0);
  void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[81], int32_T iy0);
  real_T xnrm2_l(int32_T n, const real_T x[9], int32_T ix0);
  void xaxpy_n(int32_T n, real_T a, const real_T x[81], int32_T ix0, real_T y[9],
               int32_T iy0);
  void xaxpy_ny(int32_T n, real_T a, const real_T x[9], int32_T ix0, real_T y[81],
                int32_T iy0);
  void xswap(real_T x[81], int32_T ix0, int32_T iy0);
  void xrotg(real_T *a, real_T *b, real_T *c, real_T *s);
  void xrot(real_T x[81], int32_T ix0, int32_T iy0, real_T c, real_T s);
  void svd(const real_T A[81], real_T U[81], real_T s[9], real_T V[81]);
  real_T xnrm2_ln(int32_T n, const real_T x[225], int32_T ix0);
  void xgemv(int32_T m, int32_T n, const real_T A[225], int32_T ia0, const
             real_T x[225], int32_T ix0, real_T y[9]);
  void xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T y[9],
             real_T A[225], int32_T ia0);
  void trisolve(const real_T A[81], real_T B_0[144]);
  void trisolve_b(const real_T A[81], real_T B_2[144]);
  real_T xnrm2_lno(int32_T n, const real_T x[400], int32_T ix0);
  void xgemv_i(int32_T m, int32_T n, const real_T A[400], int32_T ia0, const
               real_T x[400], int32_T ix0, real_T y[16]);
  void xgerc_j(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T y
               [16], real_T A[400], int32_T ia0);
  void EKFCorrector_correctStateAndSqr(real_T x[16], real_T S[256], const real_T
    residue[9], const real_T Pxy[144], const real_T Sy[81], const real_T H[144],
    const real_T Rsqrt[81]);
  real_T xnrm2_h(int32_T n, const real_T x[9], int32_T ix0);
  real_T xdotc_j(int32_T n, const real_T x[9], int32_T ix0, const real_T y[9],
                 int32_T iy0);
  void xaxpy_d(int32_T n, real_T a, int32_T ix0, real_T y[9], int32_T iy0);
  real_T xnrm2_hm(const real_T x[3], int32_T ix0);
  void xaxpy_dp(int32_T n, real_T a, const real_T x[9], int32_T ix0, real_T y[3],
                int32_T iy0);
  void xaxpy_dpe(int32_T n, real_T a, const real_T x[3], int32_T ix0, real_T y[9],
                 int32_T iy0);
  void xswap_o(real_T x[9], int32_T ix0, int32_T iy0);
  void xrot_j(real_T x[9], int32_T ix0, int32_T iy0, real_T c, real_T s);
  void svd_a(const real_T A[9], real_T U[9], real_T s[3], real_T V[9]);
  real_T xnrm2_hmd(int32_T n, const real_T x[57], int32_T ix0);
  void xgemv_h(int32_T m, int32_T n, const real_T A[57], int32_T ia0, const
               real_T x[57], int32_T ix0, real_T y[3]);
  void xgerc_f(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T y
               [3], real_T A[57], int32_T ia0);
  void trisolve_i(const real_T A[9], real_T B_3[48]);
  void trisolve_if(const real_T A[9], real_T B_5[48]);
  real_T xnrm2_hmda(int32_T n, const real_T x[304], int32_T ix0);
  void xgemv_ht(int32_T m, int32_T n, const real_T A[304], int32_T ia0, const
                real_T x[304], int32_T ix0, real_T y[16]);
  void xgerc_f0(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T
                y[16], real_T A[304], int32_T ia0);
  void EKFCorrector_correctStateAndS_n(real_T x[16], real_T S[256], const real_T
    residue[3], const real_T Pxy[48], const real_T Sy[9], const real_T H[48],
    const real_T Rsqrt[9]);
  real_T xnrm2_n(int32_T n, const real_T x[17], int32_T ix0);
  void trisolve_c(real_T A, real_T B_9[16]);
  real_T xnrm2_nd(int32_T n, const real_T x[272], int32_T ix0);
  void xgemv_e(int32_T m, int32_T n, const real_T A[272], int32_T ia0, const
               real_T x[272], int32_T ix0, real_T y[16]);
  void xgerc_n1(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T
                y[16], real_T A[272], int32_T ia0);
  void EKFCorrector_correctStateAndS_h(real_T x[16], real_T S[256], real_T
    residue, const real_T Pxy[16], real_T Sy, const real_T H[16], real_T Rsqrt);
  void EKFCorrector_correct(real_T z, real_T Rs, real_T x[16], real_T S[256],
    real_T varargin_1);
  real_T xnrm2_nn(int32_T n, const real_T x[256], int32_T ix0);
  real_T xdotc_f(int32_T n, const real_T x[256], int32_T ix0, const real_T y[256],
                 int32_T iy0);
  void xaxpy_c(int32_T n, real_T a, int32_T ix0, real_T y[256], int32_T iy0);
  real_T xnrm2_nnh(int32_T n, const real_T x[16], int32_T ix0);
  void xaxpy_ck(int32_T n, real_T a, const real_T x[256], int32_T ix0, real_T y
                [16], int32_T iy0);
  void xaxpy_ckp(int32_T n, real_T a, const real_T x[16], int32_T ix0, real_T y
                 [256], int32_T iy0);
  void xswap_d(real_T x[256], int32_T ix0, int32_T iy0);
  void xrot_g(real_T x[256], int32_T ix0, int32_T iy0, real_T c, real_T s);
  void svd_d(const real_T A[256], real_T U[256], real_T s[16], real_T V[256]);
  real_T xnrm2_nnhl(int32_T n, const real_T x[512], int32_T ix0);
  void xgemv_ez(int32_T m, int32_T n, const real_T A[512], int32_T ia0, const
                real_T x[512], int32_T ix0, real_T y[16]);
  void xgerc_p(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T y
               [16], real_T A[512], int32_T ia0);
  void qrFactor(const real_T A[256], real_T S[256], const real_T Ns[256]);
  void trisolve_j(const real_T A[256], real_T B_b[256]);
  void trisolve_jl(const real_T A[256], real_T B_d[256]);
  real_T xnrm2_d(int32_T n, const real_T x[256], int32_T ix0);
  real_T xdotc_e(int32_T n, const real_T x[256], int32_T ix0, const real_T y[256],
                 int32_T iy0);
  void xaxpy_d2(int32_T n, real_T a, int32_T ix0, real_T y[256], int32_T iy0);
  real_T xnrm2_dz(int32_T n, const real_T x[16], int32_T ix0);
  void xaxpy_d2s(int32_T n, real_T a, const real_T x[256], int32_T ix0, real_T
                 y[16], int32_T iy0);
  void xaxpy_d2sz(int32_T n, real_T a, const real_T x[16], int32_T ix0, real_T
                  y[256], int32_T iy0);
  void svd_n(const real_T A[256], real_T U[256], real_T s[16], real_T V[256]);
  void talos_state_transition(const real_T x[16], real_T dt, real_T next[16]);
  real_T xnrm2_dzn(int32_T n, const real_T x[512], int32_T ix0);
  void xgemv_j(int32_T m, int32_T n, const real_T A[512], int32_T ia0, const
               real_T x[512], int32_T ix0, real_T y[16]);
  void xgerc_e(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T y
               [16], real_T A[512], int32_T ia0);

  // Real-Time Model
  RT_MODEL rtM;
};

// Constant parameters (default storage)
extern const talos_ekf::ConstP rtConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S2>/RegisterSimulinkFcn' : Unused code path elimination
//  Block '<S3>/RegisterSimulinkFcn' : Unused code path elimination
//  Block '<S4>/RegisterSimulinkFcn' : Unused code path elimination
//  Block '<S5>/RegisterSimulinkFcn' : Unused code path elimination
//  Block '<S6>/RegisterSimulinkFcn' : Unused code path elimination
//  Block '<S8>/RegisterSimulinkFcn' : Unused code path elimination
//  Block '<S1>/checkMeasurementFcn1Signals' : Unused code path elimination
//  Block '<S1>/checkMeasurementFcn2Signals' : Unused code path elimination
//  Block '<S1>/checkMeasurementFcn3Signals' : Unused code path elimination
//  Block '<S1>/checkMeasurementFcn4Signals' : Unused code path elimination
//  Block '<S1>/checkMeasurementFcn5Signals' : Unused code path elimination
//  Block '<S1>/checkStateTransitionFcnSignals' : Unused code path elimination
//  Block '<S1>/DataTypeConversion_Enable1' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_Enable2' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_Enable3' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_Enable4' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_Enable5' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_Q' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_R1' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_R2' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_R3' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_R4' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_R5' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_uMeas1' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_uMeas2' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_uMeas3' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_uMeas4' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_uMeas5' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_uState' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_y1' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_y2' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_y3' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_y4' : Eliminate redundant data type conversion
//  Block '<S1>/DataTypeConversion_y5' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'talos_ekf'
//  '<S1>'   : 'talos_ekf/Quaternion_EKF'
//  '<S2>'   : 'talos_ekf/Quaternion_EKF/Correct1'
//  '<S3>'   : 'talos_ekf/Quaternion_EKF/Correct2'
//  '<S4>'   : 'talos_ekf/Quaternion_EKF/Correct3'
//  '<S5>'   : 'talos_ekf/Quaternion_EKF/Correct4'
//  '<S6>'   : 'talos_ekf/Quaternion_EKF/Correct5'
//  '<S7>'   : 'talos_ekf/Quaternion_EKF/Output'
//  '<S8>'   : 'talos_ekf/Quaternion_EKF/Predict'
//  '<S9>'   : 'talos_ekf/Quaternion_EKF/Correct1/Correct'
//  '<S10>'  : 'talos_ekf/Quaternion_EKF/Correct2/Correct'
//  '<S11>'  : 'talos_ekf/Quaternion_EKF/Correct3/Correct'
//  '<S12>'  : 'talos_ekf/Quaternion_EKF/Correct4/Correct'
//  '<S13>'  : 'talos_ekf/Quaternion_EKF/Correct5/Correct'
//  '<S14>'  : 'talos_ekf/Quaternion_EKF/Output/MATLAB Function'
//  '<S15>'  : 'talos_ekf/Quaternion_EKF/Predict/Predict'

#endif                                 // RTW_HEADER_talos_ekf_h_

//
// File trailer for generated code.
//
// [EOF]
//
