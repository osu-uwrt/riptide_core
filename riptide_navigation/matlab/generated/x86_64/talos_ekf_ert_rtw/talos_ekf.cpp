//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: talos_ekf.cpp
//
// Code generated for Simulink model 'talos_ekf'.
//
// Model version                  : 1.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Fri Oct  2 01:21:03 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "talos_ekf.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>
#include "talos_ekf_private.h"

extern "C"
{

#include "rt_nonfinite.h"

}

int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

// Function for MATLAB Function: '<S2>/Correct'
real_T talos_ekf::xnrm2(int32_T n, const real_T x[64], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S2>/Correct'
real_T talos_ekf::xdotc(int32_T n, const real_T x[64], int32_T ix0, const real_T
  y[64], int32_T iy0)
{
  real_T d;
  int32_T b;
  d = 0.0;
  b = static_cast<uint8_T>(n);
  for (int32_T k{0}; k < b; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[64], int32_T
                      iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
real_T talos_ekf::xnrm2_l(int32_T n, const real_T x[8], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xaxpy_n(int32_T n, real_T a, const real_T x[64], int32_T ix0,
  real_T y[8], int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k{scalarLB}; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xaxpy_ny(int32_T n, real_T a, const real_T x[8], int32_T ix0,
  real_T y[64], int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k{scalarLB}; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xswap(real_T x[64], int32_T ix0, int32_T iy0)
{
  for (int32_T k{0}; k < 8; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    tmp = (iy0 + k) - 1;
    x[temp_tmp] = x[tmp];
    x[tmp] = temp;
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xrotg(real_T *a, real_T *b, real_T *c, real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;
  roe = *b;
  absa = std::abs(*a);
  absb = std::abs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= std::sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xrot(real_T x[64], int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  for (int32_T k{0}; k < 8; k++) {
    real_T temp_tmp;
    real_T temp_tmp_0;
    int32_T temp_tmp_tmp;
    int32_T temp_tmp_tmp_0;
    temp_tmp_tmp = (iy0 + k) - 1;
    temp_tmp = x[temp_tmp_tmp];
    temp_tmp_tmp_0 = (ix0 + k) - 1;
    temp_tmp_0 = x[temp_tmp_tmp_0];
    x[temp_tmp_tmp] = temp_tmp * c - temp_tmp_0 * s;
    x[temp_tmp_tmp_0] = temp_tmp_0 * c + temp_tmp * s;
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::svd(const real_T A[64], real_T U[64], real_T s[8], real_T V[64])
{
  __m128d tmp;
  real_T b_A[64];
  real_T e[8];
  real_T work[8];
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  int32_T exitg1;
  int32_T i;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  int32_T qq_tmp;
  int32_T qq_tmp_tmp;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T apply_transform;
  boolean_T exitg2;
  std::memcpy(&b_A[0], &A[0], sizeof(real_T) << 6U);
  std::memset(&s[0], 0, sizeof(real_T) << 3U);
  std::memset(&e[0], 0, sizeof(real_T) << 3U);
  std::memset(&work[0], 0, sizeof(real_T) << 3U);
  std::memset(&U[0], 0, sizeof(real_T) << 6U);
  std::memset(&V[0], 0, sizeof(real_T) << 6U);
  for (i = 0; i < 7; i++) {
    qp1 = i + 2;
    qq_tmp_tmp = i << 3;
    qq_tmp = qq_tmp_tmp + i;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = xnrm2(8 - i, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      s[i] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qq_tmp - i) + 8;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - i) + 8;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_div_pd(tmp, _mm_set1_pd(s[i])));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] /= s[i];
        }
      }

      b_A[qq_tmp]++;
      s[i] = -s[i];
    } else {
      s[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 9; qp1jj++) {
      qjj = ((qp1jj - 1) << 3) + i;
      if (apply_transform) {
        xaxpy(8 - i, -(xdotc(8 - i, b_A, qq_tmp + 1, b_A, qjj + 1) / b_A[qq_tmp]),
              qq_tmp + 1, b_A, qjj + 1);
      }

      e[qp1jj - 1] = b_A[qjj];
    }

    for (qq = i + 1; qq < 9; qq++) {
      qp1jj = (qq_tmp_tmp + qq) - 1;
      U[qp1jj] = b_A[qp1jj];
    }

    if (i + 1 <= 6) {
      nrm = xnrm2_l(7 - i, e, i + 2);
      if (nrm == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[i + 1] < 0.0) {
          e[i] = -nrm;
        } else {
          e[i] = nrm;
        }

        nrm = e[i];
        if (std::abs(e[i]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[i];
          scalarLB = ((((7 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 9; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          scalarLB = ((((7 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_div_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 9; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 9; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 9; qq++) {
          xaxpy_n(7 - i, e[qq - 1], b_A, (i + ((qq - 1) << 3)) + 2, work, i + 2);
        }

        for (qq = qp1; qq < 9; qq++) {
          xaxpy_ny(7 - i, -e[qq - 1] / e[i + 1], work, i + 2, b_A, (i + ((qq - 1)
                     << 3)) + 2);
        }
      }

      for (qq = qp1; qq < 9; qq++) {
        V[(qq + qq_tmp_tmp) - 1] = e[qq - 1];
      }
    }
  }

  i = 6;
  s[7] = b_A[63];
  e[6] = b_A[62];
  e[7] = 0.0;
  std::memset(&U[56], 0, sizeof(real_T) << 3U);
  U[63] = 1.0;
  for (qp1 = 6; qp1 >= 0; qp1--) {
    qq_tmp = qp1 << 3;
    qq = qq_tmp + qp1;
    if (s[qp1] != 0.0) {
      for (qp1jj = qp1 + 2; qp1jj < 9; qp1jj++) {
        qjj = (((qp1jj - 1) << 3) + qp1) + 1;
        xaxpy(8 - qp1, -(xdotc(8 - qp1, U, qq + 1, U, qjj) / U[qq]), qq + 1, U,
              qjj);
      }

      for (qjj = qp1 + 1; qjj < 9; qjj++) {
        qp1jj = (qq_tmp + qjj) - 1;
        U[qp1jj] = -U[qp1jj];
      }

      U[qq]++;
      for (qjj = 0; qjj < qp1; qjj++) {
        U[qjj + qq_tmp] = 0.0;
      }
    } else {
      std::memset(&U[qq_tmp], 0, sizeof(real_T) << 3U);
      U[qq] = 1.0;
    }
  }

  for (qp1 = 7; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 6) && (e[qp1] != 0.0)) {
      qq = ((qp1 << 3) + qp1) + 2;
      for (qjj = qp1 + 2; qjj < 9; qjj++) {
        qp1jj = (((qjj - 1) << 3) + qp1) + 2;
        xaxpy(7 - qp1, -(xdotc(7 - qp1, V, qq, V, qp1jj) / V[qq - 1]), qq, V,
              qp1jj);
      }
    }

    std::memset(&V[qp1 << 3], 0, sizeof(real_T) << 3U);
    V[qp1 + (qp1 << 3)] = 1.0;
  }

  for (qp1 = 0; qp1 < 8; qp1++) {
    nrm = s[qp1];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      s[qp1] = rt;
      if (qp1 + 1 < 8) {
        e[qp1] /= nrm;
      }

      qq = (qp1 << 3) + 1;
      scalarLB = 8 + qq;
      vectorUB = qq + 6;
      for (qjj = qq; qjj <= vectorUB; qjj += 2) {
        tmp = _mm_loadu_pd(&U[qjj - 1]);
        _mm_storeu_pd(&U[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
      }

      for (qjj = scalarLB; qjj <= qq + 7; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if (qp1 + 1 < 8) {
      smm1 = e[qp1];
      if (smm1 != 0.0) {
        rt = std::abs(smm1);
        nrm = rt / smm1;
        e[qp1] = rt;
        s[qp1 + 1] *= nrm;
        qq = ((qp1 + 1) << 3) + 1;
        scalarLB = 8 + qq;
        vectorUB = qq + 6;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= qq + 7; qjj++) {
          V[qjj - 1] *= nrm;
        }
      }
    }
  }

  qp1 = 0;
  nrm = 0.0;
  for (qq = 0; qq < 8; qq++) {
    nrm = std::fmax(nrm, std::fmax(std::abs(s[qq]), std::abs(e[qq])));
  }

  while ((i + 2 > 0) && (qp1 < 75)) {
    qp1jj = i + 1;
    do {
      exitg1 = 0;
      qq = qp1jj;
      if (qp1jj == 0) {
        exitg1 = 1;
      } else {
        rt = std::abs(e[qp1jj - 1]);
        if (rt <= (std::abs(s[qp1jj - 1]) + std::abs(s[qp1jj])) *
            2.2204460492503131E-16) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else if ((rt <= 1.0020841800044864E-292) || ((qp1 > 20) && (rt <=
                     2.2204460492503131E-16 * nrm))) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg1 == 0);

    if (i + 1 == qp1jj) {
      qp1jj = 4;
    } else {
      qjj = i + 2;
      qq_tmp_tmp = i + 2;
      exitg2 = false;
      while ((!exitg2) && (qq_tmp_tmp >= qp1jj)) {
        qjj = qq_tmp_tmp;
        if (qq_tmp_tmp == qp1jj) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (qq_tmp_tmp < i + 2) {
            rt = std::abs(e[qq_tmp_tmp - 1]);
          }

          if (qq_tmp_tmp > qp1jj + 1) {
            rt += std::abs(e[qq_tmp_tmp - 2]);
          }

          ztest = std::abs(s[qq_tmp_tmp - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            s[qq_tmp_tmp - 1] = 0.0;
            exitg2 = true;
          } else {
            qq_tmp_tmp--;
          }
        }
      }

      if (qjj == qp1jj) {
        qp1jj = 3;
      } else if (i + 2 == qjj) {
        qp1jj = 1;
      } else {
        qp1jj = 2;
        qq = qjj;
      }
    }

    switch (qp1jj) {
     case 1:
      rt = e[i];
      e[i] = 0.0;
      for (qjj = i + 1; qjj >= qq + 1; qjj--) {
        xrotg(&s[qjj - 1], &rt, &ztest, &sqds);
        if (qjj > qq + 1) {
          smm1 = e[qjj - 2];
          rt = -sqds * smm1;
          e[qjj - 2] = smm1 * ztest;
        }

        xrot(V, ((qjj - 1) << 3) + 1, ((i + 1) << 3) + 1, ztest, sqds);
      }
      break;

     case 2:
      rt = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qjj = qq + 1; qjj <= i + 2; qjj++) {
        xrotg(&s[qjj - 1], &rt, &ztest, &sqds);
        smm1 = e[qjj - 1];
        rt = -sqds * smm1;
        e[qjj - 1] = smm1 * ztest;
        xrot(U, ((qjj - 1) << 3) + 1, ((qq - 1) << 3) + 1, ztest, sqds);
      }
      break;

     case 3:
      rt = s[i + 1];
      ztest = std::fmax(std::fmax(std::fmax(std::fmax(std::abs(rt), std::abs(s[i])),
        std::abs(e[i])), std::abs(s[qq])), std::abs(e[qq]));
      rt /= ztest;
      smm1 = s[i] / ztest;
      emm1 = e[i] / ztest;
      sqds = s[qq] / ztest;
      smm1 = ((smm1 + rt) * (smm1 - rt) + emm1 * emm1) / 2.0;
      emm1 *= rt;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = std::sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      rt = (sqds + rt) * (sqds - rt) + shift;
      ztest = e[qq] / ztest * sqds;
      for (qjj = qq + 1; qjj <= i + 1; qjj++) {
        xrotg(&rt, &ztest, &sqds, &smm1);
        if (qjj > qq + 1) {
          e[qjj - 2] = rt;
        }

        emm1 = e[qjj - 1];
        rt = s[qjj - 1];
        e[qjj - 1] = emm1 * sqds - rt * smm1;
        ztest = smm1 * s[qjj];
        s[qjj] *= sqds;
        qq_tmp_tmp = ((qjj - 1) << 3) + 1;
        qq_tmp = (qjj << 3) + 1;
        xrot(V, qq_tmp_tmp, qq_tmp, sqds, smm1);
        s[qjj - 1] = rt * sqds + emm1 * smm1;
        xrotg(&s[qjj - 1], &ztest, &sqds, &smm1);
        ztest = e[qjj - 1];
        rt = ztest * sqds + smm1 * s[qjj];
        s[qjj] = ztest * -smm1 + sqds * s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        xrot(U, qq_tmp_tmp, qq_tmp, sqds, smm1);
      }

      e[i] = rt;
      qp1++;
      break;

     default:
      if (s[qq] < 0.0) {
        s[qq] = -s[qq];
        qp1 = (qq << 3) + 1;
        scalarLB = 8 + qp1;
        vectorUB = qp1 + 6;
        for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
        }

        for (qjj = scalarLB; qjj <= qp1 + 7; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 8) && (s[qq] < s[qp1])) {
        rt = s[qq];
        s[qq] = s[qp1];
        s[qp1] = rt;
        qq_tmp_tmp = (qq << 3) + 1;
        qq_tmp = ((qq + 1) << 3) + 1;
        xswap(V, qq_tmp_tmp, qq_tmp);
        xswap(U, qq_tmp_tmp, qq_tmp);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      i--;
      break;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
real_T talos_ekf::xnrm2_ln(int32_T n, const real_T x[192], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = std::sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = std::sqrt(b * b + 1.0) * a;
  } else if (std::isnan(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xgemv(int32_T m, int32_T n, const real_T A[192], int32_T ia0,
                      const real_T x[192], int32_T ix0, real_T y[8])
{
  if (n != 0) {
    int32_T d;
    std::memset(&y[0], 0, static_cast<uint8_T>(n) * sizeof(real_T));
    d = (n - 1) * 24 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 24) {
      real_T c;
      int32_T b;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x[((ix0 + b) - b_iy) - 1] * A[b - 1];
      }

      b = div_nde_s32_floor(b_iy - ia0, 24);
      y[b] += c;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                      real_T y[8], real_T A[192], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T b;
    int32_T jA;
    jA = ia0;
    b = static_cast<uint8_T>(n);
    for (int32_T j{0}; j < b; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T c;
        temp *= alpha1;
        c = m + jA;
        for (int32_T ijA{jA}; ijA < c; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 24;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::trisolve(const real_T A[64], real_T B_0[128])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = j << 3;
    for (int32_T b_k{0}; b_k < 8; b_k++) {
      real_T B_1;
      int32_T B_tmp;
      int32_T kAcol;
      kAcol = b_k << 3;
      B_tmp = b_k + jBcol;
      B_1 = B_0[B_tmp];
      if (B_1 != 0.0) {
        B_0[B_tmp] = B_1 / A[b_k + kAcol];
        for (int32_T i{b_k + 2}; i < 9; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          B_0[tmp] -= A[(i + kAcol) - 1] * B_0[B_tmp];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::trisolve_b(const real_T A[64], real_T B_2[128])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = j << 3;
    for (int32_T k{7}; k >= 0; k--) {
      real_T tmp;
      int32_T kAcol;
      int32_T tmp_0;
      kAcol = k << 3;
      tmp_0 = k + jBcol;
      tmp = B_2[tmp_0];
      if (tmp != 0.0) {
        B_2[tmp_0] = tmp / A[k + kAcol];
        for (int32_T i{0}; i < k; i++) {
          int32_T tmp_1;
          tmp_1 = i + jBcol;
          B_2[tmp_1] -= A[i + kAcol] * B_2[tmp_0];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
real_T talos_ekf::xnrm2_lno(int32_T n, const real_T x[384], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xgemv_i(int32_T m, int32_T n, const real_T A[384], int32_T ia0,
  const real_T x[384], int32_T ix0, real_T y[16])
{
  if (n != 0) {
    int32_T d;
    std::memset(&y[0], 0, static_cast<uint8_T>(n) * sizeof(real_T));
    d = (n - 1) * 24 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 24) {
      real_T c;
      int32_T b;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x[((ix0 + b) - b_iy) - 1] * A[b - 1];
      }

      b = div_nde_s32_floor(b_iy - ia0, 24);
      y[b] += c;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xgerc_j(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
  real_T y[16], real_T A[384], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T b;
    int32_T jA;
    jA = ia0;
    b = static_cast<uint8_T>(n);
    for (int32_T j{0}; j < b; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T c;
        temp *= alpha1;
        c = m + jA;
        for (int32_T ijA{jA}; ijA < c; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 24;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::EKFCorrector_correctStateAndSqr(real_T x[16], real_T S[256],
  const real_T residue[8], const real_T Pxy[128], const real_T Sy[64], const
  real_T H[128], const real_T Rsqrt[64])
{
  __m128d tmp;
  real_T b_A[384];
  real_T A[256];
  real_T y[256];
  real_T K[128];
  real_T b_C[128];
  real_T Sy_0[64];
  real_T tau[16];
  real_T work[16];
  real_T A_0;
  real_T b_A_0;
  real_T s;
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T ii;
  int32_T j;
  int32_T lastv;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  boolean_T exitg2;
  for (ii = 0; ii < 8; ii++) {
    for (j = 0; j < 16; j++) {
      K[ii + (j << 3)] = Pxy[(ii << 4) + j];
    }
  }

  trisolve(Sy, K);
  std::memcpy(&b_C[0], &K[0], sizeof(real_T) << 7U);
  for (j = 0; j < 8; j++) {
    for (ii = 0; ii < 8; ii++) {
      Sy_0[ii + (j << 3)] = Sy[(ii << 3) + j];
    }
  }

  trisolve_b(Sy_0, b_C);
  for (j = 0; j < 8; j++) {
    for (ii = 0; ii < 16; ii++) {
      K[ii + (j << 4)] = b_C[(ii << 3) + j];
    }
  }

  for (j = 0; j < 16; j++) {
    A_0 = 0.0;
    for (ii = 0; ii < 8; ii++) {
      A_0 += K[(ii << 4) + j] * residue[ii];
    }

    x[j] += A_0;
  }

  for (j = 0; j <= 126; j += 2) {
    tmp = _mm_loadu_pd(&K[j]);
    _mm_storeu_pd(&b_C[j], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii < 16; ii++) {
      A_0 = 0.0;
      for (coffset = 0; coffset < 8; coffset++) {
        A_0 += b_C[(coffset << 4) + ii] * H[(j << 3) + coffset];
      }

      A[ii + (j << 4)] = A_0;
    }
  }

  for (j = 0; j < 16; j++) {
    ii = (j << 4) + j;
    A[ii]++;
  }

  for (j = 0; j < 16; j++) {
    coffset = j << 4;
    for (ii = 0; ii < 16; ii++) {
      aoffset = ii << 4;
      s = 0.0;
      for (lastv = 0; lastv < 16; lastv++) {
        s += A[(lastv << 4) + j] * S[aoffset + lastv];
      }

      y[coffset + ii] = s;
    }
  }

  for (j = 0; j < 8; j++) {
    for (ii = 0; ii < 16; ii++) {
      A_0 = 0.0;
      for (coffset = 0; coffset < 8; coffset++) {
        A_0 += K[(coffset << 4) + ii] * Rsqrt[(j << 3) + coffset];
      }

      b_C[j + (ii << 3)] = A_0;
    }
  }

  for (ii = 0; ii < 16; ii++) {
    std::memcpy(&b_A[ii * 24], &y[ii << 4], sizeof(real_T) << 4U);
    std::memcpy(&b_A[ii * 24 + 16], &b_C[ii << 3], sizeof(real_T) << 3U);
    work[ii] = 0.0;
  }

  for (j = 0; j < 16; j++) {
    ii = j * 24 + j;
    A_0 = b_A[ii];
    lastv = ii + 2;
    tau[j] = 0.0;
    s = xnrm2_lno(23 - j, b_A, ii + 2);
    if (s != 0.0) {
      b_A_0 = b_A[ii];
      s = rt_hypotd_snf(b_A_0, s);
      if (b_A_0 >= 0.0) {
        s = -s;
      }

      if (std::abs(s) < 1.0020841800044864E-292) {
        coffset = 0;
        scalarLB = (ii - j) + 24;
        do {
          coffset++;
          vectorUB = (((((scalarLB - ii) - 1) / 2) << 1) + ii) + 2;
          vectorUB_tmp = vectorUB - 2;
          for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
            tmp = _mm_loadu_pd(&b_A[aoffset - 1]);
            _mm_storeu_pd(&b_A[aoffset - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
            b_A[aoffset - 1] *= 9.9792015476736E+291;
          }

          s *= 9.9792015476736E+291;
          A_0 *= 9.9792015476736E+291;
        } while ((std::abs(s) < 1.0020841800044864E-292) && (coffset < 20));

        s = rt_hypotd_snf(A_0, xnrm2_lno(23 - j, b_A, ii + 2));
        if (A_0 >= 0.0) {
          s = -s;
        }

        tau[j] = (s - A_0) / s;
        A_0 = 1.0 / (A_0 - s);
        for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
          tmp = _mm_loadu_pd(&b_A[aoffset - 1]);
          _mm_storeu_pd(&b_A[aoffset - 1], _mm_mul_pd(tmp, _mm_set1_pd(A_0)));
        }

        for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
          b_A[aoffset - 1] *= A_0;
        }

        for (lastv = 0; lastv < coffset; lastv++) {
          s *= 1.0020841800044864E-292;
        }

        A_0 = s;
      } else {
        tau[j] = (s - b_A_0) / s;
        A_0 = 1.0 / (b_A_0 - s);
        aoffset = (ii - j) + 24;
        scalarLB = (((((aoffset - ii) - 1) / 2) << 1) + ii) + 2;
        vectorUB = scalarLB - 2;
        for (coffset = lastv; coffset <= vectorUB; coffset += 2) {
          tmp = _mm_loadu_pd(&b_A[coffset - 1]);
          _mm_storeu_pd(&b_A[coffset - 1], _mm_mul_pd(tmp, _mm_set1_pd(A_0)));
        }

        for (coffset = scalarLB; coffset <= aoffset; coffset++) {
          b_A[coffset - 1] *= A_0;
        }

        A_0 = s;
      }
    }

    b_A[ii] = A_0;
    if (j + 1 < 16) {
      b_A[ii] = 1.0;
      if (tau[j] != 0.0) {
        lastv = 24 - j;
        coffset = (ii - j) + 23;
        while ((lastv > 0) && (b_A[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }

        coffset = 15 - j;
        exitg2 = false;
        while ((!exitg2) && (coffset > 0)) {
          aoffset = ((coffset - 1) * 24 + ii) + 24;
          scalarLB = aoffset;
          do {
            exitg1 = 0;
            if (scalarLB + 1 <= aoffset + lastv) {
              if (b_A[scalarLB] != 0.0) {
                exitg1 = 1;
              } else {
                scalarLB++;
              }
            } else {
              coffset--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        coffset = 0;
      }

      if (lastv > 0) {
        xgemv_i(lastv, coffset, b_A, ii + 25, b_A, ii + 1, work);
        xgerc_j(lastv, coffset, -tau[j], ii + 1, work, b_A, ii + 25);
      }

      b_A[ii] = A_0;
    }
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii <= j; ii++) {
      A[ii + (j << 4)] = b_A[24 * j + ii];
    }

    for (ii = j + 2; ii < 17; ii++) {
      A[(ii + (j << 4)) - 1] = 0.0;
    }
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii < 16; ii++) {
      S[ii + (j << 4)] = A[(ii << 4) + j];
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
real_T talos_ekf::xnrm2_h(int32_T n, const real_T x[17], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::trisolve_i(real_T A, real_T B_3[16])
{
  for (int32_T j{0}; j < 16; j++) {
    real_T B_4;
    B_4 = B_3[j];
    if (B_4 != 0.0) {
      B_3[j] = B_4 / A;
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
real_T talos_ekf::xnrm2_hm(int32_T n, const real_T x[272], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xgemv_h(int32_T m, int32_T n, const real_T A[272], int32_T ia0,
  const real_T x[272], int32_T ix0, real_T y[16])
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      std::memset(&y[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
    }

    b = (n - 1) * 17 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= b; b_iy += 17) {
      real_T c;
      int32_T d;
      int32_T iyend;
      c = 0.0;
      d = (b_iy + m) - 1;
      for (iyend = b_iy; iyend <= d; iyend++) {
        c += x[((ix0 + iyend) - b_iy) - 1] * A[iyend - 1];
      }

      iyend = div_nde_s32_floor(b_iy - ia0, 17);
      y[iyend] += c;
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xgerc_f(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
  real_T y[16], real_T A[272], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j{0}; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = m + jA;
        for (int32_T ijA{jA}; ijA < b; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 17;
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::EKFCorrector_correctStateAndS_n(real_T x[16], real_T S[256],
  real_T residue, const real_T Pxy[16], real_T Sy, const real_T H[16], real_T
  Rsqrt)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T b_A[272];
  real_T A[256];
  real_T C[16];
  real_T K[16];
  real_T atmp;
  real_T b_A_0;
  real_T s;
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T ii;
  int32_T j;
  int32_T lastv;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  boolean_T exitg2;
  std::memcpy(&C[0], &Pxy[0], sizeof(real_T) << 4U);
  trisolve_i(Sy, C);
  std::memcpy(&K[0], &C[0], sizeof(real_T) << 4U);
  trisolve_i(Sy, K);
  for (j = 0; j <= 14; j += 2) {
    tmp = _mm_loadu_pd(&K[j]);
    tmp_0 = _mm_loadu_pd(&x[j]);
    _mm_storeu_pd(&x[j], _mm_add_pd(_mm_mul_pd(tmp, _mm_set1_pd(residue)), tmp_0));
    _mm_storeu_pd(&C[j], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii <= 14; ii += 2) {
      tmp = _mm_loadu_pd(&C[ii]);
      _mm_storeu_pd(&A[ii + (j << 4)], _mm_mul_pd(tmp, _mm_set1_pd(H[j])));
    }
  }

  for (j = 0; j < 16; j++) {
    ii = (j << 4) + j;
    A[ii]++;
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii < 16; ii++) {
      aoffset = ii << 4;
      s = 0.0;
      for (lastv = 0; lastv < 16; lastv++) {
        s += A[(lastv << 4) + j] * S[aoffset + lastv];
      }

      b_A[ii + 17 * j] = s;
    }

    b_A[17 * j + 16] = K[j] * Rsqrt;
    K[j] = 0.0;
  }

  for (j = 0; j < 16; j++) {
    ii = j * 17 + j;
    atmp = b_A[ii];
    lastv = ii + 2;
    C[j] = 0.0;
    s = xnrm2_hm(16 - j, b_A, ii + 2);
    if (s != 0.0) {
      b_A_0 = b_A[ii];
      s = rt_hypotd_snf(b_A_0, s);
      if (b_A_0 >= 0.0) {
        s = -s;
      }

      if (std::abs(s) < 1.0020841800044864E-292) {
        coffset = 0;
        scalarLB = (ii - j) + 17;
        do {
          coffset++;
          vectorUB = (((((scalarLB - ii) - 1) / 2) << 1) + ii) + 2;
          vectorUB_tmp = vectorUB - 2;
          for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
            tmp = _mm_loadu_pd(&b_A[aoffset - 1]);
            _mm_storeu_pd(&b_A[aoffset - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
            b_A[aoffset - 1] *= 9.9792015476736E+291;
          }

          s *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((std::abs(s) < 1.0020841800044864E-292) && (coffset < 20));

        s = rt_hypotd_snf(atmp, xnrm2_hm(16 - j, b_A, ii + 2));
        if (atmp >= 0.0) {
          s = -s;
        }

        C[j] = (s - atmp) / s;
        atmp = 1.0 / (atmp - s);
        for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
          tmp = _mm_loadu_pd(&b_A[aoffset - 1]);
          _mm_storeu_pd(&b_A[aoffset - 1], _mm_mul_pd(tmp, _mm_set1_pd(atmp)));
        }

        for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
          b_A[aoffset - 1] *= atmp;
        }

        for (lastv = 0; lastv < coffset; lastv++) {
          s *= 1.0020841800044864E-292;
        }

        atmp = s;
      } else {
        C[j] = (s - b_A_0) / s;
        atmp = 1.0 / (b_A_0 - s);
        aoffset = (ii - j) + 17;
        scalarLB = (((((aoffset - ii) - 1) / 2) << 1) + ii) + 2;
        vectorUB = scalarLB - 2;
        for (coffset = lastv; coffset <= vectorUB; coffset += 2) {
          tmp = _mm_loadu_pd(&b_A[coffset - 1]);
          _mm_storeu_pd(&b_A[coffset - 1], _mm_mul_pd(tmp, _mm_set1_pd(atmp)));
        }

        for (coffset = scalarLB; coffset <= aoffset; coffset++) {
          b_A[coffset - 1] *= atmp;
        }

        atmp = s;
      }
    }

    b_A[ii] = atmp;
    if (j + 1 < 16) {
      b_A[ii] = 1.0;
      if (C[j] != 0.0) {
        lastv = 17 - j;
        coffset = (ii - j) + 16;
        while ((lastv > 0) && (b_A[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }

        coffset = 15 - j;
        exitg2 = false;
        while ((!exitg2) && (coffset > 0)) {
          aoffset = ((coffset - 1) * 17 + ii) + 17;
          scalarLB = aoffset;
          do {
            exitg1 = 0;
            if (scalarLB + 1 <= aoffset + lastv) {
              if (b_A[scalarLB] != 0.0) {
                exitg1 = 1;
              } else {
                scalarLB++;
              }
            } else {
              coffset--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        coffset = 0;
      }

      if (lastv > 0) {
        xgemv_h(lastv, coffset, b_A, ii + 18, b_A, ii + 1, K);
        xgerc_f(lastv, coffset, -C[j], ii + 1, K, b_A, ii + 18);
      }

      b_A[ii] = atmp;
    }
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii <= j; ii++) {
      A[ii + (j << 4)] = b_A[17 * j + ii];
    }

    for (ii = j + 2; ii < 17; ii++) {
      A[(ii + (j << 4)) - 1] = 0.0;
    }
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii < 16; ii++) {
      S[ii + (j << 4)] = A[(ii << 4) + j];
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::EKFCorrector_correct(real_T z, real_T Rs, real_T x[16], real_T
  S[256])
{
  __m128d tmp;
  real_T A[17];
  real_T b_x[16];
  real_T c_x[16];
  real_T dHdx[16];
  real_T h;
  real_T x_0;
  int32_T S_tmp;
  int32_T f_k;
  int32_T i;
  int32_T knt;
  for (f_k = 0; f_k < 16; f_k++) {
    h = 1.0E-6 * std::fmax(1.0, std::abs(x[f_k]));
    std::memcpy(&b_x[0], &x[0], sizeof(real_T) << 4U);
    std::memcpy(&c_x[0], &x[0], sizeof(real_T) << 4U);
    x_0 = x[f_k];
    b_x[f_k] = x_0 + h;
    c_x[f_k] = x_0 - h;
    dHdx[f_k] = (b_x[12] - c_x[12]) / (2.0 * h);
  }

  for (f_k = 0; f_k < 16; f_k++) {
    i = f_k << 4;
    h = 0.0;
    for (knt = 0; knt < 16; knt++) {
      h += S[i + knt] * dHdx[knt];
    }

    A[f_k] = h;
  }

  A[16] = Rs;
  x_0 = A[0];
  h = xnrm2_h(16, A, 2);
  if (h != 0.0) {
    h = rt_hypotd_snf(A[0], h);
    if (A[0] >= 0.0) {
      h = -h;
    }

    if (std::abs(h) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (i = 0; i <= 14; i += 2) {
          tmp = _mm_loadu_pd(&A[i + 1]);
          _mm_storeu_pd(&A[i + 1], _mm_mul_pd(tmp, _mm_set1_pd
            (9.9792015476736E+291)));
        }

        h *= 9.9792015476736E+291;
        x_0 *= 9.9792015476736E+291;
      } while ((std::abs(h) < 1.0020841800044864E-292) && (knt < 20));

      h = rt_hypotd_snf(x_0, xnrm2_h(16, A, 2));
      if (x_0 >= 0.0) {
        h = -h;
      }

      for (i = 0; i < knt; i++) {
        h *= 1.0020841800044864E-292;
      }

      x_0 = h;
    } else {
      x_0 = h;
    }
  }

  for (f_k = 0; f_k < 16; f_k++) {
    c_x[f_k] = 0.0;
    for (i = 0; i < 16; i++) {
      h = 0.0;
      for (knt = 0; knt < 16; knt++) {
        S_tmp = knt << 4;
        h += S[S_tmp + f_k] * S[S_tmp + i];
      }

      c_x[f_k] += h * dHdx[i];
    }
  }

  EKFCorrector_correctStateAndS_n(x, S, z - x[12], c_x, x_0, dHdx, Rs);
}

// Function for MATLAB Function: '<S4>/Correct'
real_T talos_ekf::xnrm2_i(int32_T n, const real_T x[9], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S4>/Correct'
real_T talos_ekf::xdotc_m(int32_T n, const real_T x[9], int32_T ix0, const
  real_T y[9], int32_T iy0)
{
  real_T d;
  int32_T b;
  d = 0.0;
  b = static_cast<uint8_T>(n);
  for (int32_T k{0}; k < b; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xaxpy_j(int32_T n, real_T a, int32_T ix0, real_T y[9], int32_T
  iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
real_T talos_ekf::xnrm2_iz(const real_T x[3], int32_T ix0)
{
  real_T scale;
  real_T y;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (int32_T k{ix0}; k <= ix0 + 1; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xaxpy_jn(int32_T n, real_T a, const real_T x[9], int32_T ix0,
  real_T y[3], int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k{scalarLB}; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xaxpy_jnu(int32_T n, real_T a, const real_T x[3], int32_T ix0,
  real_T y[9], int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k{scalarLB}; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xswap_l(real_T x[9], int32_T ix0, int32_T iy0)
{
  real_T temp;
  temp = x[ix0 - 1];
  x[ix0 - 1] = x[iy0 - 1];
  x[iy0 - 1] = temp;
  temp = x[ix0];
  x[ix0] = x[iy0];
  x[iy0] = temp;
  temp = x[ix0 + 1];
  x[ix0 + 1] = x[iy0 + 1];
  x[iy0 + 1] = temp;
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xrot_f(real_T x[9], int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  real_T temp;
  real_T temp_tmp;
  temp = x[iy0 - 1];
  temp_tmp = x[ix0 - 1];
  x[iy0 - 1] = temp * c - temp_tmp * s;
  x[ix0 - 1] = temp_tmp * c + temp * s;
  temp = x[ix0] * c + x[iy0] * s;
  x[iy0] = x[iy0] * c - x[ix0] * s;
  x[ix0] = temp;
  temp = x[iy0 + 1];
  temp_tmp = x[ix0 + 1];
  x[iy0 + 1] = temp * c - temp_tmp * s;
  x[ix0 + 1] = temp_tmp * c + temp * s;
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::svd_k(const real_T A[9], real_T U[9], real_T s[3], real_T V[9])
{
  __m128d tmp;
  real_T b_A[9];
  real_T b_s[3];
  real_T e[3];
  real_T work[3];
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  int32_T exitg1;
  int32_T kase;
  int32_T m;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  int32_T qq_tmp;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T apply_transform;
  boolean_T exitg2;
  b_s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  b_s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  b_s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (kase = 0; kase < 9; kase++) {
    b_A[kase] = A[kase];
    U[kase] = 0.0;
    V[kase] = 0.0;
  }

  for (m = 0; m < 2; m++) {
    qp1 = m + 2;
    qq_tmp = 3 * m + m;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = xnrm2_i(3 - m, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      b_s[m] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qq_tmp - m) + 3;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (kase = qq; kase <= vectorUB; kase += 2) {
          tmp = _mm_loadu_pd(&b_A[kase - 1]);
          _mm_storeu_pd(&b_A[kase - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (kase = scalarLB; kase <= qjj; kase++) {
          b_A[kase - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - m) + 3;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (kase = qq; kase <= vectorUB; kase += 2) {
          tmp = _mm_loadu_pd(&b_A[kase - 1]);
          _mm_storeu_pd(&b_A[kase - 1], _mm_div_pd(tmp, _mm_set1_pd(b_s[m])));
        }

        for (kase = scalarLB; kase <= qjj; kase++) {
          b_A[kase - 1] /= b_s[m];
        }
      }

      b_A[qq_tmp]++;
      b_s[m] = -b_s[m];
    } else {
      b_s[m] = 0.0;
    }

    for (kase = qp1; kase < 4; kase++) {
      qjj = (kase - 1) * 3 + m;
      if (apply_transform) {
        xaxpy_j(3 - m, -(xdotc_m(3 - m, b_A, qq_tmp + 1, b_A, qjj + 1) /
                         b_A[qq_tmp]), qq_tmp + 1, b_A, qjj + 1);
      }

      e[kase - 1] = b_A[qjj];
    }

    for (qq = m + 1; qq < 4; qq++) {
      kase = (3 * m + qq) - 1;
      U[kase] = b_A[kase];
    }

    if (m + 1 <= 1) {
      nrm = xnrm2_iz(e, 2);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }

        nrm = e[0];
        if (std::abs(e[0]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[0];
          scalarLB = ((((2 - m) / 2) << 1) + m) + 2;
          vectorUB = scalarLB - 2;
          for (qq = qp1; qq <= vectorUB; qq += 2) {
            tmp = _mm_loadu_pd(&e[qq - 1]);
            _mm_storeu_pd(&e[qq - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qq = scalarLB; qq < 4; qq++) {
            e[qq - 1] *= nrm;
          }
        } else {
          scalarLB = ((((2 - m) / 2) << 1) + m) + 2;
          vectorUB = scalarLB - 2;
          for (qq = qp1; qq <= vectorUB; qq += 2) {
            tmp = _mm_loadu_pd(&e[qq - 1]);
            _mm_storeu_pd(&e[qq - 1], _mm_div_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qq = scalarLB; qq < 4; qq++) {
            e[qq - 1] /= nrm;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (qq = qp1; qq < 4; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 4; qq++) {
          xaxpy_jn(2, e[qq - 1], b_A, 3 * (qq - 1) + 2, work, 2);
        }

        for (qq = qp1; qq < 4; qq++) {
          xaxpy_jnu(2, -e[qq - 1] / e[1], work, 2, b_A, 3 * (qq - 1) + 2);
        }
      }

      for (qq = qp1; qq < 4; qq++) {
        V[qq - 1] = e[qq - 1];
      }
    }
  }

  m = 1;
  b_s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (qp1 = 1; qp1 >= 0; qp1--) {
    qq = 3 * qp1 + qp1;
    if (b_s[qp1] != 0.0) {
      for (kase = qp1 + 2; kase < 4; kase++) {
        qjj = ((kase - 1) * 3 + qp1) + 1;
        xaxpy_j(3 - qp1, -(xdotc_m(3 - qp1, U, qq + 1, U, qjj) / U[qq]), qq + 1,
                U, qjj);
      }

      for (qjj = qp1 + 1; qjj < 4; qjj++) {
        kase = (3 * qp1 + qjj) - 1;
        U[kase] = -U[kase];
      }

      U[qq]++;
      if (qp1 - 1 >= 0) {
        U[3 * qp1] = 0.0;
      }
    } else {
      U[3 * qp1] = 0.0;
      U[3 * qp1 + 1] = 0.0;
      U[3 * qp1 + 2] = 0.0;
      U[qq] = 1.0;
    }
  }

  for (qp1 = 2; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 1) && (e[0] != 0.0)) {
      xaxpy_j(2, -(xdotc_m(2, V, 2, V, 5) / V[1]), 2, V, 5);
      xaxpy_j(2, -(xdotc_m(2, V, 2, V, 8) / V[1]), 2, V, 8);
    }

    V[3 * qp1] = 0.0;
    V[3 * qp1 + 1] = 0.0;
    V[3 * qp1 + 2] = 0.0;
    V[qp1 + 3 * qp1] = 1.0;
  }

  for (qp1 = 0; qp1 < 3; qp1++) {
    nrm = b_s[qp1];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      b_s[qp1] = rt;
      if (qp1 + 1 < 3) {
        e[qp1] /= nrm;
      }

      qq = 3 * qp1 + 1;
      scalarLB = 2 + qq;
      for (qjj = qq; qjj <= qq; qjj += 2) {
        tmp = _mm_loadu_pd(&U[qjj - 1]);
        _mm_storeu_pd(&U[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
      }

      for (qjj = scalarLB; qjj <= qq + 2; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if (qp1 + 1 < 3) {
      smm1 = e[qp1];
      if (smm1 != 0.0) {
        rt = std::abs(smm1);
        nrm = rt / smm1;
        e[qp1] = rt;
        b_s[qp1 + 1] *= nrm;
        qq = (qp1 + 1) * 3 + 1;
        scalarLB = 2 + qq;
        for (qjj = qq; qjj <= qq; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= qq + 2; qjj++) {
          V[qjj - 1] *= nrm;
        }
      }
    }
  }

  qp1 = 0;
  nrm = std::fmax(std::fmax(std::fmax(0.0, std::fmax(std::abs(b_s[0]), std::abs
    (e[0]))), std::fmax(std::abs(b_s[1]), std::abs(e[1]))), std::fmax(std::abs
    (b_s[2]), std::abs(e[2])));
  while ((m + 2 > 0) && (qp1 < 75)) {
    kase = m + 1;
    do {
      exitg1 = 0;
      qq = kase;
      if (kase == 0) {
        exitg1 = 1;
      } else {
        rt = std::abs(e[kase - 1]);
        if (rt <= (std::abs(b_s[kase - 1]) + std::abs(b_s[kase])) *
            2.2204460492503131E-16) {
          e[kase - 1] = 0.0;
          exitg1 = 1;
        } else if ((rt <= 1.0020841800044864E-292) || ((qp1 > 20) && (rt <=
                     2.2204460492503131E-16 * nrm))) {
          e[kase - 1] = 0.0;
          exitg1 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg1 == 0);

    if (m + 1 == kase) {
      kase = 4;
    } else {
      qjj = m + 2;
      qq_tmp = m + 2;
      exitg2 = false;
      while ((!exitg2) && (qq_tmp >= kase)) {
        qjj = qq_tmp;
        if (qq_tmp == kase) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (qq_tmp < m + 2) {
            rt = std::abs(e[qq_tmp - 1]);
          }

          if (qq_tmp > kase + 1) {
            rt += std::abs(e[qq_tmp - 2]);
          }

          ztest = std::abs(b_s[qq_tmp - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            b_s[qq_tmp - 1] = 0.0;
            exitg2 = true;
          } else {
            qq_tmp--;
          }
        }
      }

      if (qjj == kase) {
        kase = 3;
      } else if (m + 2 == qjj) {
        kase = 1;
      } else {
        kase = 2;
        qq = qjj;
      }
    }

    switch (kase) {
     case 1:
      rt = e[m];
      e[m] = 0.0;
      for (qjj = m + 1; qjj >= qq + 1; qjj--) {
        xrotg(&b_s[qjj - 1], &rt, &ztest, &sqds);
        if (qjj > qq + 1) {
          rt = -sqds * e[0];
          e[0] *= ztest;
        }

        xrot_f(V, 3 * (qjj - 1) + 1, 3 * (m + 1) + 1, ztest, sqds);
      }
      break;

     case 2:
      rt = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qjj = qq + 1; qjj <= m + 2; qjj++) {
        xrotg(&b_s[qjj - 1], &rt, &ztest, &sqds);
        smm1 = e[qjj - 1];
        rt = -sqds * smm1;
        e[qjj - 1] = smm1 * ztest;
        xrot_f(U, 3 * (qjj - 1) + 1, 3 * (qq - 1) + 1, ztest, sqds);
      }
      break;

     case 3:
      rt = b_s[m + 1];
      ztest = std::fmax(std::fmax(std::fmax(std::fmax(std::abs(rt), std::abs
        (b_s[m])), std::abs(e[m])), std::abs(b_s[qq])), std::abs(e[qq]));
      rt /= ztest;
      smm1 = b_s[m] / ztest;
      emm1 = e[m] / ztest;
      sqds = b_s[qq] / ztest;
      smm1 = ((smm1 + rt) * (smm1 - rt) + emm1 * emm1) / 2.0;
      emm1 *= rt;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = std::sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      rt = (sqds + rt) * (sqds - rt) + shift;
      ztest = e[qq] / ztest * sqds;
      for (qjj = qq + 1; qjj <= m + 1; qjj++) {
        xrotg(&rt, &ztest, &sqds, &smm1);
        if (qjj > qq + 1) {
          e[0] = rt;
        }

        emm1 = e[qjj - 1];
        rt = b_s[qjj - 1];
        e[qjj - 1] = emm1 * sqds - rt * smm1;
        ztest = smm1 * b_s[qjj];
        b_s[qjj] *= sqds;
        kase = (qjj - 1) * 3 + 1;
        qq_tmp = 3 * qjj + 1;
        xrot_f(V, kase, qq_tmp, sqds, smm1);
        b_s[qjj - 1] = rt * sqds + emm1 * smm1;
        xrotg(&b_s[qjj - 1], &ztest, &sqds, &smm1);
        emm1 = e[qjj - 1];
        rt = emm1 * sqds + smm1 * b_s[qjj];
        b_s[qjj] = emm1 * -smm1 + sqds * b_s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        xrot_f(U, kase, qq_tmp, sqds, smm1);
      }

      e[m] = rt;
      qp1++;
      break;

     default:
      if (b_s[qq] < 0.0) {
        b_s[qq] = -b_s[qq];
        qp1 = 3 * qq + 1;
        scalarLB = 2 + qp1;
        for (qjj = qp1; qjj <= qp1; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
        }

        for (qjj = scalarLB; qjj <= qp1 + 2; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 3) && (b_s[qq] < b_s[qp1])) {
        rt = b_s[qq];
        b_s[qq] = b_s[qp1];
        b_s[qp1] = rt;
        kase = 3 * qq + 1;
        qq_tmp = (qq + 1) * 3 + 1;
        xswap_l(V, kase, qq_tmp);
        xswap_l(U, kase, qq_tmp);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      m--;
      break;
    }
  }

  s[0] = b_s[0];
  s[1] = b_s[1];
  s[2] = b_s[2];
}

// Function for MATLAB Function: '<S4>/Correct'
real_T talos_ekf::xnrm2_izk(int32_T n, const real_T x[57], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xgemv_a(int32_T m, int32_T n, const real_T A[57], int32_T ia0,
  const real_T x[57], int32_T ix0, real_T y[3])
{
  if (n != 0) {
    int32_T d;
    std::memset(&y[0], 0, static_cast<uint8_T>(n) * sizeof(real_T));
    d = (n - 1) * 19 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 19) {
      real_T c;
      int32_T b;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x[((ix0 + b) - b_iy) - 1] * A[b - 1];
      }

      b = div_nde_s32_floor(b_iy - ia0, 19);
      y[b] += c;
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xgerc_n(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
  real_T y[3], real_T A[57], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T b;
    int32_T jA;
    jA = ia0;
    b = static_cast<uint8_T>(n);
    for (int32_T j{0}; j < b; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T c;
        temp *= alpha1;
        c = m + jA;
        for (int32_T ijA{jA}; ijA < c; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 19;
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::trisolve_f(const real_T A[9], real_T B_5[48])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = 3 * j;
    for (int32_T b_k{0}; b_k < 3; b_k++) {
      real_T B_6;
      int32_T B_tmp;
      int32_T kAcol;
      kAcol = 3 * b_k;
      B_tmp = b_k + jBcol;
      B_6 = B_5[B_tmp];
      if (B_6 != 0.0) {
        B_5[B_tmp] = B_6 / A[b_k + kAcol];
        for (int32_T i{b_k + 2}; i < 4; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          B_5[tmp] -= A[(i + kAcol) - 1] * B_5[B_tmp];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::trisolve_fc(const real_T A[9], real_T B_7[48])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = 3 * j;
    for (int32_T k{2}; k >= 0; k--) {
      real_T tmp;
      int32_T kAcol;
      int32_T tmp_0;
      kAcol = 3 * k;
      tmp_0 = k + jBcol;
      tmp = B_7[tmp_0];
      if (tmp != 0.0) {
        B_7[tmp_0] = tmp / A[k + kAcol];
        for (int32_T i{0}; i < k; i++) {
          int32_T tmp_1;
          tmp_1 = i + jBcol;
          B_7[tmp_1] -= A[i + kAcol] * B_7[tmp_0];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
real_T talos_ekf::xnrm2_izk3(int32_T n, const real_T x[304], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xgemv_a3(int32_T m, int32_T n, const real_T A[304], int32_T ia0,
  const real_T x[304], int32_T ix0, real_T y[16])
{
  if (n != 0) {
    int32_T d;
    std::memset(&y[0], 0, static_cast<uint8_T>(n) * sizeof(real_T));
    d = (n - 1) * 19 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 19) {
      real_T c;
      int32_T b;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x[((ix0 + b) - b_iy) - 1] * A[b - 1];
      }

      b = div_nde_s32_floor(b_iy - ia0, 19);
      y[b] += c;
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::xgerc_nv(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
  real_T y[16], real_T A[304], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T b;
    int32_T jA;
    jA = ia0;
    b = static_cast<uint8_T>(n);
    for (int32_T j{0}; j < b; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T c;
        temp *= alpha1;
        c = m + jA;
        for (int32_T ijA{jA}; ijA < c; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 19;
    }
  }
}

// Function for MATLAB Function: '<S4>/Correct'
void talos_ekf::EKFCorrector_correctStateAndS_l(real_T x[16], real_T S[256],
  const real_T residue[3], const real_T Pxy[48], const real_T Sy[9], const
  real_T H[48], const real_T Rsqrt[9])
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  real_T b_A[304];
  real_T A[256];
  real_T y[256];
  real_T K[48];
  real_T b_C[48];
  real_T tau[16];
  real_T work[16];
  real_T Sy_0[9];
  real_T K_0;
  real_T residue_0;
  real_T residue_1;
  real_T s;
  real_T tmp;
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T ii;
  int32_T j;
  int32_T lastv;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  boolean_T exitg2;
  for (j = 0; j < 16; j++) {
    K[3 * j] = Pxy[j];
    K[3 * j + 1] = Pxy[j + 16];
    K[3 * j + 2] = Pxy[j + 32];
  }

  trisolve_f(Sy, K);
  for (j = 0; j < 16; j++) {
    b_C[3 * j] = K[3 * j];
    coffset = 3 * j + 1;
    b_C[coffset] = K[coffset];
    coffset = 3 * j + 2;
    b_C[coffset] = K[coffset];
  }

  for (coffset = 0; coffset < 3; coffset++) {
    Sy_0[3 * coffset] = Sy[coffset];
    Sy_0[3 * coffset + 1] = Sy[coffset + 3];
    Sy_0[3 * coffset + 2] = Sy[coffset + 6];
  }

  trisolve_fc(Sy_0, b_C);
  residue_0 = residue[0];
  s = residue[1];
  residue_1 = residue[2];
  for (coffset = 0; coffset < 16; coffset++) {
    K_0 = b_C[3 * coffset];
    K[coffset] = K_0;
    tmp = K_0 * residue_0;
    K_0 = b_C[3 * coffset + 1];
    K[coffset + 16] = K_0;
    tmp += K_0 * s;
    K_0 = b_C[3 * coffset + 2];
    K[coffset + 32] = K_0;
    x[coffset] += K_0 * residue_1 + tmp;
  }

  for (coffset = 0; coffset <= 46; coffset += 2) {
    tmp_2 = _mm_loadu_pd(&K[coffset]);
    _mm_storeu_pd(&b_C[coffset], _mm_mul_pd(tmp_2, _mm_set1_pd(-1.0)));
  }

  for (coffset = 0; coffset < 16; coffset++) {
    K_0 = H[3 * coffset + 1];
    residue_0 = H[3 * coffset];
    s = H[3 * coffset + 2];
    for (j = 0; j <= 14; j += 2) {
      tmp_2 = _mm_loadu_pd(&b_C[j + 16]);
      tmp_0 = _mm_loadu_pd(&b_C[j]);
      tmp_1 = _mm_loadu_pd(&b_C[j + 32]);
      _mm_storeu_pd(&A[j + (coffset << 4)], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(K_0), tmp_2), _mm_mul_pd(_mm_set1_pd(residue_0), tmp_0)),
        _mm_mul_pd(_mm_set1_pd(s), tmp_1)));
    }
  }

  for (j = 0; j < 16; j++) {
    coffset = (j << 4) + j;
    A[coffset]++;
  }

  for (j = 0; j < 16; j++) {
    coffset = j << 4;
    for (ii = 0; ii < 16; ii++) {
      aoffset = ii << 4;
      s = 0.0;
      for (lastv = 0; lastv < 16; lastv++) {
        s += A[(lastv << 4) + j] * S[aoffset + lastv];
      }

      y[coffset + ii] = s;
    }

    K_0 = K[j + 16];
    residue_0 = K[j];
    s = K[j + 32];
    for (coffset = 0; coffset < 3; coffset++) {
      b_C[coffset + 3 * j] = (Rsqrt[3 * coffset + 1] * K_0 + Rsqrt[3 * coffset] *
        residue_0) + Rsqrt[3 * coffset + 2] * s;
    }
  }

  for (j = 0; j < 16; j++) {
    std::memcpy(&b_A[j * 19], &y[j << 4], sizeof(real_T) << 4U);
    b_A[19 * j + 16] = b_C[3 * j];
    b_A[19 * j + 17] = b_C[3 * j + 1];
    b_A[19 * j + 18] = b_C[3 * j + 2];
    work[j] = 0.0;
  }

  for (j = 0; j < 16; j++) {
    ii = j * 19 + j;
    K_0 = b_A[ii];
    lastv = ii + 2;
    tau[j] = 0.0;
    s = xnrm2_izk3(18 - j, b_A, ii + 2);
    if (s != 0.0) {
      residue_0 = b_A[ii];
      s = rt_hypotd_snf(residue_0, s);
      if (residue_0 >= 0.0) {
        s = -s;
      }

      if (std::abs(s) < 1.0020841800044864E-292) {
        coffset = 0;
        scalarLB = (ii - j) + 19;
        do {
          coffset++;
          vectorUB = (((((scalarLB - ii) - 1) / 2) << 1) + ii) + 2;
          vectorUB_tmp = vectorUB - 2;
          for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
            tmp_2 = _mm_loadu_pd(&b_A[aoffset - 1]);
            _mm_storeu_pd(&b_A[aoffset - 1], _mm_mul_pd(tmp_2, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
            b_A[aoffset - 1] *= 9.9792015476736E+291;
          }

          s *= 9.9792015476736E+291;
          K_0 *= 9.9792015476736E+291;
        } while ((std::abs(s) < 1.0020841800044864E-292) && (coffset < 20));

        s = rt_hypotd_snf(K_0, xnrm2_izk3(18 - j, b_A, ii + 2));
        if (K_0 >= 0.0) {
          s = -s;
        }

        tau[j] = (s - K_0) / s;
        K_0 = 1.0 / (K_0 - s);
        for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
          tmp_2 = _mm_loadu_pd(&b_A[aoffset - 1]);
          _mm_storeu_pd(&b_A[aoffset - 1], _mm_mul_pd(tmp_2, _mm_set1_pd(K_0)));
        }

        for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
          b_A[aoffset - 1] *= K_0;
        }

        for (lastv = 0; lastv < coffset; lastv++) {
          s *= 1.0020841800044864E-292;
        }

        K_0 = s;
      } else {
        tau[j] = (s - residue_0) / s;
        K_0 = 1.0 / (residue_0 - s);
        aoffset = (ii - j) + 19;
        scalarLB = (((((aoffset - ii) - 1) / 2) << 1) + ii) + 2;
        vectorUB = scalarLB - 2;
        for (coffset = lastv; coffset <= vectorUB; coffset += 2) {
          tmp_2 = _mm_loadu_pd(&b_A[coffset - 1]);
          _mm_storeu_pd(&b_A[coffset - 1], _mm_mul_pd(tmp_2, _mm_set1_pd(K_0)));
        }

        for (coffset = scalarLB; coffset <= aoffset; coffset++) {
          b_A[coffset - 1] *= K_0;
        }

        K_0 = s;
      }
    }

    b_A[ii] = K_0;
    if (j + 1 < 16) {
      b_A[ii] = 1.0;
      if (tau[j] != 0.0) {
        lastv = 19 - j;
        coffset = (ii - j) + 18;
        while ((lastv > 0) && (b_A[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }

        coffset = 15 - j;
        exitg2 = false;
        while ((!exitg2) && (coffset > 0)) {
          aoffset = ((coffset - 1) * 19 + ii) + 19;
          scalarLB = aoffset;
          do {
            exitg1 = 0;
            if (scalarLB + 1 <= aoffset + lastv) {
              if (b_A[scalarLB] != 0.0) {
                exitg1 = 1;
              } else {
                scalarLB++;
              }
            } else {
              coffset--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        coffset = 0;
      }

      if (lastv > 0) {
        xgemv_a3(lastv, coffset, b_A, ii + 20, b_A, ii + 1, work);
        xgerc_nv(lastv, coffset, -tau[j], ii + 1, work, b_A, ii + 20);
      }

      b_A[ii] = K_0;
    }
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii <= j; ii++) {
      A[ii + (j << 4)] = b_A[19 * j + ii];
    }

    for (ii = j + 2; ii < 17; ii++) {
      A[(ii + (j << 4)) - 1] = 0.0;
    }
  }

  for (coffset = 0; coffset < 16; coffset++) {
    for (j = 0; j < 16; j++) {
      S[j + (coffset << 4)] = A[(j << 4) + coffset];
    }
  }
}

// Function for MATLAB Function: '<S5>/Correct'
void talos_ekf::EKFCorrector_correct_p(real_T z, real_T Rs, real_T x[16], real_T
  S[256])
{
  __m128d tmp;
  real_T A[17];
  real_T b_x[16];
  real_T c_x[16];
  real_T dHdx[16];
  real_T h;
  real_T x_0;
  int32_T S_tmp;
  int32_T f_k;
  int32_T i;
  int32_T knt;
  for (f_k = 0; f_k < 16; f_k++) {
    h = 1.0E-6 * std::fmax(1.0, std::abs(x[f_k]));
    std::memcpy(&b_x[0], &x[0], sizeof(real_T) << 4U);
    std::memcpy(&c_x[0], &x[0], sizeof(real_T) << 4U);
    x_0 = x[f_k];
    b_x[f_k] = x_0 + h;
    c_x[f_k] = x_0 - h;
    dHdx[f_k] = (b_x[2] - c_x[2]) / (2.0 * h);
  }

  for (f_k = 0; f_k < 16; f_k++) {
    i = f_k << 4;
    h = 0.0;
    for (knt = 0; knt < 16; knt++) {
      h += S[i + knt] * dHdx[knt];
    }

    A[f_k] = h;
  }

  A[16] = Rs;
  x_0 = A[0];
  h = xnrm2_h(16, A, 2);
  if (h != 0.0) {
    h = rt_hypotd_snf(A[0], h);
    if (A[0] >= 0.0) {
      h = -h;
    }

    if (std::abs(h) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (i = 0; i <= 14; i += 2) {
          tmp = _mm_loadu_pd(&A[i + 1]);
          _mm_storeu_pd(&A[i + 1], _mm_mul_pd(tmp, _mm_set1_pd
            (9.9792015476736E+291)));
        }

        h *= 9.9792015476736E+291;
        x_0 *= 9.9792015476736E+291;
      } while ((std::abs(h) < 1.0020841800044864E-292) && (knt < 20));

      h = rt_hypotd_snf(x_0, xnrm2_h(16, A, 2));
      if (x_0 >= 0.0) {
        h = -h;
      }

      for (i = 0; i < knt; i++) {
        h *= 1.0020841800044864E-292;
      }

      x_0 = h;
    } else {
      x_0 = h;
    }
  }

  for (f_k = 0; f_k < 16; f_k++) {
    c_x[f_k] = 0.0;
    for (i = 0; i < 16; i++) {
      h = 0.0;
      for (knt = 0; knt < 16; knt++) {
        S_tmp = knt << 4;
        h += S[S_tmp + f_k] * S[S_tmp + i];
      }

      c_x[f_k] += h * dHdx[i];
    }
  }

  EKFCorrector_correctStateAndS_n(x, S, z - x[2], c_x, x_0, dHdx, Rs);
}

// Function for MATLAB Function: '<S6>/Correct'
real_T talos_ekf::xnrm2_nn(int32_T n, const real_T x[256], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S6>/Correct'
real_T talos_ekf::xdotc_f(int32_T n, const real_T x[256], int32_T ix0, const
  real_T y[256], int32_T iy0)
{
  real_T d;
  int32_T b;
  d = 0.0;
  b = static_cast<uint8_T>(n);
  for (int32_T k{0}; k < b; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::xaxpy_c(int32_T n, real_T a, int32_T ix0, real_T y[256], int32_T
  iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
real_T talos_ekf::xnrm2_nnh(int32_T n, const real_T x[16], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::xaxpy_ck(int32_T n, real_T a, const real_T x[256], int32_T ix0,
  real_T y[16], int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k{scalarLB}; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::xaxpy_ckp(int32_T n, real_T a, const real_T x[16], int32_T ix0,
  real_T y[256], int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k{scalarLB}; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::xswap_d(real_T x[256], int32_T ix0, int32_T iy0)
{
  for (int32_T k{0}; k < 16; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    tmp = (iy0 + k) - 1;
    x[temp_tmp] = x[tmp];
    x[tmp] = temp;
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::xrot_g(real_T x[256], int32_T ix0, int32_T iy0, real_T c, real_T
  s)
{
  for (int32_T k{0}; k < 16; k++) {
    real_T temp_tmp;
    real_T temp_tmp_0;
    int32_T temp_tmp_tmp;
    int32_T temp_tmp_tmp_0;
    temp_tmp_tmp = (iy0 + k) - 1;
    temp_tmp = x[temp_tmp_tmp];
    temp_tmp_tmp_0 = (ix0 + k) - 1;
    temp_tmp_0 = x[temp_tmp_tmp_0];
    x[temp_tmp_tmp] = temp_tmp * c - temp_tmp_0 * s;
    x[temp_tmp_tmp_0] = temp_tmp_0 * c + temp_tmp * s;
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::svd_d(const real_T A[256], real_T U[256], real_T s[16], real_T
                      V[256])
{
  __m128d tmp;
  real_T b_A[256];
  real_T e[16];
  real_T work[16];
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  int32_T exitg1;
  int32_T i;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  int32_T qq_tmp;
  int32_T qq_tmp_tmp;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T apply_transform;
  boolean_T exitg2;
  std::memcpy(&b_A[0], &A[0], sizeof(real_T) << 8U);
  std::memset(&s[0], 0, sizeof(real_T) << 4U);
  std::memset(&e[0], 0, sizeof(real_T) << 4U);
  std::memset(&work[0], 0, sizeof(real_T) << 4U);
  std::memset(&U[0], 0, sizeof(real_T) << 8U);
  std::memset(&V[0], 0, sizeof(real_T) << 8U);
  for (i = 0; i < 15; i++) {
    qp1 = i + 2;
    qq_tmp_tmp = i << 4;
    qq_tmp = qq_tmp_tmp + i;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = xnrm2_nn(16 - i, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      s[i] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qq_tmp - i) + 16;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - i) + 16;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_div_pd(tmp, _mm_set1_pd(s[i])));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] /= s[i];
        }
      }

      b_A[qq_tmp]++;
      s[i] = -s[i];
    } else {
      s[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 17; qp1jj++) {
      qjj = ((qp1jj - 1) << 4) + i;
      if (apply_transform) {
        xaxpy_c(16 - i, -(xdotc_f(16 - i, b_A, qq_tmp + 1, b_A, qjj + 1) /
                          b_A[qq_tmp]), qq_tmp + 1, b_A, qjj + 1);
      }

      e[qp1jj - 1] = b_A[qjj];
    }

    for (qq = i + 1; qq < 17; qq++) {
      qp1jj = (qq_tmp_tmp + qq) - 1;
      U[qp1jj] = b_A[qp1jj];
    }

    if (i + 1 <= 14) {
      nrm = xnrm2_nnh(15 - i, e, i + 2);
      if (nrm == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[i + 1] < 0.0) {
          e[i] = -nrm;
        } else {
          e[i] = nrm;
        }

        nrm = e[i];
        if (std::abs(e[i]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[i];
          scalarLB = ((((15 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 17; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          scalarLB = ((((15 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_div_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 17; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 17; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 17; qq++) {
          xaxpy_ck(15 - i, e[qq - 1], b_A, (i + ((qq - 1) << 4)) + 2, work, i +
                   2);
        }

        for (qq = qp1; qq < 17; qq++) {
          xaxpy_ckp(15 - i, -e[qq - 1] / e[i + 1], work, i + 2, b_A, (i + ((qq -
            1) << 4)) + 2);
        }
      }

      for (qq = qp1; qq < 17; qq++) {
        V[(qq + qq_tmp_tmp) - 1] = e[qq - 1];
      }
    }
  }

  i = 14;
  s[15] = b_A[255];
  e[14] = b_A[254];
  e[15] = 0.0;
  std::memset(&U[240], 0, sizeof(real_T) << 4U);
  U[255] = 1.0;
  for (qp1 = 14; qp1 >= 0; qp1--) {
    qq_tmp = qp1 << 4;
    qq = qq_tmp + qp1;
    if (s[qp1] != 0.0) {
      for (qp1jj = qp1 + 2; qp1jj < 17; qp1jj++) {
        qjj = (((qp1jj - 1) << 4) + qp1) + 1;
        xaxpy_c(16 - qp1, -(xdotc_f(16 - qp1, U, qq + 1, U, qjj) / U[qq]), qq +
                1, U, qjj);
      }

      for (qjj = qp1 + 1; qjj < 17; qjj++) {
        qp1jj = (qq_tmp + qjj) - 1;
        U[qp1jj] = -U[qp1jj];
      }

      U[qq]++;
      for (qjj = 0; qjj < qp1; qjj++) {
        U[qjj + qq_tmp] = 0.0;
      }
    } else {
      std::memset(&U[qq_tmp], 0, sizeof(real_T) << 4U);
      U[qq] = 1.0;
    }
  }

  for (qp1 = 15; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 14) && (e[qp1] != 0.0)) {
      qq = ((qp1 << 4) + qp1) + 2;
      for (qjj = qp1 + 2; qjj < 17; qjj++) {
        qp1jj = (((qjj - 1) << 4) + qp1) + 2;
        xaxpy_c(15 - qp1, -(xdotc_f(15 - qp1, V, qq, V, qp1jj) / V[qq - 1]), qq,
                V, qp1jj);
      }
    }

    std::memset(&V[qp1 << 4], 0, sizeof(real_T) << 4U);
    V[qp1 + (qp1 << 4)] = 1.0;
  }

  for (qp1 = 0; qp1 < 16; qp1++) {
    nrm = s[qp1];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      s[qp1] = rt;
      if (qp1 + 1 < 16) {
        e[qp1] /= nrm;
      }

      qq = (qp1 << 4) + 1;
      scalarLB = 16 + qq;
      vectorUB = qq + 14;
      for (qjj = qq; qjj <= vectorUB; qjj += 2) {
        tmp = _mm_loadu_pd(&U[qjj - 1]);
        _mm_storeu_pd(&U[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
      }

      for (qjj = scalarLB; qjj <= qq + 15; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if (qp1 + 1 < 16) {
      smm1 = e[qp1];
      if (smm1 != 0.0) {
        rt = std::abs(smm1);
        nrm = rt / smm1;
        e[qp1] = rt;
        s[qp1 + 1] *= nrm;
        qq = ((qp1 + 1) << 4) + 1;
        scalarLB = 16 + qq;
        vectorUB = qq + 14;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= qq + 15; qjj++) {
          V[qjj - 1] *= nrm;
        }
      }
    }
  }

  qp1 = 0;
  nrm = 0.0;
  for (qq = 0; qq < 16; qq++) {
    nrm = std::fmax(nrm, std::fmax(std::abs(s[qq]), std::abs(e[qq])));
  }

  while ((i + 2 > 0) && (qp1 < 75)) {
    qp1jj = i + 1;
    do {
      exitg1 = 0;
      qq = qp1jj;
      if (qp1jj == 0) {
        exitg1 = 1;
      } else {
        rt = std::abs(e[qp1jj - 1]);
        if (rt <= (std::abs(s[qp1jj - 1]) + std::abs(s[qp1jj])) *
            2.2204460492503131E-16) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else if ((rt <= 1.0020841800044864E-292) || ((qp1 > 20) && (rt <=
                     2.2204460492503131E-16 * nrm))) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg1 == 0);

    if (i + 1 == qp1jj) {
      qp1jj = 4;
    } else {
      qjj = i + 2;
      qq_tmp_tmp = i + 2;
      exitg2 = false;
      while ((!exitg2) && (qq_tmp_tmp >= qp1jj)) {
        qjj = qq_tmp_tmp;
        if (qq_tmp_tmp == qp1jj) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (qq_tmp_tmp < i + 2) {
            rt = std::abs(e[qq_tmp_tmp - 1]);
          }

          if (qq_tmp_tmp > qp1jj + 1) {
            rt += std::abs(e[qq_tmp_tmp - 2]);
          }

          ztest = std::abs(s[qq_tmp_tmp - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            s[qq_tmp_tmp - 1] = 0.0;
            exitg2 = true;
          } else {
            qq_tmp_tmp--;
          }
        }
      }

      if (qjj == qp1jj) {
        qp1jj = 3;
      } else if (i + 2 == qjj) {
        qp1jj = 1;
      } else {
        qp1jj = 2;
        qq = qjj;
      }
    }

    switch (qp1jj) {
     case 1:
      rt = e[i];
      e[i] = 0.0;
      for (qjj = i + 1; qjj >= qq + 1; qjj--) {
        xrotg(&s[qjj - 1], &rt, &ztest, &sqds);
        if (qjj > qq + 1) {
          smm1 = e[qjj - 2];
          rt = -sqds * smm1;
          e[qjj - 2] = smm1 * ztest;
        }

        xrot_g(V, ((qjj - 1) << 4) + 1, ((i + 1) << 4) + 1, ztest, sqds);
      }
      break;

     case 2:
      rt = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qjj = qq + 1; qjj <= i + 2; qjj++) {
        xrotg(&s[qjj - 1], &rt, &ztest, &sqds);
        smm1 = e[qjj - 1];
        rt = -sqds * smm1;
        e[qjj - 1] = smm1 * ztest;
        xrot_g(U, ((qjj - 1) << 4) + 1, ((qq - 1) << 4) + 1, ztest, sqds);
      }
      break;

     case 3:
      rt = s[i + 1];
      ztest = std::fmax(std::fmax(std::fmax(std::fmax(std::abs(rt), std::abs(s[i])),
        std::abs(e[i])), std::abs(s[qq])), std::abs(e[qq]));
      rt /= ztest;
      smm1 = s[i] / ztest;
      emm1 = e[i] / ztest;
      sqds = s[qq] / ztest;
      smm1 = ((smm1 + rt) * (smm1 - rt) + emm1 * emm1) / 2.0;
      emm1 *= rt;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = std::sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      rt = (sqds + rt) * (sqds - rt) + shift;
      ztest = e[qq] / ztest * sqds;
      for (qjj = qq + 1; qjj <= i + 1; qjj++) {
        xrotg(&rt, &ztest, &sqds, &smm1);
        if (qjj > qq + 1) {
          e[qjj - 2] = rt;
        }

        emm1 = e[qjj - 1];
        rt = s[qjj - 1];
        e[qjj - 1] = emm1 * sqds - rt * smm1;
        ztest = smm1 * s[qjj];
        s[qjj] *= sqds;
        qq_tmp_tmp = ((qjj - 1) << 4) + 1;
        qq_tmp = (qjj << 4) + 1;
        xrot_g(V, qq_tmp_tmp, qq_tmp, sqds, smm1);
        s[qjj - 1] = rt * sqds + emm1 * smm1;
        xrotg(&s[qjj - 1], &ztest, &sqds, &smm1);
        ztest = e[qjj - 1];
        rt = ztest * sqds + smm1 * s[qjj];
        s[qjj] = ztest * -smm1 + sqds * s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        xrot_g(U, qq_tmp_tmp, qq_tmp, sqds, smm1);
      }

      e[i] = rt;
      qp1++;
      break;

     default:
      if (s[qq] < 0.0) {
        s[qq] = -s[qq];
        qp1 = (qq << 4) + 1;
        scalarLB = 16 + qp1;
        vectorUB = qp1 + 14;
        for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
        }

        for (qjj = scalarLB; qjj <= qp1 + 15; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 16) && (s[qq] < s[qp1])) {
        rt = s[qq];
        s[qq] = s[qp1];
        s[qp1] = rt;
        qq_tmp_tmp = (qq << 4) + 1;
        qq_tmp = ((qq + 1) << 4) + 1;
        xswap_d(V, qq_tmp_tmp, qq_tmp);
        xswap_d(U, qq_tmp_tmp, qq_tmp);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      i--;
      break;
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
real_T talos_ekf::xnrm2_nnhl(int32_T n, const real_T x[512], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int32_T k{ix0}; k <= kend; k++) {
    real_T absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::xgemv_ez(int32_T m, int32_T n, const real_T A[512], int32_T ia0,
  const real_T x[512], int32_T ix0, real_T y[16])
{
  if (n != 0) {
    int32_T d;
    std::memset(&y[0], 0, static_cast<uint8_T>(n) * sizeof(real_T));
    d = ((n - 1) << 5) + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 32) {
      real_T c;
      int32_T b;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x[((ix0 + b) - b_iy) - 1] * A[b - 1];
      }

      b = (b_iy - ia0) >> 5;
      y[b] += c;
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::xgerc_p(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
  real_T y[16], real_T A[512], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T b;
    int32_T jA;
    jA = ia0;
    b = static_cast<uint8_T>(n);
    for (int32_T j{0}; j < b; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T c;
        temp *= alpha1;
        c = m + jA;
        for (int32_T ijA{jA}; ijA < c; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 32;
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::qrFactor(const real_T A[256], real_T S[256], const real_T Ns[256])
{
  __m128d tmp;
  real_T b_A[512];
  real_T y[256];
  real_T tau[16];
  real_T work[16];
  real_T atmp;
  real_T b_A_0;
  real_T s;
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T ii;
  int32_T j;
  int32_T lastv;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  boolean_T exitg2;
  for (j = 0; j < 16; j++) {
    for (ii = 0; ii < 16; ii++) {
      aoffset = ii << 4;
      s = 0.0;
      for (lastv = 0; lastv < 16; lastv++) {
        s += A[(lastv << 4) + j] * S[aoffset + lastv];
      }

      lastv = (j << 5) + ii;
      b_A[lastv] = s;
      b_A[lastv + 16] = Ns[aoffset + j];
    }

    work[j] = 0.0;
  }

  for (j = 0; j < 16; j++) {
    ii = (j << 5) + j;
    atmp = b_A[ii];
    lastv = ii + 2;
    tau[j] = 0.0;
    s = xnrm2_nnhl(31 - j, b_A, ii + 2);
    if (s != 0.0) {
      b_A_0 = b_A[ii];
      s = rt_hypotd_snf(b_A_0, s);
      if (b_A_0 >= 0.0) {
        s = -s;
      }

      if (std::abs(s) < 1.0020841800044864E-292) {
        coffset = 0;
        scalarLB = (ii - j) + 32;
        do {
          coffset++;
          vectorUB = (((((scalarLB - ii) - 1) / 2) << 1) + ii) + 2;
          vectorUB_tmp = vectorUB - 2;
          for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
            tmp = _mm_loadu_pd(&b_A[aoffset - 1]);
            _mm_storeu_pd(&b_A[aoffset - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
            b_A[aoffset - 1] *= 9.9792015476736E+291;
          }

          s *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((std::abs(s) < 1.0020841800044864E-292) && (coffset < 20));

        s = rt_hypotd_snf(atmp, xnrm2_nnhl(31 - j, b_A, ii + 2));
        if (atmp >= 0.0) {
          s = -s;
        }

        tau[j] = (s - atmp) / s;
        atmp = 1.0 / (atmp - s);
        for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
          tmp = _mm_loadu_pd(&b_A[aoffset - 1]);
          _mm_storeu_pd(&b_A[aoffset - 1], _mm_mul_pd(tmp, _mm_set1_pd(atmp)));
        }

        for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
          b_A[aoffset - 1] *= atmp;
        }

        for (lastv = 0; lastv < coffset; lastv++) {
          s *= 1.0020841800044864E-292;
        }

        atmp = s;
      } else {
        tau[j] = (s - b_A_0) / s;
        atmp = 1.0 / (b_A_0 - s);
        aoffset = (ii - j) + 32;
        scalarLB = (((((aoffset - ii) - 1) / 2) << 1) + ii) + 2;
        vectorUB = scalarLB - 2;
        for (coffset = lastv; coffset <= vectorUB; coffset += 2) {
          tmp = _mm_loadu_pd(&b_A[coffset - 1]);
          _mm_storeu_pd(&b_A[coffset - 1], _mm_mul_pd(tmp, _mm_set1_pd(atmp)));
        }

        for (coffset = scalarLB; coffset <= aoffset; coffset++) {
          b_A[coffset - 1] *= atmp;
        }

        atmp = s;
      }
    }

    b_A[ii] = atmp;
    if (j + 1 < 16) {
      b_A[ii] = 1.0;
      if (tau[j] != 0.0) {
        lastv = 32 - j;
        coffset = (ii - j) + 31;
        while ((lastv > 0) && (b_A[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }

        coffset = 15 - j;
        exitg2 = false;
        while ((!exitg2) && (coffset > 0)) {
          aoffset = (((coffset - 1) << 5) + ii) + 32;
          scalarLB = aoffset;
          do {
            exitg1 = 0;
            if (scalarLB + 1 <= aoffset + lastv) {
              if (b_A[scalarLB] != 0.0) {
                exitg1 = 1;
              } else {
                scalarLB++;
              }
            } else {
              coffset--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        coffset = 0;
      }

      if (lastv > 0) {
        xgemv_ez(lastv, coffset, b_A, ii + 33, b_A, ii + 1, work);
        xgerc_p(lastv, coffset, -tau[j], ii + 1, work, b_A, ii + 33);
      }

      b_A[ii] = atmp;
    }
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii <= j; ii++) {
      y[ii + (j << 4)] = b_A[(j << 5) + ii];
    }

    for (ii = j + 2; ii < 17; ii++) {
      y[(ii + (j << 4)) - 1] = 0.0;
    }
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii < 16; ii++) {
      S[ii + (j << 4)] = y[(ii << 4) + j];
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::trisolve_j(const real_T A[256], real_T B_a[256])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = j << 4;
    for (int32_T b_k{0}; b_k < 16; b_k++) {
      real_T B_b;
      int32_T B_tmp;
      int32_T kAcol;
      kAcol = b_k << 4;
      B_tmp = b_k + jBcol;
      B_b = B_a[B_tmp];
      if (B_b != 0.0) {
        B_a[B_tmp] = B_b / A[b_k + kAcol];
        for (int32_T i{b_k + 2}; i < 17; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          B_a[tmp] -= A[(i + kAcol) - 1] * B_a[B_tmp];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::trisolve_jl(const real_T A[256], real_T B_c[256])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = j << 4;
    for (int32_T k{15}; k >= 0; k--) {
      real_T tmp;
      int32_T kAcol;
      int32_T tmp_0;
      kAcol = k << 4;
      tmp_0 = k + jBcol;
      tmp = B_c[tmp_0];
      if (tmp != 0.0) {
        B_c[tmp_0] = tmp / A[k + kAcol];
        for (int32_T i{0}; i < k; i++) {
          int32_T tmp_1;
          tmp_1 = i + jBcol;
          B_c[tmp_1] -= A[i + kAcol] * B_c[tmp_0];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S8>/Predict'
real_T talos_ekf::xnrm2_d(int32_T n, const real_T x[256], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S8>/Predict'
real_T talos_ekf::xdotc_e(int32_T n, const real_T x[256], int32_T ix0, const
  real_T y[256], int32_T iy0)
{
  real_T d;
  d = 0.0;
  if (n >= 1) {
    for (int32_T k{0}; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }

  return d;
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::xaxpy_d(int32_T n, real_T a, int32_T ix0, real_T y[256], int32_T
  iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S8>/Predict'
real_T talos_ekf::xnrm2_dz(int32_T n, const real_T x[16], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::xaxpy_d2(int32_T n, real_T a, const real_T x[256], int32_T ix0,
  real_T y[16], int32_T iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k{scalarLB}; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::xaxpy_d2s(int32_T n, real_T a, const real_T x[16], int32_T ix0,
  real_T y[256], int32_T iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (int32_T k{scalarLB}; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::svd_n(const real_T A[256], real_T U[256], real_T s[16], real_T
                      V[256])
{
  __m128d tmp;
  real_T b_A[256];
  real_T e[16];
  real_T work[16];
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  int32_T exitg1;
  int32_T i;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  int32_T qq_tmp;
  int32_T qq_tmp_tmp;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T apply_transform;
  boolean_T exitg2;
  std::memcpy(&b_A[0], &A[0], sizeof(real_T) << 8U);
  std::memset(&s[0], 0, sizeof(real_T) << 4U);
  std::memset(&e[0], 0, sizeof(real_T) << 4U);
  std::memset(&work[0], 0, sizeof(real_T) << 4U);
  std::memset(&U[0], 0, sizeof(real_T) << 8U);
  std::memset(&V[0], 0, sizeof(real_T) << 8U);
  for (i = 0; i < 15; i++) {
    qp1 = i + 2;
    qq_tmp_tmp = i << 4;
    qq_tmp = qq_tmp_tmp + i;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = xnrm2_d(16 - i, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      s[i] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qq_tmp - i) + 16;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - i) + 16;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_div_pd(tmp, _mm_set1_pd(s[i])));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] /= s[i];
        }
      }

      b_A[qq_tmp]++;
      s[i] = -s[i];
    } else {
      s[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 17; qp1jj++) {
      qjj = ((qp1jj - 1) << 4) + i;
      if (apply_transform) {
        xaxpy_d(16 - i, -(xdotc_e(16 - i, b_A, qq_tmp + 1, b_A, qjj + 1) /
                          b_A[qq_tmp]), qq_tmp + 1, b_A, qjj + 1);
      }

      e[qp1jj - 1] = b_A[qjj];
    }

    for (qq = i + 1; qq < 17; qq++) {
      qp1jj = (qq_tmp_tmp + qq) - 1;
      U[qp1jj] = b_A[qp1jj];
    }

    if (i + 1 <= 14) {
      nrm = xnrm2_dz(15 - i, e, i + 2);
      if (nrm == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[i + 1] < 0.0) {
          e[i] = -nrm;
        } else {
          e[i] = nrm;
        }

        nrm = e[i];
        if (std::abs(e[i]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[i];
          scalarLB = ((((15 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 17; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          scalarLB = ((((15 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_div_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 17; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 17; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 17; qq++) {
          xaxpy_d2(15 - i, e[qq - 1], b_A, (i + ((qq - 1) << 4)) + 2, work, i +
                   2);
        }

        for (qq = qp1; qq < 17; qq++) {
          xaxpy_d2s(15 - i, -e[qq - 1] / e[i + 1], work, i + 2, b_A, (i + ((qq -
            1) << 4)) + 2);
        }
      }

      for (qq = qp1; qq < 17; qq++) {
        V[(qq + qq_tmp_tmp) - 1] = e[qq - 1];
      }
    }
  }

  i = 14;
  s[15] = b_A[255];
  e[14] = b_A[254];
  e[15] = 0.0;
  std::memset(&U[240], 0, sizeof(real_T) << 4U);
  U[255] = 1.0;
  for (qp1 = 14; qp1 >= 0; qp1--) {
    qq_tmp = qp1 << 4;
    qq = qq_tmp + qp1;
    if (s[qp1] != 0.0) {
      for (qp1jj = qp1 + 2; qp1jj < 17; qp1jj++) {
        qjj = (((qp1jj - 1) << 4) + qp1) + 1;
        xaxpy_d(16 - qp1, -(xdotc_e(16 - qp1, U, qq + 1, U, qjj) / U[qq]), qq +
                1, U, qjj);
      }

      for (qjj = qp1 + 1; qjj < 17; qjj++) {
        qp1jj = (qq_tmp + qjj) - 1;
        U[qp1jj] = -U[qp1jj];
      }

      U[qq]++;
      for (qjj = 0; qjj < qp1; qjj++) {
        U[qjj + qq_tmp] = 0.0;
      }
    } else {
      std::memset(&U[qq_tmp], 0, sizeof(real_T) << 4U);
      U[qq] = 1.0;
    }
  }

  for (qp1 = 15; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 14) && (e[qp1] != 0.0)) {
      qq = ((qp1 << 4) + qp1) + 2;
      for (qjj = qp1 + 2; qjj < 17; qjj++) {
        qp1jj = (((qjj - 1) << 4) + qp1) + 2;
        xaxpy_d(15 - qp1, -(xdotc_e(15 - qp1, V, qq, V, qp1jj) / V[qq - 1]), qq,
                V, qp1jj);
      }
    }

    std::memset(&V[qp1 << 4], 0, sizeof(real_T) << 4U);
    V[qp1 + (qp1 << 4)] = 1.0;
  }

  for (qp1 = 0; qp1 < 16; qp1++) {
    nrm = s[qp1];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      s[qp1] = rt;
      if (qp1 + 1 < 16) {
        e[qp1] /= nrm;
      }

      qq = (qp1 << 4) + 1;
      scalarLB = 16 + qq;
      vectorUB = qq + 14;
      for (qjj = qq; qjj <= vectorUB; qjj += 2) {
        tmp = _mm_loadu_pd(&U[qjj - 1]);
        _mm_storeu_pd(&U[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
      }

      for (qjj = scalarLB; qjj <= qq + 15; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if (qp1 + 1 < 16) {
      smm1 = e[qp1];
      if (smm1 != 0.0) {
        rt = std::abs(smm1);
        nrm = rt / smm1;
        e[qp1] = rt;
        s[qp1 + 1] *= nrm;
        qq = ((qp1 + 1) << 4) + 1;
        scalarLB = 16 + qq;
        vectorUB = qq + 14;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= qq + 15; qjj++) {
          V[qjj - 1] *= nrm;
        }
      }
    }
  }

  qp1 = 0;
  nrm = 0.0;
  for (qq = 0; qq < 16; qq++) {
    nrm = std::fmax(nrm, std::fmax(std::abs(s[qq]), std::abs(e[qq])));
  }

  while ((i + 2 > 0) && (qp1 < 75)) {
    qp1jj = i + 1;
    do {
      exitg1 = 0;
      qq = qp1jj;
      if (qp1jj == 0) {
        exitg1 = 1;
      } else {
        rt = std::abs(e[qp1jj - 1]);
        if (rt <= (std::abs(s[qp1jj - 1]) + std::abs(s[qp1jj])) *
            2.2204460492503131E-16) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else if ((rt <= 1.0020841800044864E-292) || ((qp1 > 20) && (rt <=
                     2.2204460492503131E-16 * nrm))) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg1 == 0);

    if (i + 1 == qp1jj) {
      qp1jj = 4;
    } else {
      qjj = i + 2;
      qq_tmp_tmp = i + 2;
      exitg2 = false;
      while ((!exitg2) && (qq_tmp_tmp >= qp1jj)) {
        qjj = qq_tmp_tmp;
        if (qq_tmp_tmp == qp1jj) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (qq_tmp_tmp < i + 2) {
            rt = std::abs(e[qq_tmp_tmp - 1]);
          }

          if (qq_tmp_tmp > qp1jj + 1) {
            rt += std::abs(e[qq_tmp_tmp - 2]);
          }

          ztest = std::abs(s[qq_tmp_tmp - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            s[qq_tmp_tmp - 1] = 0.0;
            exitg2 = true;
          } else {
            qq_tmp_tmp--;
          }
        }
      }

      if (qjj == qp1jj) {
        qp1jj = 3;
      } else if (i + 2 == qjj) {
        qp1jj = 1;
      } else {
        qp1jj = 2;
        qq = qjj;
      }
    }

    switch (qp1jj) {
     case 1:
      rt = e[i];
      e[i] = 0.0;
      for (qjj = i + 1; qjj >= qq + 1; qjj--) {
        xrotg(&s[qjj - 1], &rt, &ztest, &sqds);
        if (qjj > qq + 1) {
          smm1 = e[qjj - 2];
          rt = -sqds * smm1;
          e[qjj - 2] = smm1 * ztest;
        }

        xrot_g(V, ((qjj - 1) << 4) + 1, ((i + 1) << 4) + 1, ztest, sqds);
      }
      break;

     case 2:
      rt = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qjj = qq + 1; qjj <= i + 2; qjj++) {
        xrotg(&s[qjj - 1], &rt, &ztest, &sqds);
        smm1 = e[qjj - 1];
        rt = -sqds * smm1;
        e[qjj - 1] = smm1 * ztest;
        xrot_g(U, ((qjj - 1) << 4) + 1, ((qq - 1) << 4) + 1, ztest, sqds);
      }
      break;

     case 3:
      rt = s[i + 1];
      ztest = std::fmax(std::fmax(std::fmax(std::fmax(std::abs(rt), std::abs(s[i])),
        std::abs(e[i])), std::abs(s[qq])), std::abs(e[qq]));
      rt /= ztest;
      smm1 = s[i] / ztest;
      emm1 = e[i] / ztest;
      sqds = s[qq] / ztest;
      smm1 = ((smm1 + rt) * (smm1 - rt) + emm1 * emm1) / 2.0;
      emm1 *= rt;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = std::sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      rt = (sqds + rt) * (sqds - rt) + shift;
      ztest = e[qq] / ztest * sqds;
      for (qjj = qq + 1; qjj <= i + 1; qjj++) {
        xrotg(&rt, &ztest, &sqds, &smm1);
        if (qjj > qq + 1) {
          e[qjj - 2] = rt;
        }

        emm1 = e[qjj - 1];
        rt = s[qjj - 1];
        e[qjj - 1] = emm1 * sqds - rt * smm1;
        ztest = smm1 * s[qjj];
        s[qjj] *= sqds;
        qq_tmp_tmp = ((qjj - 1) << 4) + 1;
        qq_tmp = (qjj << 4) + 1;
        xrot_g(V, qq_tmp_tmp, qq_tmp, sqds, smm1);
        s[qjj - 1] = rt * sqds + emm1 * smm1;
        xrotg(&s[qjj - 1], &ztest, &sqds, &smm1);
        ztest = e[qjj - 1];
        rt = ztest * sqds + smm1 * s[qjj];
        s[qjj] = ztest * -smm1 + sqds * s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        xrot_g(U, qq_tmp_tmp, qq_tmp, sqds, smm1);
      }

      e[i] = rt;
      qp1++;
      break;

     default:
      if (s[qq] < 0.0) {
        s[qq] = -s[qq];
        qp1 = (qq << 4) + 1;
        scalarLB = 16 + qp1;
        vectorUB = qp1 + 14;
        for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
        }

        for (qjj = scalarLB; qjj <= qp1 + 15; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 16) && (s[qq] < s[qp1])) {
        rt = s[qq];
        s[qq] = s[qp1];
        s[qp1] = rt;
        qq_tmp_tmp = (qq << 4) + 1;
        qq_tmp = ((qq + 1) << 4) + 1;
        xswap_d(V, qq_tmp_tmp, qq_tmp);
        xswap_d(U, qq_tmp_tmp, qq_tmp);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      i--;
      break;
    }
  }
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::talos_state_transition(const real_T x[16], real_T dt, real_T
  next[16])
{
  real_T R[9];
  real_T c_x[3];
  real_T r[3];
  real_T R_tmp;
  real_T R_tmp_0;
  real_T R_tmp_1;
  real_T R_tmp_2;
  real_T R_tmp_3;
  real_T b_q_idx_3;
  real_T c_q_idx_2;
  real_T n;
  real_T q_idx_0;
  real_T q_idx_1;
  real_T q_idx_2;
  real_T q_idx_3;
  dt = std::fmin(std::fmax(dt, 0.0), 0.1);
  n = std::sqrt(((x[3] * x[3] + x[4] * x[4]) + x[5] * x[5]) + x[6] * x[6]);
  if (n < 1.0E-12) {
    q_idx_0 = 1.0;
    q_idx_1 = 0.0;
    q_idx_2 = 0.0;
    q_idx_3 = 0.0;
  } else {
    q_idx_0 = x[3] / n;
    q_idx_1 = x[4] / n;
    q_idx_2 = x[5] / n;
    q_idx_3 = x[6] / n;
  }

  n = q_idx_3 * q_idx_3;
  R_tmp_3 = q_idx_2 * q_idx_2;
  R[0] = 1.0 - (R_tmp_3 + n) * 2.0;
  R_tmp = q_idx_1 * q_idx_2;
  R_tmp_0 = q_idx_0 * q_idx_3;
  R[3] = (R_tmp - R_tmp_0) * 2.0;
  R_tmp_1 = q_idx_1 * q_idx_3;
  R_tmp_2 = q_idx_0 * q_idx_2;
  R[6] = (R_tmp_1 + R_tmp_2) * 2.0;
  R[1] = (R_tmp + R_tmp_0) * 2.0;
  R_tmp = q_idx_1 * q_idx_1;
  R[4] = 1.0 - (R_tmp + n) * 2.0;
  n = q_idx_2 * q_idx_3;
  R_tmp_0 = q_idx_0 * q_idx_1;
  R[7] = (n - R_tmp_0) * 2.0;
  R[2] = (R_tmp_1 - R_tmp_2) * 2.0;
  R[5] = (n + R_tmp_0) * 2.0;
  R[8] = 1.0 - (R_tmp + R_tmp_3) * 2.0;
  std::memcpy(&next[0], &x[0], sizeof(real_T) << 4U);
  for (int32_T i{0}; i <= 0; i += 2) {
    __m128d tmp;
    __m128d tmp_0;
    __m128d tmp_1;
    __m128d tmp_2;
    __m128d tmp_3;
    tmp = _mm_loadu_pd(&R[i]);
    tmp_2 = _mm_set1_pd(0.5);
    tmp_0 = _mm_loadu_pd(&R[i + 3]);
    tmp_1 = _mm_loadu_pd(&R[i + 6]);
    tmp_3 = _mm_set1_pd(dt);
    _mm_storeu_pd(&next[i], _mm_add_pd(_mm_mul_pd(_mm_mul_pd(_mm_add_pd
      (_mm_mul_pd(_mm_mul_pd(tmp_2, tmp_1), _mm_set1_pd(x[15])), _mm_add_pd
       (_mm_mul_pd(_mm_mul_pd(tmp_2, tmp_0), _mm_set1_pd(x[14])), _mm_mul_pd
        (_mm_mul_pd(tmp_2, tmp), _mm_set1_pd(x[13])))), tmp_3), tmp_3),
      _mm_add_pd(_mm_mul_pd(_mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(x[9])),
      _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd(x[8])), _mm_mul_pd(tmp,
      _mm_set1_pd(x[7])))), tmp_3), _mm_loadu_pd(&x[i]))));
    tmp = _mm_mul_pd(_mm_loadu_pd(&x[i + 10]), tmp_3);
    _mm_storeu_pd(&r[i], tmp);
    _mm_storeu_pd(&c_x[i], _mm_mul_pd(tmp, tmp));
  }

  for (int32_T i{2}; i < 3; i++) {
    n = R[i];
    R_tmp_3 = n * x[7];
    R_tmp = 0.5 * n * x[13];
    n = R[i + 3];
    R_tmp_3 += n * x[8];
    R_tmp += 0.5 * n * x[14];
    n = R[i + 6];
    next[i] = (0.5 * n * x[15] + R_tmp) * dt * dt + ((n * x[9] + R_tmp_3) * dt +
      x[i]);
    n = x[i + 10] * dt;
    r[i] = n;
    c_x[i] = n * n;
  }

  n = std::sqrt((c_x[0] + c_x[1]) + c_x[2]);
  if (n < 1.0E-9) {
    R_tmp_3 = 1.0;
    R_tmp = 0.5 * r[0];
    R_tmp_0 = 0.5 * r[1];
    b_q_idx_3 = 0.5 * r[2];
    n = std::sqrt(((R_tmp * R_tmp + 1.0) + R_tmp_0 * R_tmp_0) + b_q_idx_3 *
                  b_q_idx_3);
    if (n < 1.0E-12) {
      R_tmp = 0.0;
      R_tmp_0 = 0.0;
      b_q_idx_3 = 0.0;
    } else {
      R_tmp_3 = 1.0 / n;
      R_tmp /= n;
      R_tmp_0 /= n;
      b_q_idx_3 /= n;
    }
  } else {
    R_tmp_1 = std::sin(0.5 * n);
    R_tmp_3 = std::cos(0.5 * n);
    R_tmp = R_tmp_1 * r[0] / n;
    R_tmp_0 = R_tmp_1 * r[1] / n;
    b_q_idx_3 = R_tmp_1 * r[2] / n;
  }

  R_tmp_1 = q_idx_0 * R_tmp_3 - ((q_idx_1 * R_tmp + q_idx_2 * R_tmp_0) + q_idx_3
    * b_q_idx_3);
  R_tmp_2 = (q_idx_0 * R_tmp + R_tmp_3 * q_idx_1) + (q_idx_2 * b_q_idx_3 -
    R_tmp_0 * q_idx_3);
  c_q_idx_2 = (q_idx_0 * R_tmp_0 + R_tmp_3 * q_idx_2) + (R_tmp * q_idx_3 -
    q_idx_1 * b_q_idx_3);
  q_idx_0 = (q_idx_0 * b_q_idx_3 + R_tmp_3 * q_idx_3) + (q_idx_1 * R_tmp_0 -
    R_tmp * q_idx_2);
  n = std::sqrt(((R_tmp_1 * R_tmp_1 + R_tmp_2 * R_tmp_2) + c_q_idx_2 * c_q_idx_2)
                + q_idx_0 * q_idx_0);
  if (n < 1.0E-12) {
    next[3] = 1.0;
    next[4] = 0.0;
    next[5] = 0.0;
    next[6] = 0.0;
  } else {
    next[3] = R_tmp_1 / n;
    next[4] = R_tmp_2 / n;
    next[5] = c_q_idx_2 / n;
    next[6] = q_idx_0 / n;
  }

  next[7] = (x[13] - (x[9] * x[11] - x[8] * x[12])) * dt + x[7];
  next[8] = (x[14] - (x[7] * x[12] - x[9] * x[10])) * dt + x[8];
  next[9] = (x[15] - (x[8] * x[10] - x[7] * x[11])) * dt + x[9];
}

// Function for MATLAB Function: '<S8>/Predict'
real_T talos_ekf::xnrm2_dzn(int32_T n, const real_T x[512], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::xgemv_j(int32_T m, int32_T n, const real_T A[512], int32_T ia0,
  const real_T x[512], int32_T ix0, real_T y[16])
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      std::memset(&y[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
    }

    b = ((n - 1) << 5) + ia0;
    for (int32_T b_iy{ia0}; b_iy <= b; b_iy += 32) {
      real_T c;
      int32_T d;
      int32_T iyend;
      c = 0.0;
      d = (b_iy + m) - 1;
      for (iyend = b_iy; iyend <= d; iyend++) {
        c += x[((ix0 + iyend) - b_iy) - 1] * A[iyend - 1];
      }

      iyend = (b_iy - ia0) >> 5;
      y[iyend] += c;
    }
  }
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::xgerc_e(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
  real_T y[16], real_T A[512], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j{0}; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = m + jA;
        for (int32_T ijA{jA}; ijA < b; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 32;
    }
  }
}

// Model step function
void talos_ekf::step()
{
  static const real_T b[256]{ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  real_T A_1[512];
  real_T C[256];
  real_T K[256];
  real_T K_0[256];
  real_T Rsqrt_1[256];
  real_T Ss_1[256];
  real_T A[192];
  real_T dHdx[128];
  real_T tmp_0[128];
  real_T Rsqrt[64];
  real_T Ss[64];
  real_T a[64];
  real_T A_0[57];
  real_T dHdx_0[48];
  real_T y[48];
  real_T rtb_xNew_k[16];
  real_T tmp[16];
  real_T xm[16];
  real_T Rsqrt_0[9];
  real_T Ss_0[9];
  real_T a_0[9];
  real_T s[8];
  real_T work[8];
  real_T s_0[3];
  real_T tau[3];
  real_T Vf;
  real_T b_q_idx_0;
  real_T b_q_idx_1;
  real_T b_q_idx_2;
  real_T h;
  real_T q_idx_0;
  real_T q_idx_1;
  real_T q_idx_2;
  real_T q_idx_3;
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T i;
  int32_T lastv;
  int32_T m;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T vectorUB_tmp;
  boolean_T exitg2;
  boolean_T p;

  // Outputs for Enabled SubSystem: '<S1>/Correct1' incorporates:
  //   EnablePort: '<S2>/Enable'

  // Inport: '<Root>/enable_imu'
  if (rtU.enable_imu) {
    // MATLAB Function: '<S2>/Correct' incorporates:
    //   Constant: '<S1>/BlockOrdering'
    //   Inport: '<Root>/R_imu'

    rtDW.blockOrdering_k = true;
    p = true;
    for (m = 0; m < 64; m++) {
      if (p && (std::isinf(rtU.R_imu[m]) || std::isnan(rtU.R_imu[m]))) {
        p = false;
      }
    }

    if (p) {
      svd(rtU.R_imu, Ss, s, a);
    } else {
      for (i = 0; i < 8; i++) {
        s[i] = (rtNaN);
      }

      for (lastv = 0; lastv < 64; lastv++) {
        a[lastv] = (rtNaN);
      }
    }

    std::memset(&Ss[0], 0, sizeof(real_T) << 6U);
    for (m = 0; m < 8; m++) {
      Ss[m + (m << 3)] = s[m];
    }

    for (m = 0; m <= 62; m += 2) {
      // MATLAB Function: '<S2>/Correct'
      tmp_3 = _mm_loadu_pd(&Ss[m]);
      _mm_storeu_pd(&Ss[m], _mm_sqrt_pd(tmp_3));
    }

    // MATLAB Function: '<S2>/Correct' incorporates:
    //   DataStoreRead: '<S2>/Data Store ReadX'
    //   DataStoreWrite: '<S2>/Data Store WriteP'
    //   Inport: '<Root>/imu_measurement'

    for (lastv = 0; lastv < 8; lastv++) {
      for (m = 0; m < 8; m++) {
        h = 0.0;
        for (i = 0; i < 8; i++) {
          h += a[(i << 3) + m] * Ss[(lastv << 3) + i];
        }

        Rsqrt[m + (lastv << 3)] = h;
      }
    }

    for (aoffset = 0; aoffset < 16; aoffset++) {
      h = 1.0E-6 * std::fmax(1.0, std::abs(rtDW.x[aoffset]));
      std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
      std::memcpy(&xm[0], &rtDW.x[0], sizeof(real_T) << 4U);
      q_idx_0 = rtDW.x[aoffset];
      rtb_xNew_k[aoffset] = q_idx_0 + h;
      xm[aoffset] = q_idx_0 - h;
      Vf = std::fmax(std::sqrt(((rtb_xNew_k[3] * rtb_xNew_k[3] + rtb_xNew_k[4] *
        rtb_xNew_k[4]) + rtb_xNew_k[5] * rtb_xNew_k[5]) + rtb_xNew_k[6] *
        rtb_xNew_k[6]), 1.0E-12);
      q_idx_0 = rtb_xNew_k[3] / Vf;
      q_idx_1 = rtb_xNew_k[4] / Vf;
      q_idx_2 = rtb_xNew_k[5] / Vf;
      q_idx_3 = rtb_xNew_k[6] / Vf;
      Vf = std::fmax(std::sqrt(((xm[3] * xm[3] + xm[4] * xm[4]) + xm[5] * xm[5])
        + xm[6] * xm[6]), 1.0E-12);
      b_q_idx_0 = xm[3] / Vf;
      b_q_idx_1 = xm[4] / Vf;
      b_q_idx_2 = xm[5] / Vf;
      Vf = xm[6] / Vf;
      s[0] = (q_idx_1 * q_idx_3 - q_idx_0 * q_idx_2) * 2.0;
      s[1] = (q_idx_2 * q_idx_3 + q_idx_0 * q_idx_1) * 2.0;
      s[2] = 1.0 - (q_idx_1 * q_idx_1 + q_idx_2 * q_idx_2) * 2.0;
      s[3] = rtb_xNew_k[10];
      s[4] = rtb_xNew_k[11];
      s[5] = rtb_xNew_k[13];
      s[6] = rtb_xNew_k[14];
      s[7] = rtb_xNew_k[15];
      work[0] = (b_q_idx_1 * Vf - b_q_idx_0 * b_q_idx_2) * 2.0;
      work[1] = (b_q_idx_2 * Vf + b_q_idx_0 * b_q_idx_1) * 2.0;
      work[2] = 1.0 - (b_q_idx_1 * b_q_idx_1 + b_q_idx_2 * b_q_idx_2) * 2.0;
      work[3] = xm[10];
      work[4] = xm[11];
      work[5] = xm[13];
      work[6] = xm[14];
      work[7] = xm[15];
      h *= 2.0;
      for (lastv = 0; lastv <= 6; lastv += 2) {
        tmp_3 = _mm_loadu_pd(&s[lastv]);
        tmp_2 = _mm_loadu_pd(&work[lastv]);
        _mm_storeu_pd(&dHdx[lastv + (aoffset << 3)], _mm_div_pd(_mm_sub_pd(tmp_3,
          tmp_2), _mm_set1_pd(h)));
      }
    }

    h = std::fmax(std::sqrt(((rtDW.x[3] * rtDW.x[3] + rtDW.x[4] * rtDW.x[4]) +
      rtDW.x[5] * rtDW.x[5]) + rtDW.x[6] * rtDW.x[6]), 1.0E-12);
    q_idx_0 = rtDW.x[3] / h;
    q_idx_1 = rtDW.x[4] / h;
    q_idx_2 = rtDW.x[5] / h;
    q_idx_3 = rtDW.x[6] / h;
    for (m = 0; m < 8; m++) {
      for (i = 0; i < 16; i++) {
        aoffset = i << 4;
        h = 0.0;
        for (lastv = 0; lastv < 16; lastv++) {
          h += dHdx[(lastv << 3) + m] * rtDW.P_i[aoffset + lastv];
        }

        A[i + 24 * m] = h;
      }

      for (lastv = 0; lastv < 8; lastv++) {
        A[(lastv + 24 * m) + 16] = Rsqrt[(lastv << 3) + m];
      }

      work[m] = 0.0;
    }

    for (m = 0; m < 8; m++) {
      coffset = m * 24 + m;
      Vf = A[coffset];
      lastv = coffset + 2;
      s[m] = 0.0;
      h = xnrm2_ln(23 - m, A, coffset + 2);
      if (h != 0.0) {
        b_q_idx_0 = A[coffset];
        h = rt_hypotd_snf(b_q_idx_0, h);
        if (b_q_idx_0 >= 0.0) {
          h = -h;
        }

        if (std::abs(h) < 1.0020841800044864E-292) {
          i = 0;
          scalarLB = (coffset - m) + 24;
          do {
            i++;
            vectorUB = (((((scalarLB - coffset) - 1) / 2) << 1) + coffset) + 2;
            vectorUB_tmp = vectorUB - 2;
            for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
              tmp_3 = _mm_loadu_pd(&A[aoffset - 1]);
              _mm_storeu_pd(&A[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd
                (9.9792015476736E+291)));
            }

            for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
              A[aoffset - 1] *= 9.9792015476736E+291;
            }

            h *= 9.9792015476736E+291;
            Vf *= 9.9792015476736E+291;
          } while ((std::abs(h) < 1.0020841800044864E-292) && (i < 20));

          h = rt_hypotd_snf(Vf, xnrm2_ln(23 - m, A, coffset + 2));
          if (Vf >= 0.0) {
            h = -h;
          }

          s[m] = (h - Vf) / h;
          Vf = 1.0 / (Vf - h);
          for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
            tmp_3 = _mm_loadu_pd(&A[aoffset - 1]);
            _mm_storeu_pd(&A[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd(Vf)));
          }

          for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
            A[aoffset - 1] *= Vf;
          }

          for (lastv = 0; lastv < i; lastv++) {
            h *= 1.0020841800044864E-292;
          }

          Vf = h;
        } else {
          s[m] = (h - b_q_idx_0) / h;
          Vf = 1.0 / (b_q_idx_0 - h);
          i = (coffset - m) + 24;
          scalarLB = (((((i - coffset) - 1) / 2) << 1) + coffset) + 2;
          vectorUB = scalarLB - 2;
          for (aoffset = lastv; aoffset <= vectorUB; aoffset += 2) {
            tmp_3 = _mm_loadu_pd(&A[aoffset - 1]);
            _mm_storeu_pd(&A[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd(Vf)));
          }

          for (aoffset = scalarLB; aoffset <= i; aoffset++) {
            A[aoffset - 1] *= Vf;
          }

          Vf = h;
        }
      }

      A[coffset] = Vf;
      if (m + 1 < 8) {
        A[coffset] = 1.0;
        if (s[m] != 0.0) {
          lastv = 24 - m;
          i = (coffset - m) + 23;
          while ((lastv > 0) && (A[i] == 0.0)) {
            lastv--;
            i--;
          }

          i = 7 - m;
          exitg2 = false;
          while ((!exitg2) && (i > 0)) {
            aoffset = ((i - 1) * 24 + coffset) + 24;
            scalarLB = aoffset;
            do {
              exitg1 = 0;
              if (scalarLB + 1 <= aoffset + lastv) {
                if (A[scalarLB] != 0.0) {
                  exitg1 = 1;
                } else {
                  scalarLB++;
                }
              } else {
                i--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          i = 0;
        }

        if (lastv > 0) {
          xgemv(lastv, i, A, coffset + 25, A, coffset + 1, work);
          xgerc(lastv, i, -s[m], coffset + 1, work, A, coffset + 25);
        }

        A[coffset] = Vf;
      }
    }

    for (m = 0; m < 8; m++) {
      for (coffset = 0; coffset <= m; coffset++) {
        Ss[coffset + (m << 3)] = A[24 * m + coffset];
      }

      for (coffset = m + 2; coffset < 9; coffset++) {
        Ss[(coffset + (m << 3)) - 1] = 0.0;
      }
    }

    std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
    std::memcpy(&Ss_1[0], &rtDW.P_i[0], sizeof(real_T) << 8U);
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        h = 0.0;
        for (i = 0; i < 16; i++) {
          aoffset = i << 4;
          h += rtDW.P_i[aoffset + lastv] * rtDW.P_i[aoffset + m];
        }

        C[lastv + (m << 4)] = h;
      }
    }

    s[0] = rtU.imu_measurement[0] - (q_idx_1 * q_idx_3 - q_idx_0 * q_idx_2) *
      2.0;
    s[1] = rtU.imu_measurement[1] - (q_idx_2 * q_idx_3 + q_idx_0 * q_idx_1) *
      2.0;
    s[2] = rtU.imu_measurement[2] - (1.0 - (q_idx_1 * q_idx_1 + q_idx_2 *
      q_idx_2) * 2.0);
    s[3] = rtU.imu_measurement[3] - rtDW.x[10];
    s[4] = rtU.imu_measurement[4] - rtDW.x[11];
    s[5] = rtU.imu_measurement[5] - rtDW.x[13];
    s[6] = rtU.imu_measurement[6] - rtDW.x[14];
    s[7] = rtU.imu_measurement[7] - rtDW.x[15];
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 8; m++) {
        h = 0.0;
        for (i = 0; i < 16; i++) {
          h += C[(i << 4) + lastv] * dHdx[(i << 3) + m];
        }

        tmp_0[lastv + (m << 4)] = h;
      }
    }

    for (lastv = 0; lastv < 8; lastv++) {
      for (m = 0; m < 8; m++) {
        a[m + (lastv << 3)] = Ss[(m << 3) + lastv];
      }
    }

    EKFCorrector_correctStateAndSqr(rtb_xNew_k, Ss_1, s, tmp_0, a, dHdx, Rsqrt);
    std::memcpy(&rtDW.P_i[0], &Ss_1[0], sizeof(real_T) << 8U);

    // DataStoreWrite: '<S2>/Data Store WriteX'
    std::memcpy(&rtDW.x[0], &rtb_xNew_k[0], sizeof(real_T) << 4U);
  }

  // End of Inport: '<Root>/enable_imu'
  // End of Outputs for SubSystem: '<S1>/Correct1'

  // Outputs for Enabled SubSystem: '<S1>/Correct2' incorporates:
  //   EnablePort: '<S3>/Enable'

  // Inport: '<Root>/enable_fog'
  if (rtU.enable_fog) {
    // MATLAB Function: '<S3>/Correct' incorporates:
    //   Inport: '<Root>/R_fog'

    rtDW.blockOrdering_n = rtDW.blockOrdering_k;
    if ((!std::isinf(rtU.R_fog)) && (!std::isnan(rtU.R_fog))) {
      h = rtU.R_fog;
      Vf = 1.0;
      if (rtU.R_fog != 0.0) {
        h = std::abs(rtU.R_fog);
      }

      if (h < 0.0) {
        h = -h;
        Vf = -1.0;
      }
    } else {
      h = (rtNaN);
      Vf = (rtNaN);
    }

    // DataStoreWrite: '<S3>/Data Store WriteX' incorporates:
    //   DataStoreWrite: '<S3>/Data Store WriteP'
    //   Inport: '<Root>/fog_measurement'
    //   MATLAB Function: '<S3>/Correct'

    EKFCorrector_correct(rtU.fog_measurement, Vf * std::sqrt(h), rtDW.x,
                         rtDW.P_i);
  }

  // End of Inport: '<Root>/enable_fog'
  // End of Outputs for SubSystem: '<S1>/Correct2'

  // Outputs for Enabled SubSystem: '<S1>/Correct3' incorporates:
  //   EnablePort: '<S4>/Enable'

  // Inport: '<Root>/enable_dvl'
  if (rtU.enable_dvl) {
    // MATLAB Function: '<S4>/Correct' incorporates:
    //   Inport: '<Root>/R_dvl'

    rtDW.blockOrdering_p = rtDW.blockOrdering_n;
    p = true;
    for (m = 0; m < 9; m++) {
      if (p && (std::isinf(rtU.R_dvl[m]) || std::isnan(rtU.R_dvl[m]))) {
        p = false;
      }
    }

    if (p) {
      svd_k(rtU.R_dvl, Ss_0, s_0, a_0);
    } else {
      s_0[0] = (rtNaN);
      s_0[1] = (rtNaN);
      s_0[2] = (rtNaN);
      for (lastv = 0; lastv < 9; lastv++) {
        a_0[lastv] = (rtNaN);
      }
    }

    std::memset(&Ss_0[0], 0, 9U * sizeof(real_T));
    Ss_0[0] = s_0[0];
    Ss_0[4] = s_0[1];
    Ss_0[8] = s_0[2];
    for (m = 0; m <= 6; m += 2) {
      // MATLAB Function: '<S4>/Correct'
      tmp_3 = _mm_loadu_pd(&Ss_0[m]);
      _mm_storeu_pd(&Ss_0[m], _mm_sqrt_pd(tmp_3));
    }

    // MATLAB Function: '<S4>/Correct' incorporates:
    //   DataStoreRead: '<S4>/Data Store ReadX'
    //   DataStoreWrite: '<S4>/Data Store WriteP'
    //   Inport: '<Root>/dvl_measurement'
    //   Inport: '<Root>/dvl_offset'

    for (m = 8; m < 9; m++) {
      Ss_0[m] = std::sqrt(Ss_0[m]);
    }

    for (lastv = 0; lastv < 3; lastv++) {
      h = Ss_0[3 * lastv + 1];
      q_idx_0 = Ss_0[3 * lastv];
      q_idx_1 = Ss_0[3 * lastv + 2];
      for (m = 0; m <= 0; m += 2) {
        tmp_3 = _mm_loadu_pd(&a_0[m + 3]);
        tmp_2 = _mm_loadu_pd(&a_0[m]);
        tmp_1 = _mm_loadu_pd(&a_0[m + 6]);
        _mm_storeu_pd(&Rsqrt_0[m + 3 * lastv], _mm_add_pd(_mm_add_pd(_mm_mul_pd
          (_mm_set1_pd(h), tmp_3), _mm_mul_pd(_mm_set1_pd(q_idx_0), tmp_2)),
          _mm_mul_pd(_mm_set1_pd(q_idx_1), tmp_1)));
      }

      for (m = 2; m < 3; m++) {
        Rsqrt_0[m + 3 * lastv] = (a_0[m + 3] * h + q_idx_0 * a_0[m]) + a_0[m + 6]
          * q_idx_1;
      }
    }

    std::memset(&dHdx_0[0], 0, 48U * sizeof(real_T));
    for (lastv = 0; lastv < 3; lastv++) {
      m = (lastv + 7) * 3;
      dHdx_0[m] = 0.0;
      dHdx_0[m + 1] = 0.0;
      dHdx_0[m + 2] = 0.0;
    }

    dHdx_0[21] = 1.0;
    dHdx_0[25] = 1.0;
    dHdx_0[29] = 1.0;
    dHdx_0[30] = 0.0;
    dHdx_0[33] = rtU.dvl_offset[2];
    dHdx_0[36] = -rtU.dvl_offset[1];
    dHdx_0[31] = -rtU.dvl_offset[2];
    dHdx_0[34] = 0.0;
    dHdx_0[37] = rtU.dvl_offset[0];
    dHdx_0[32] = rtU.dvl_offset[1];
    dHdx_0[35] = -rtU.dvl_offset[0];
    dHdx_0[38] = 0.0;
    for (m = 0; m < 3; m++) {
      coffset = m << 4;
      for (i = 0; i < 16; i++) {
        aoffset = i << 4;
        h = 0.0;
        for (lastv = 0; lastv < 16; lastv++) {
          h += dHdx_0[lastv * 3 + m] * rtDW.P_i[aoffset + lastv];
        }

        y[coffset + i] = h;
      }
    }

    for (lastv = 0; lastv < 16; lastv++) {
      A_0[lastv] = y[lastv];
      A_0[lastv + 19] = y[lastv + 16];
      A_0[lastv + 38] = y[lastv + 32];
    }

    for (i = 0; i < 3; i++) {
      A_0[19 * i + 16] = Rsqrt_0[i];
      A_0[19 * i + 17] = Rsqrt_0[i + 3];
      A_0[19 * i + 18] = Rsqrt_0[i + 6];
      s_0[i] = 0.0;
    }

    for (m = 0; m < 3; m++) {
      coffset = m * 19 + m;
      Vf = A_0[coffset];
      lastv = coffset + 2;
      tau[m] = 0.0;
      h = xnrm2_izk(18 - m, A_0, coffset + 2);
      if (h != 0.0) {
        b_q_idx_0 = A_0[coffset];
        h = rt_hypotd_snf(b_q_idx_0, h);
        if (b_q_idx_0 >= 0.0) {
          h = -h;
        }

        if (std::abs(h) < 1.0020841800044864E-292) {
          i = 0;
          scalarLB = (coffset - m) + 19;
          do {
            i++;
            vectorUB = (((((scalarLB - coffset) - 1) / 2) << 1) + coffset) + 2;
            vectorUB_tmp = vectorUB - 2;
            for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
              tmp_3 = _mm_loadu_pd(&A_0[aoffset - 1]);
              _mm_storeu_pd(&A_0[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd
                (9.9792015476736E+291)));
            }

            for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
              A_0[aoffset - 1] *= 9.9792015476736E+291;
            }

            h *= 9.9792015476736E+291;
            Vf *= 9.9792015476736E+291;
          } while ((std::abs(h) < 1.0020841800044864E-292) && (i < 20));

          h = rt_hypotd_snf(Vf, xnrm2_izk(18 - m, A_0, coffset + 2));
          if (Vf >= 0.0) {
            h = -h;
          }

          tau[m] = (h - Vf) / h;
          Vf = 1.0 / (Vf - h);
          for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
            tmp_3 = _mm_loadu_pd(&A_0[aoffset - 1]);
            _mm_storeu_pd(&A_0[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd(Vf)));
          }

          for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
            A_0[aoffset - 1] *= Vf;
          }

          for (lastv = 0; lastv < i; lastv++) {
            h *= 1.0020841800044864E-292;
          }

          Vf = h;
        } else {
          tau[m] = (h - b_q_idx_0) / h;
          Vf = 1.0 / (b_q_idx_0 - h);
          i = (coffset - m) + 19;
          scalarLB = (((((i - coffset) - 1) / 2) << 1) + coffset) + 2;
          vectorUB = scalarLB - 2;
          for (aoffset = lastv; aoffset <= vectorUB; aoffset += 2) {
            tmp_3 = _mm_loadu_pd(&A_0[aoffset - 1]);
            _mm_storeu_pd(&A_0[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd(Vf)));
          }

          for (aoffset = scalarLB; aoffset <= i; aoffset++) {
            A_0[aoffset - 1] *= Vf;
          }

          Vf = h;
        }
      }

      A_0[coffset] = Vf;
      if (m + 1 < 3) {
        A_0[coffset] = 1.0;
        if (tau[m] != 0.0) {
          lastv = 19 - m;
          i = (coffset - m) + 18;
          while ((lastv > 0) && (A_0[i] == 0.0)) {
            lastv--;
            i--;
          }

          i = 2 - m;
          exitg2 = false;
          while ((!exitg2) && (i > 0)) {
            aoffset = ((i - 1) * 19 + coffset) + 19;
            scalarLB = aoffset;
            do {
              exitg1 = 0;
              if (scalarLB + 1 <= aoffset + lastv) {
                if (A_0[scalarLB] != 0.0) {
                  exitg1 = 1;
                } else {
                  scalarLB++;
                }
              } else {
                i--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          i = 0;
        }

        if (lastv > 0) {
          xgemv_a(lastv, i, A_0, coffset + 20, A_0, coffset + 1, s_0);
          xgerc_n(lastv, i, -tau[m], coffset + 1, s_0, A_0, coffset + 20);
        }

        A_0[coffset] = Vf;
      }
    }

    for (m = 0; m < 3; m++) {
      for (coffset = 0; coffset <= m; coffset++) {
        Ss_0[coffset + 3 * m] = A_0[19 * m + coffset];
      }

      for (coffset = m + 2; coffset < 4; coffset++) {
        Ss_0[(coffset + 3 * m) - 1] = 0.0;
      }
    }

    std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
    std::memcpy(&Ss_1[0], &rtDW.P_i[0], sizeof(real_T) << 8U);
    s_0[0] = rtU.dvl_measurement[0] - ((rtU.dvl_offset[2] * rtDW.x[11] -
      rtU.dvl_offset[1] * rtDW.x[12]) + rtDW.x[7]);
    s_0[1] = rtU.dvl_measurement[1] - ((rtU.dvl_offset[0] * rtDW.x[12] -
      rtU.dvl_offset[2] * rtDW.x[10]) + rtDW.x[8]);
    s_0[2] = rtU.dvl_measurement[2] - ((rtU.dvl_offset[1] * rtDW.x[10] -
      rtU.dvl_offset[0] * rtDW.x[11]) + rtDW.x[9]);
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        h = 0.0;
        for (i = 0; i < 16; i++) {
          aoffset = i << 4;
          h += rtDW.P_i[aoffset + lastv] * rtDW.P_i[aoffset + m];
        }

        C[lastv + (m << 4)] = h;
      }

      for (m = 0; m < 3; m++) {
        h = 0.0;
        for (i = 0; i < 16; i++) {
          h += C[(i << 4) + lastv] * dHdx_0[3 * i + m];
        }

        y[lastv + (m << 4)] = h;
      }
    }

    for (lastv = 0; lastv < 3; lastv++) {
      a_0[3 * lastv] = Ss_0[lastv];
      a_0[3 * lastv + 1] = Ss_0[lastv + 3];
      a_0[3 * lastv + 2] = Ss_0[lastv + 6];
    }

    EKFCorrector_correctStateAndS_l(rtb_xNew_k, Ss_1, s_0, y, a_0, dHdx_0,
      Rsqrt_0);
    std::memcpy(&rtDW.P_i[0], &Ss_1[0], sizeof(real_T) << 8U);

    // DataStoreWrite: '<S4>/Data Store WriteX'
    std::memcpy(&rtDW.x[0], &rtb_xNew_k[0], sizeof(real_T) << 4U);
  }

  // End of Inport: '<Root>/enable_dvl'
  // End of Outputs for SubSystem: '<S1>/Correct3'

  // Outputs for Enabled SubSystem: '<S1>/Correct4' incorporates:
  //   EnablePort: '<S5>/Enable'

  // Inport: '<Root>/enable_depth'
  if (rtU.enable_depth) {
    // MATLAB Function: '<S5>/Correct' incorporates:
    //   Inport: '<Root>/R_depth'

    if ((!std::isinf(rtU.R_depth)) && (!std::isnan(rtU.R_depth))) {
      h = rtU.R_depth;
      Vf = 1.0;
      if (rtU.R_depth != 0.0) {
        h = std::abs(rtU.R_depth);
      }

      if (h < 0.0) {
        h = -h;
        Vf = -1.0;
      }
    } else {
      h = (rtNaN);
      Vf = (rtNaN);
    }

    // DataStoreWrite: '<S5>/Data Store WriteX' incorporates:
    //   DataStoreWrite: '<S5>/Data Store WriteP'
    //   Inport: '<Root>/depth_measurement'
    //   MATLAB Function: '<S5>/Correct'

    EKFCorrector_correct_p(rtU.depth_measurement, Vf * std::sqrt(h), rtDW.x,
      rtDW.P_i);
  }

  // End of Inport: '<Root>/enable_depth'
  // End of Outputs for SubSystem: '<S1>/Correct4'

  // Outputs for Enabled SubSystem: '<S1>/Correct5' incorporates:
  //   EnablePort: '<S6>/Enable'

  // Inport: '<Root>/enable_reset'
  if (rtU.enable_reset) {
    // MATLAB Function: '<S6>/Correct' incorporates:
    //   Inport: '<Root>/R_reset'

    p = true;
    for (m = 0; m < 256; m++) {
      if (p && (std::isinf(rtU.R_reset[m]) || std::isnan(rtU.R_reset[m]))) {
        p = false;
      }
    }

    if (p) {
      svd_d(rtU.R_reset, Ss_1, rtb_xNew_k, K);
    } else {
      for (i = 0; i < 16; i++) {
        rtb_xNew_k[i] = (rtNaN);
      }

      for (lastv = 0; lastv < 256; lastv++) {
        K[lastv] = (rtNaN);
      }
    }

    std::memset(&Ss_1[0], 0, sizeof(real_T) << 8U);
    for (m = 0; m < 16; m++) {
      Ss_1[m + (m << 4)] = rtb_xNew_k[m];
    }

    for (m = 0; m <= 254; m += 2) {
      // MATLAB Function: '<S6>/Correct'
      tmp_3 = _mm_loadu_pd(&Ss_1[m]);
      _mm_storeu_pd(&Ss_1[m], _mm_sqrt_pd(tmp_3));
    }

    // MATLAB Function: '<S6>/Correct' incorporates:
    //   DataStoreWrite: '<S6>/Data Store WriteP'

    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        h = 0.0;
        for (i = 0; i < 16; i++) {
          h += K[(i << 4) + m] * Ss_1[(lastv << 4) + i];
        }

        Rsqrt_1[m + (lastv << 4)] = h;
      }
    }

    std::memcpy(&K[0], &rtDW.P_i[0], sizeof(real_T) << 8U);
    qrFactor(b, K, Rsqrt_1);
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        h = 0.0;
        for (i = 0; i < 16; i++) {
          aoffset = i << 4;
          h += rtDW.P_i[aoffset + lastv] * rtDW.P_i[aoffset + m];
        }

        C[lastv + (m << 4)] = h;
      }
    }

    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        h = 0.0;
        for (i = 0; i < 16; i++) {
          h += C[(i << 4) + m] * b[(lastv << 4) + i];
        }

        Ss_1[lastv + (m << 4)] = h;
      }
    }

    std::memcpy(&C[0], &Ss_1[0], sizeof(real_T) << 8U);
    trisolve_j(K, C);
    for (m = 0; m < 16; m++) {
      std::memcpy(&Ss_1[m << 4], &C[m << 4], sizeof(real_T) << 4U);
      for (coffset = 0; coffset < 16; coffset++) {
        K_0[(m << 4) + coffset] = K[(coffset << 4) + m];
      }
    }

    trisolve_jl(K_0, Ss_1);
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        K[m + (lastv << 4)] = Ss_1[(m << 4) + lastv];
      }
    }

    for (lastv = 0; lastv <= 254; lastv += 2) {
      // MATLAB Function: '<S6>/Correct'
      tmp_3 = _mm_loadu_pd(&K[lastv]);
      _mm_storeu_pd(&K_0[lastv], _mm_mul_pd(tmp_3, _mm_set1_pd(-1.0)));
    }

    // MATLAB Function: '<S6>/Correct' incorporates:
    //   DataStoreWrite: '<S6>/Data Store WriteP'

    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        h = 0.0;
        for (i = 0; i < 16; i++) {
          h += K_0[(i << 4) + m] * b[(lastv << 4) + i];
        }

        Ss_1[m + (lastv << 4)] = h;
      }
    }

    for (i = 0; i < 16; i++) {
      m = (i << 4) + i;
      Ss_1[m]++;
      for (lastv = 0; lastv < 16; lastv++) {
        q_idx_0 = 0.0;
        for (m = 0; m < 16; m++) {
          q_idx_0 += K[(m << 4) + i] * Rsqrt_1[(lastv << 4) + m];
        }

        K_0[i + (lastv << 4)] = q_idx_0;
      }
    }

    qrFactor(Ss_1, rtDW.P_i, K_0);
    for (lastv = 0; lastv <= 14; lastv += 2) {
      // MATLAB Function: '<S6>/Correct' incorporates:
      //   DataStoreRead: '<S6>/Data Store ReadX'
      //   Inport: '<Root>/reset_state'

      tmp_3 = _mm_loadu_pd(&rtDW.x[lastv]);
      _mm_storeu_pd(&tmp[lastv], _mm_sub_pd(_mm_loadu_pd(&rtU.reset_state[lastv]),
        tmp_3));
    }

    // DataStoreWrite: '<S6>/Data Store WriteX' incorporates:
    //   DataStoreRead: '<S6>/Data Store ReadX'
    //   MATLAB Function: '<S6>/Correct'

    for (lastv = 0; lastv < 16; lastv++) {
      h = 0.0;
      for (m = 0; m < 16; m++) {
        h += K[(m << 4) + lastv] * tmp[m];
      }

      rtDW.x[lastv] += h;
    }

    // End of DataStoreWrite: '<S6>/Data Store WriteX'
  }

  // End of Inport: '<Root>/enable_reset'
  // End of Outputs for SubSystem: '<S1>/Correct5'
  for (i = 0; i < 16; i++) {
    // Outport: '<Root>/state' incorporates:
    //   DataStoreRead: '<S7>/Data Store Read'

    rtY.state[i] = rtDW.x[i];

    // Outport: '<Root>/covariance' incorporates:
    //   DataStoreRead: '<S7>/Data Store Read1'
    //   MATLAB Function: '<S7>/MATLAB Function'

    for (lastv = 0; lastv < 16; lastv++) {
      // Outputs for Atomic SubSystem: '<S1>/Output'
      // MATLAB Function: '<S7>/MATLAB Function'
      h = 0.0;

      // End of Outputs for SubSystem: '<S1>/Output'
      for (m = 0; m < 16; m++) {
        // Outputs for Atomic SubSystem: '<S1>/Output'
        // MATLAB Function: '<S7>/MATLAB Function'
        aoffset = m << 4;
        h += rtDW.P_i[aoffset + i] * rtDW.P_i[aoffset + lastv];

        // End of Outputs for SubSystem: '<S1>/Output'
      }

      // Outputs for Atomic SubSystem: '<S1>/Output'
      // MATLAB Function: '<S7>/MATLAB Function' incorporates:
      //   DataStoreRead: '<S7>/Data Store Read1'

      rtY.covariance[i + (lastv << 4)] = h;

      // End of Outputs for SubSystem: '<S1>/Output'
    }

    // End of Outport: '<Root>/covariance'
  }

  // Outputs for Atomic SubSystem: '<S1>/Predict'
  // MATLAB Function: '<S8>/Predict' incorporates:
  //   Inport: '<Root>/Q'

  p = true;
  for (lastv = 0; lastv < 256; lastv++) {
    if (p && (std::isinf(rtU.Q[lastv]) || std::isnan(rtU.Q[lastv]))) {
      p = false;
    }
  }

  if (p) {
    svd_n(rtU.Q, Ss_1, rtb_xNew_k, K);
  } else {
    for (i = 0; i < 16; i++) {
      rtb_xNew_k[i] = (rtNaN);
    }

    for (lastv = 0; lastv < 256; lastv++) {
      K[lastv] = (rtNaN);
    }
  }

  std::memset(&Ss_1[0], 0, sizeof(real_T) << 8U);
  for (m = 0; m < 16; m++) {
    Ss_1[m + (m << 4)] = rtb_xNew_k[m];
  }

  // End of Outputs for SubSystem: '<S1>/Predict'
  for (m = 0; m <= 254; m += 2) {
    // Outputs for Atomic SubSystem: '<S1>/Predict'
    // MATLAB Function: '<S8>/Predict'
    tmp_3 = _mm_loadu_pd(&Ss_1[m]);
    _mm_storeu_pd(&Ss_1[m], _mm_sqrt_pd(tmp_3));

    // End of Outputs for SubSystem: '<S1>/Predict'
  }

  // Outputs for Atomic SubSystem: '<S1>/Predict'
  // MATLAB Function: '<S8>/Predict' incorporates:
  //   DataStoreRead: '<S8>/Data Store ReadX'
  //   DataStoreWrite: '<S8>/Data Store WriteP'
  //   DataStoreWrite: '<S8>/Data Store WriteX'
  //   Inport: '<Root>/dt'

  for (m = 0; m < 16; m++) {
    h = 1.0E-6 * std::fmax(1.0, std::abs(rtDW.x[m]));
    std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
    std::memcpy(&xm[0], &rtDW.x[0], sizeof(real_T) << 4U);
    q_idx_0 = rtDW.x[m];
    rtb_xNew_k[m] = q_idx_0 + h;
    xm[m] = q_idx_0 - h;
    talos_state_transition(rtb_xNew_k, rtU.dt, tmp);
    talos_state_transition(xm, rtU.dt, rtb_xNew_k);
    h *= 2.0;
    for (lastv = 0; lastv <= 14; lastv += 2) {
      tmp_3 = _mm_loadu_pd(&tmp[lastv]);
      tmp_2 = _mm_loadu_pd(&rtb_xNew_k[lastv]);
      _mm_storeu_pd(&Rsqrt_1[lastv + (m << 4)], _mm_div_pd(_mm_sub_pd(tmp_3,
        tmp_2), _mm_set1_pd(h)));
    }
  }

  for (m = 0; m < 16; m++) {
    aoffset = m << 4;
    for (i = 0; i < 16; i++) {
      coffset = i << 4;
      h = 0.0;
      q_idx_0 = 0.0;
      for (lastv = 0; lastv < 16; lastv++) {
        scalarLB = lastv << 4;
        h += Rsqrt_1[scalarLB + m] * rtDW.P_i[coffset + lastv];
        q_idx_0 += K[scalarLB + i] * Ss_1[aoffset + lastv];
      }

      K_0[m + coffset] = q_idx_0;
      C[aoffset + i] = h;
    }
  }

  for (i = 0; i < 16; i++) {
    for (lastv = 0; lastv < 16; lastv++) {
      m = (i << 4) + lastv;
      aoffset = (i << 5) + lastv;
      A_1[aoffset] = C[m];
      A_1[aoffset + 16] = K_0[m];
    }

    xm[i] = 0.0;
  }

  for (m = 0; m < 16; m++) {
    coffset = (m << 5) + m;
    Vf = A_1[coffset];
    lastv = coffset + 2;
    rtb_xNew_k[m] = 0.0;
    h = xnrm2_dzn(31 - m, A_1, coffset + 2);
    if (h != 0.0) {
      b_q_idx_0 = A_1[coffset];
      h = rt_hypotd_snf(b_q_idx_0, h);
      if (b_q_idx_0 >= 0.0) {
        h = -h;
      }

      if (std::abs(h) < 1.0020841800044864E-292) {
        i = 0;
        scalarLB = (coffset - m) + 32;
        do {
          i++;
          vectorUB = (((((scalarLB - coffset) - 1) / 2) << 1) + coffset) + 2;
          vectorUB_tmp = vectorUB - 2;
          for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
            tmp_3 = _mm_loadu_pd(&A_1[aoffset - 1]);
            _mm_storeu_pd(&A_1[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
            A_1[aoffset - 1] *= 9.9792015476736E+291;
          }

          h *= 9.9792015476736E+291;
          Vf *= 9.9792015476736E+291;
        } while ((std::abs(h) < 1.0020841800044864E-292) && (i < 20));

        h = rt_hypotd_snf(Vf, xnrm2_dzn(31 - m, A_1, coffset + 2));
        if (Vf >= 0.0) {
          h = -h;
        }

        rtb_xNew_k[m] = (h - Vf) / h;
        Vf = 1.0 / (Vf - h);
        for (aoffset = lastv; aoffset <= vectorUB_tmp; aoffset += 2) {
          tmp_3 = _mm_loadu_pd(&A_1[aoffset - 1]);
          _mm_storeu_pd(&A_1[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd(Vf)));
        }

        for (aoffset = vectorUB; aoffset <= scalarLB; aoffset++) {
          A_1[aoffset - 1] *= Vf;
        }

        for (aoffset = 0; aoffset < i; aoffset++) {
          h *= 1.0020841800044864E-292;
        }

        Vf = h;
      } else {
        rtb_xNew_k[m] = (h - b_q_idx_0) / h;
        Vf = 1.0 / (b_q_idx_0 - h);
        i = (coffset - m) + 32;
        scalarLB = (((((i - coffset) - 1) / 2) << 1) + coffset) + 2;
        vectorUB = scalarLB - 2;
        for (aoffset = lastv; aoffset <= vectorUB; aoffset += 2) {
          tmp_3 = _mm_loadu_pd(&A_1[aoffset - 1]);
          _mm_storeu_pd(&A_1[aoffset - 1], _mm_mul_pd(tmp_3, _mm_set1_pd(Vf)));
        }

        for (aoffset = scalarLB; aoffset <= i; aoffset++) {
          A_1[aoffset - 1] *= Vf;
        }

        Vf = h;
      }
    }

    A_1[coffset] = Vf;
    if (m + 1 < 16) {
      A_1[coffset] = 1.0;
      if (rtb_xNew_k[m] != 0.0) {
        lastv = 32 - m;
        i = (coffset - m) + 31;
        while ((lastv > 0) && (A_1[i] == 0.0)) {
          lastv--;
          i--;
        }

        i = 15 - m;
        exitg2 = false;
        while ((!exitg2) && (i > 0)) {
          aoffset = (((i - 1) << 5) + coffset) + 32;
          scalarLB = aoffset;
          do {
            exitg1 = 0;
            if (scalarLB + 1 <= aoffset + lastv) {
              if (A_1[scalarLB] != 0.0) {
                exitg1 = 1;
              } else {
                scalarLB++;
              }
            } else {
              i--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        i = 0;
      }

      if (lastv > 0) {
        xgemv_j(lastv, i, A_1, coffset + 33, A_1, coffset + 1, xm);
        xgerc_e(lastv, i, -rtb_xNew_k[m], coffset + 1, xm, A_1, coffset + 33);
      }

      A_1[coffset] = Vf;
    }
  }

  for (m = 0; m < 16; m++) {
    for (coffset = 0; coffset <= m; coffset++) {
      Ss_1[coffset + (m << 4)] = A_1[(m << 5) + coffset];
    }

    for (coffset = m + 2; coffset < 17; coffset++) {
      Ss_1[(coffset + (m << 4)) - 1] = 0.0;
    }
  }

  for (lastv = 0; lastv < 16; lastv++) {
    for (m = 0; m < 16; m++) {
      rtDW.P_i[m + (lastv << 4)] = Ss_1[(m << 4) + lastv];
    }
  }

  std::memcpy(&tmp[0], &rtDW.x[0], sizeof(real_T) << 4U);
  talos_state_transition(tmp, rtU.dt, rtDW.x);

  // End of Outputs for SubSystem: '<S1>/Predict'
}

// Model initialize function
void talos_ekf::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // Start for DataStoreMemory: '<S1>/DataStoreMemory - P'
  std::memcpy(&rtDW.P_i[0], &rtConstP.DataStoreMemoryP_InitialValue[0], sizeof
              (real_T) << 8U);

  // Start for DataStoreMemory: '<S1>/DataStoreMemory - x'
  std::memcpy(&rtDW.x[0], &rtConstP.DataStoreMemoryx_InitialValue[0], sizeof
              (real_T) << 4U);
}

// Constructor
talos_ekf::talos_ekf() :
  rtU(),
  rtY(),
  rtDW(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
talos_ekf::~talos_ekf() = default;

// Real-Time Model get method
talos_ekf::RT_MODEL * talos_ekf::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
