//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: talos_ekf.cpp
//
// Code generated for Simulink model 'talos_ekf'.
//
// Model version                  : 1.6
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Fri Oct  2 20:29:44 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "talos_ekf.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
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
real_T talos_ekf::xnrm2(int32_T n, const real_T x[81], int32_T ix0)
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
real_T talos_ekf::xdotc(int32_T n, const real_T x[81], int32_T ix0, const real_T
  y[81], int32_T iy0)
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
void talos_ekf::xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[81], int32_T
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
real_T talos_ekf::xnrm2_l(int32_T n, const real_T x[9], int32_T ix0)
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
void talos_ekf::xaxpy_n(int32_T n, real_T a, const real_T x[81], int32_T ix0,
  real_T y[9], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xaxpy_ny(int32_T n, real_T a, const real_T x[9], int32_T ix0,
  real_T y[81], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xswap(real_T x[81], int32_T ix0, int32_T iy0)
{
  for (int32_T k{0}; k < 9; k++) {
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
void talos_ekf::xrot(real_T x[81], int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  for (int32_T k{0}; k < 9; k++) {
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
void talos_ekf::svd(const real_T A[81], real_T U[81], real_T s[9], real_T V[81])
{
  real_T b_A[81];
  real_T e[9];
  real_T work[9];
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
  boolean_T apply_transform;
  boolean_T exitg2;
  std::memcpy(&b_A[0], &A[0], 81U * sizeof(real_T));
  std::memset(&s[0], 0, 9U * sizeof(real_T));
  std::memset(&e[0], 0, 9U * sizeof(real_T));
  std::memset(&work[0], 0, 9U * sizeof(real_T));
  std::memset(&U[0], 0, 81U * sizeof(real_T));
  std::memset(&V[0], 0, 81U * sizeof(real_T));
  for (i = 0; i < 8; i++) {
    qp1 = i + 2;
    qq_tmp = 9 * i + i;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = xnrm2(9 - i, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      s[i] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qq_tmp - i) + 9;
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - i) + 9;
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] /= s[i];
        }
      }

      b_A[qq_tmp]++;
      s[i] = -s[i];
    } else {
      s[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 10; qp1jj++) {
      qjj = (qp1jj - 1) * 9 + i;
      if (apply_transform) {
        xaxpy(9 - i, -(xdotc(9 - i, b_A, qq_tmp + 1, b_A, qjj + 1) / b_A[qq_tmp]),
              qq_tmp + 1, b_A, qjj + 1);
      }

      e[qp1jj - 1] = b_A[qjj];
    }

    for (qq = i + 1; qq < 10; qq++) {
      qp1jj = (9 * i + qq) - 1;
      U[qp1jj] = b_A[qp1jj];
    }

    if (i + 1 <= 7) {
      nrm = xnrm2_l(8 - i, e, i + 2);
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
          for (qjj = qp1; qjj < 10; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          for (qjj = qp1; qjj < 10; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 10; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 10; qq++) {
          xaxpy_n(8 - i, e[qq - 1], b_A, (i + 9 * (qq - 1)) + 2, work, i + 2);
        }

        for (qq = qp1; qq < 10; qq++) {
          xaxpy_ny(8 - i, -e[qq - 1] / e[i + 1], work, i + 2, b_A, (i + 9 * (qq
                     - 1)) + 2);
        }
      }

      for (qq = qp1; qq < 10; qq++) {
        V[(qq + 9 * i) - 1] = e[qq - 1];
      }
    }
  }

  i = 7;
  s[8] = b_A[80];
  e[7] = b_A[79];
  e[8] = 0.0;
  std::memset(&U[72], 0, 9U * sizeof(real_T));
  U[80] = 1.0;
  for (qp1 = 7; qp1 >= 0; qp1--) {
    qq = 9 * qp1 + qp1;
    if (s[qp1] != 0.0) {
      for (qp1jj = qp1 + 2; qp1jj < 10; qp1jj++) {
        qjj = ((qp1jj - 1) * 9 + qp1) + 1;
        xaxpy(9 - qp1, -(xdotc(9 - qp1, U, qq + 1, U, qjj) / U[qq]), qq + 1, U,
              qjj);
      }

      for (qjj = qp1 + 1; qjj < 10; qjj++) {
        qp1jj = (9 * qp1 + qjj) - 1;
        U[qp1jj] = -U[qp1jj];
      }

      U[qq]++;
      for (qjj = 0; qjj < qp1; qjj++) {
        U[qjj + 9 * qp1] = 0.0;
      }
    } else {
      std::memset(&U[qp1 * 9], 0, 9U * sizeof(real_T));
      U[qq] = 1.0;
    }
  }

  for (qp1 = 8; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 7) && (e[qp1] != 0.0)) {
      qq = (9 * qp1 + qp1) + 2;
      for (qjj = qp1 + 2; qjj < 10; qjj++) {
        qp1jj = ((qjj - 1) * 9 + qp1) + 2;
        xaxpy(8 - qp1, -(xdotc(8 - qp1, V, qq, V, qp1jj) / V[qq - 1]), qq, V,
              qp1jj);
      }
    }

    std::memset(&V[qp1 * 9], 0, 9U * sizeof(real_T));
    V[qp1 + 9 * qp1] = 1.0;
  }

  for (qp1 = 0; qp1 < 9; qp1++) {
    nrm = s[qp1];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      s[qp1] = rt;
      if (qp1 + 1 < 9) {
        e[qp1] /= nrm;
      }

      qq = 9 * qp1 + 1;
      for (qjj = qq; qjj <= qq + 8; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if (qp1 + 1 < 9) {
      smm1 = e[qp1];
      if (smm1 != 0.0) {
        rt = std::abs(smm1);
        nrm = rt / smm1;
        e[qp1] = rt;
        s[qp1 + 1] *= nrm;
        qq = (qp1 + 1) * 9 + 1;
        for (qjj = qq; qjj <= qq + 8; qjj++) {
          V[qjj - 1] *= nrm;
        }
      }
    }
  }

  qp1 = 0;
  nrm = 0.0;
  for (qq = 0; qq < 9; qq++) {
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
      qq_tmp = i + 2;
      exitg2 = false;
      while ((!exitg2) && (qq_tmp >= qp1jj)) {
        qjj = qq_tmp;
        if (qq_tmp == qp1jj) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (qq_tmp < i + 2) {
            rt = std::abs(e[qq_tmp - 1]);
          }

          if (qq_tmp > qp1jj + 1) {
            rt += std::abs(e[qq_tmp - 2]);
          }

          ztest = std::abs(s[qq_tmp - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            s[qq_tmp - 1] = 0.0;
            exitg2 = true;
          } else {
            qq_tmp--;
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

        xrot(V, 9 * (qjj - 1) + 1, 9 * (i + 1) + 1, ztest, sqds);
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
        xrot(U, 9 * (qjj - 1) + 1, 9 * (qq - 1) + 1, ztest, sqds);
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
        qp1jj = (qjj - 1) * 9 + 1;
        qq_tmp = 9 * qjj + 1;
        xrot(V, qp1jj, qq_tmp, sqds, smm1);
        s[qjj - 1] = rt * sqds + emm1 * smm1;
        xrotg(&s[qjj - 1], &ztest, &sqds, &smm1);
        ztest = e[qjj - 1];
        rt = ztest * sqds + smm1 * s[qjj];
        s[qjj] = ztest * -smm1 + sqds * s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        xrot(U, qp1jj, qq_tmp, sqds, smm1);
      }

      e[i] = rt;
      qp1++;
      break;

     default:
      if (s[qq] < 0.0) {
        s[qq] = -s[qq];
        qp1 = 9 * qq + 1;
        for (qjj = qp1; qjj <= qp1 + 8; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 9) && (s[qq] < s[qp1])) {
        rt = s[qq];
        s[qq] = s[qp1];
        s[qp1] = rt;
        qp1jj = 9 * qq + 1;
        qq_tmp = (qq + 1) * 9 + 1;
        xswap(V, qp1jj, qq_tmp);
        xswap(U, qp1jj, qq_tmp);
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
real_T talos_ekf::xnrm2_ln(int32_T n, const real_T x[225], int32_T ix0)
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
void talos_ekf::xgemv(int32_T m, int32_T n, const real_T A[225], int32_T ia0,
                      const real_T x[225], int32_T ix0, real_T y[9])
{
  if (n != 0) {
    int32_T d;
    std::memset(&y[0], 0, static_cast<uint8_T>(n) * sizeof(real_T));
    d = (n - 1) * 25 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 25) {
      real_T c;
      int32_T b;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x[((ix0 + b) - b_iy) - 1] * A[b - 1];
      }

      b = div_nde_s32_floor(b_iy - ia0, 25);
      y[b] += c;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                      real_T y[9], real_T A[225], int32_T ia0)
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

      jA += 25;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::trisolve(const real_T A[81], real_T B_0[144])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = 9 * j;
    for (int32_T b_k{0}; b_k < 9; b_k++) {
      real_T B_1;
      int32_T B_tmp;
      int32_T kAcol;
      kAcol = 9 * b_k;
      B_tmp = b_k + jBcol;
      B_1 = B_0[B_tmp];
      if (B_1 != 0.0) {
        B_0[B_tmp] = B_1 / A[b_k + kAcol];
        for (int32_T i{b_k + 2}; i < 10; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          B_0[tmp] -= A[(i + kAcol) - 1] * B_0[B_tmp];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::trisolve_b(const real_T A[81], real_T B_2[144])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = 9 * j;
    for (int32_T k{8}; k >= 0; k--) {
      real_T tmp;
      int32_T kAcol;
      int32_T tmp_0;
      kAcol = 9 * k;
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
real_T talos_ekf::xnrm2_lno(int32_T n, const real_T x[400], int32_T ix0)
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
void talos_ekf::xgemv_i(int32_T m, int32_T n, const real_T A[400], int32_T ia0,
  const real_T x[400], int32_T ix0, real_T y[16])
{
  if (n != 0) {
    int32_T d;
    std::memset(&y[0], 0, static_cast<uint8_T>(n) * sizeof(real_T));
    d = (n - 1) * 25 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 25) {
      real_T c;
      int32_T b;
      int32_T e;
      c = 0.0;
      e = (b_iy + m) - 1;
      for (b = b_iy; b <= e; b++) {
        c += x[((ix0 + b) - b_iy) - 1] * A[b - 1];
      }

      b = div_nde_s32_floor(b_iy - ia0, 25);
      y[b] += c;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::xgerc_j(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
  real_T y[16], real_T A[400], int32_T ia0)
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

      jA += 25;
    }
  }
}

// Function for MATLAB Function: '<S2>/Correct'
void talos_ekf::EKFCorrector_correctStateAndSqr(real_T x[16], real_T S[256],
  const real_T residue[9], const real_T Pxy[144], const real_T Sy[81], const
  real_T H[144], const real_T Rsqrt[81])
{
  real_T b_A[400];
  real_T A[256];
  real_T y[256];
  real_T K[144];
  real_T b_C[144];
  real_T Sy_0[81];
  real_T tau[16];
  real_T work[16];
  real_T A_0;
  real_T b_A_0;
  real_T s;
  int32_T aoffset;
  int32_T c_tmp;
  int32_T coffset;
  int32_T exitg1;
  int32_T ii;
  int32_T j;
  int32_T lastv;
  boolean_T exitg2;
  for (ii = 0; ii < 9; ii++) {
    for (j = 0; j < 16; j++) {
      K[ii + 9 * j] = Pxy[(ii << 4) + j];
    }
  }

  trisolve(Sy, K);
  std::memcpy(&b_C[0], &K[0], 144U * sizeof(real_T));
  for (j = 0; j < 9; j++) {
    for (ii = 0; ii < 9; ii++) {
      Sy_0[ii + 9 * j] = Sy[9 * ii + j];
    }
  }

  trisolve_b(Sy_0, b_C);
  for (j = 0; j < 9; j++) {
    for (ii = 0; ii < 16; ii++) {
      K[ii + (j << 4)] = b_C[9 * ii + j];
    }
  }

  for (j = 0; j < 16; j++) {
    A_0 = 0.0;
    for (ii = 0; ii < 9; ii++) {
      A_0 += K[(ii << 4) + j] * residue[ii];
    }

    x[j] += A_0;
  }

  for (j = 0; j < 144; j++) {
    b_C[j] = -K[j];
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii < 16; ii++) {
      A_0 = 0.0;
      for (coffset = 0; coffset < 9; coffset++) {
        A_0 += b_C[(coffset << 4) + ii] * H[9 * j + coffset];
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

  for (j = 0; j < 9; j++) {
    for (ii = 0; ii < 16; ii++) {
      A_0 = 0.0;
      for (coffset = 0; coffset < 9; coffset++) {
        A_0 += K[(coffset << 4) + ii] * Rsqrt[9 * j + coffset];
      }

      b_C[j + 9 * ii] = A_0;
    }
  }

  for (ii = 0; ii < 16; ii++) {
    std::memcpy(&b_A[ii * 25], &y[ii << 4], sizeof(real_T) << 4U);
    std::memcpy(&b_A[ii * 25 + 16], &b_C[ii * 9], 9U * sizeof(real_T));
    work[ii] = 0.0;
  }

  for (j = 0; j < 16; j++) {
    ii = j * 25 + j;
    A_0 = b_A[ii];
    lastv = ii + 2;
    tau[j] = 0.0;
    s = xnrm2_lno(24 - j, b_A, ii + 2);
    if (s != 0.0) {
      b_A_0 = b_A[ii];
      s = rt_hypotd_snf(b_A_0, s);
      if (b_A_0 >= 0.0) {
        s = -s;
      }

      if (std::abs(s) < 1.0020841800044864E-292) {
        coffset = 0;
        c_tmp = (ii - j) + 25;
        do {
          coffset++;
          for (aoffset = lastv; aoffset <= c_tmp; aoffset++) {
            b_A[aoffset - 1] *= 9.9792015476736E+291;
          }

          s *= 9.9792015476736E+291;
          A_0 *= 9.9792015476736E+291;
        } while ((std::abs(s) < 1.0020841800044864E-292) && (coffset < 20));

        s = rt_hypotd_snf(A_0, xnrm2_lno(24 - j, b_A, ii + 2));
        if (A_0 >= 0.0) {
          s = -s;
        }

        tau[j] = (s - A_0) / s;
        A_0 = 1.0 / (A_0 - s);
        for (aoffset = lastv; aoffset <= c_tmp; aoffset++) {
          b_A[aoffset - 1] *= A_0;
        }

        for (lastv = 0; lastv < coffset; lastv++) {
          s *= 1.0020841800044864E-292;
        }

        A_0 = s;
      } else {
        tau[j] = (s - b_A_0) / s;
        A_0 = 1.0 / (b_A_0 - s);
        aoffset = (ii - j) + 25;
        for (coffset = lastv; coffset <= aoffset; coffset++) {
          b_A[coffset - 1] *= A_0;
        }

        A_0 = s;
      }
    }

    b_A[ii] = A_0;
    if (j + 1 < 16) {
      b_A[ii] = 1.0;
      if (tau[j] != 0.0) {
        lastv = 25 - j;
        coffset = (ii - j) + 24;
        while ((lastv > 0) && (b_A[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }

        coffset = 15 - j;
        exitg2 = false;
        while ((!exitg2) && (coffset > 0)) {
          aoffset = ((coffset - 1) * 25 + ii) + 25;
          c_tmp = aoffset;
          do {
            exitg1 = 0;
            if (c_tmp + 1 <= aoffset + lastv) {
              if (b_A[c_tmp] != 0.0) {
                exitg1 = 1;
              } else {
                c_tmp++;
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
        xgemv_i(lastv, coffset, b_A, ii + 26, b_A, ii + 1, work);
        xgerc_j(lastv, coffset, -tau[j], ii + 1, work, b_A, ii + 26);
      }

      b_A[ii] = A_0;
    }
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii <= j; ii++) {
      A[ii + (j << 4)] = b_A[25 * j + ii];
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
real_T talos_ekf::xnrm2_h(int32_T n, const real_T x[9], int32_T ix0)
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

// Function for MATLAB Function: '<S3>/Correct'
real_T talos_ekf::xdotc_j(int32_T n, const real_T x[9], int32_T ix0, const
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xaxpy_d(int32_T n, real_T a, int32_T ix0, real_T y[9], int32_T
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

// Function for MATLAB Function: '<S3>/Correct'
real_T talos_ekf::xnrm2_hm(const real_T x[3], int32_T ix0)
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xaxpy_dp(int32_T n, real_T a, const real_T x[9], int32_T ix0,
  real_T y[3], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xaxpy_dpe(int32_T n, real_T a, const real_T x[3], int32_T ix0,
  real_T y[9], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xswap_o(real_T x[9], int32_T ix0, int32_T iy0)
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xrot_j(real_T x[9], int32_T ix0, int32_T iy0, real_T c, real_T s)
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::svd_a(const real_T A[9], real_T U[9], real_T s[3], real_T V[9])
{
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
    nrm = xnrm2_h(3 - m, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      b_s[m] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qq_tmp - m) + 3;
        for (kase = qq; kase <= qjj; kase++) {
          b_A[kase - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - m) + 3;
        for (kase = qq; kase <= qjj; kase++) {
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
        xaxpy_d(3 - m, -(xdotc_j(3 - m, b_A, qq_tmp + 1, b_A, qjj + 1) /
                         b_A[qq_tmp]), qq_tmp + 1, b_A, qjj + 1);
      }

      e[kase - 1] = b_A[qjj];
    }

    for (qq = m + 1; qq < 4; qq++) {
      kase = (3 * m + qq) - 1;
      U[kase] = b_A[kase];
    }

    if (m + 1 <= 1) {
      nrm = xnrm2_hm(e, 2);
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
          for (qq = qp1; qq < 4; qq++) {
            e[qq - 1] *= nrm;
          }
        } else {
          for (qq = qp1; qq < 4; qq++) {
            e[qq - 1] /= nrm;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (qq = qp1; qq < 4; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 4; qq++) {
          xaxpy_dp(2, e[qq - 1], b_A, 3 * (qq - 1) + 2, work, 2);
        }

        for (qq = qp1; qq < 4; qq++) {
          xaxpy_dpe(2, -e[qq - 1] / e[1], work, 2, b_A, 3 * (qq - 1) + 2);
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
        xaxpy_d(3 - qp1, -(xdotc_j(3 - qp1, U, qq + 1, U, qjj) / U[qq]), qq + 1,
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
      xaxpy_d(2, -(xdotc_j(2, V, 2, V, 5) / V[1]), 2, V, 5);
      xaxpy_d(2, -(xdotc_j(2, V, 2, V, 8) / V[1]), 2, V, 8);
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
      for (qjj = qq; qjj <= qq + 2; qjj++) {
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
        for (qjj = qq; qjj <= qq + 2; qjj++) {
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

        xrot_j(V, 3 * (qjj - 1) + 1, 3 * (m + 1) + 1, ztest, sqds);
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
        xrot_j(U, 3 * (qjj - 1) + 1, 3 * (qq - 1) + 1, ztest, sqds);
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
        xrot_j(V, kase, qq_tmp, sqds, smm1);
        b_s[qjj - 1] = rt * sqds + emm1 * smm1;
        xrotg(&b_s[qjj - 1], &ztest, &sqds, &smm1);
        emm1 = e[qjj - 1];
        rt = emm1 * sqds + smm1 * b_s[qjj];
        b_s[qjj] = emm1 * -smm1 + sqds * b_s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        xrot_j(U, kase, qq_tmp, sqds, smm1);
      }

      e[m] = rt;
      qp1++;
      break;

     default:
      if (b_s[qq] < 0.0) {
        b_s[qq] = -b_s[qq];
        qp1 = 3 * qq + 1;
        for (qjj = qp1; qjj <= qp1 + 2; qjj++) {
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
        xswap_o(V, kase, qq_tmp);
        xswap_o(U, kase, qq_tmp);
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

// Function for MATLAB Function: '<S3>/Correct'
real_T talos_ekf::xnrm2_hmd(int32_T n, const real_T x[57], int32_T ix0)
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xgemv_h(int32_T m, int32_T n, const real_T A[57], int32_T ia0,
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xgerc_f(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::trisolve_i(const real_T A[9], real_T B_3[48])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = 3 * j;
    for (int32_T b_k{0}; b_k < 3; b_k++) {
      real_T B_4;
      int32_T B_tmp;
      int32_T kAcol;
      kAcol = 3 * b_k;
      B_tmp = b_k + jBcol;
      B_4 = B_3[B_tmp];
      if (B_4 != 0.0) {
        B_3[B_tmp] = B_4 / A[b_k + kAcol];
        for (int32_T i{b_k + 2}; i < 4; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          B_3[tmp] -= A[(i + kAcol) - 1] * B_3[B_tmp];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::trisolve_if(const real_T A[9], real_T B_5[48])
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
      tmp = B_5[tmp_0];
      if (tmp != 0.0) {
        B_5[tmp_0] = tmp / A[k + kAcol];
        for (int32_T i{0}; i < k; i++) {
          int32_T tmp_1;
          tmp_1 = i + jBcol;
          B_5[tmp_1] -= A[i + kAcol] * B_5[tmp_0];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S3>/Correct'
real_T talos_ekf::xnrm2_hmda(int32_T n, const real_T x[304], int32_T ix0)
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xgemv_ht(int32_T m, int32_T n, const real_T A[304], int32_T ia0,
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::xgerc_f0(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
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

// Function for MATLAB Function: '<S3>/Correct'
void talos_ekf::EKFCorrector_correctStateAndS_n(real_T x[16], real_T S[256],
  const real_T residue[3], const real_T Pxy[48], const real_T Sy[9], const
  real_T H[48], const real_T Rsqrt[9])
{
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
  int32_T c_tmp;
  int32_T coffset;
  int32_T exitg1;
  int32_T ii;
  int32_T j;
  int32_T lastv;
  boolean_T exitg2;
  for (j = 0; j < 16; j++) {
    K[3 * j] = Pxy[j];
    K[3 * j + 1] = Pxy[j + 16];
    K[3 * j + 2] = Pxy[j + 32];
  }

  trisolve_i(Sy, K);
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

  trisolve_if(Sy_0, b_C);
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

  for (coffset = 0; coffset < 48; coffset++) {
    b_C[coffset] = -K[coffset];
  }

  for (coffset = 0; coffset < 16; coffset++) {
    K_0 = H[3 * coffset + 1];
    residue_0 = H[3 * coffset];
    s = H[3 * coffset + 2];
    for (j = 0; j < 16; j++) {
      A[j + (coffset << 4)] = (b_C[j + 16] * K_0 + residue_0 * b_C[j]) + b_C[j +
        32] * s;
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
    s = xnrm2_hmda(18 - j, b_A, ii + 2);
    if (s != 0.0) {
      residue_0 = b_A[ii];
      s = rt_hypotd_snf(residue_0, s);
      if (residue_0 >= 0.0) {
        s = -s;
      }

      if (std::abs(s) < 1.0020841800044864E-292) {
        coffset = 0;
        c_tmp = (ii - j) + 19;
        do {
          coffset++;
          for (aoffset = lastv; aoffset <= c_tmp; aoffset++) {
            b_A[aoffset - 1] *= 9.9792015476736E+291;
          }

          s *= 9.9792015476736E+291;
          K_0 *= 9.9792015476736E+291;
        } while ((std::abs(s) < 1.0020841800044864E-292) && (coffset < 20));

        s = rt_hypotd_snf(K_0, xnrm2_hmda(18 - j, b_A, ii + 2));
        if (K_0 >= 0.0) {
          s = -s;
        }

        tau[j] = (s - K_0) / s;
        K_0 = 1.0 / (K_0 - s);
        for (aoffset = lastv; aoffset <= c_tmp; aoffset++) {
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
        for (coffset = lastv; coffset <= aoffset; coffset++) {
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
          c_tmp = aoffset;
          do {
            exitg1 = 0;
            if (c_tmp + 1 <= aoffset + lastv) {
              if (b_A[c_tmp] != 0.0) {
                exitg1 = 1;
              } else {
                c_tmp++;
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
        xgemv_ht(lastv, coffset, b_A, ii + 20, b_A, ii + 1, work);
        xgerc_f0(lastv, coffset, -tau[j], ii + 1, work, b_A, ii + 20);
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
real_T talos_ekf::xnrm2_n(int32_T n, const real_T x[17], int32_T ix0)
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

// Function for MATLAB Function: '<S5>/Correct'
void talos_ekf::trisolve_c(real_T A, real_T B_9[16])
{
  for (int32_T j{0}; j < 16; j++) {
    real_T B_a;
    B_a = B_9[j];
    if (B_a != 0.0) {
      B_9[j] = B_a / A;
    }
  }
}

// Function for MATLAB Function: '<S5>/Correct'
real_T talos_ekf::xnrm2_nd(int32_T n, const real_T x[272], int32_T ix0)
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

// Function for MATLAB Function: '<S5>/Correct'
void talos_ekf::xgemv_e(int32_T m, int32_T n, const real_T A[272], int32_T ia0,
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

// Function for MATLAB Function: '<S5>/Correct'
void talos_ekf::xgerc_n1(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
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

// Function for MATLAB Function: '<S5>/Correct'
void talos_ekf::EKFCorrector_correctStateAndS_h(real_T x[16], real_T S[256],
  real_T residue, const real_T Pxy[16], real_T Sy, const real_T H[16], real_T
  Rsqrt)
{
  real_T b_A[272];
  real_T A[256];
  real_T C[16];
  real_T K[16];
  real_T K_0;
  real_T b_A_0;
  real_T s;
  int32_T aoffset;
  int32_T c_tmp;
  int32_T coffset;
  int32_T exitg1;
  int32_T ii;
  int32_T j;
  int32_T lastv;
  boolean_T exitg2;
  std::memcpy(&C[0], &Pxy[0], sizeof(real_T) << 4U);
  trisolve_c(Sy, C);
  std::memcpy(&K[0], &C[0], sizeof(real_T) << 4U);
  trisolve_c(Sy, K);
  for (j = 0; j < 16; j++) {
    K_0 = K[j];
    x[j] += K_0 * residue;
    C[j] = -K_0;
  }

  for (j = 0; j < 16; j++) {
    for (ii = 0; ii < 16; ii++) {
      A[ii + (j << 4)] = C[ii] * H[j];
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
    K_0 = b_A[ii];
    lastv = ii + 2;
    C[j] = 0.0;
    s = xnrm2_nd(16 - j, b_A, ii + 2);
    if (s != 0.0) {
      b_A_0 = b_A[ii];
      s = rt_hypotd_snf(b_A_0, s);
      if (b_A_0 >= 0.0) {
        s = -s;
      }

      if (std::abs(s) < 1.0020841800044864E-292) {
        coffset = 0;
        c_tmp = (ii - j) + 17;
        do {
          coffset++;
          for (aoffset = lastv; aoffset <= c_tmp; aoffset++) {
            b_A[aoffset - 1] *= 9.9792015476736E+291;
          }

          s *= 9.9792015476736E+291;
          K_0 *= 9.9792015476736E+291;
        } while ((std::abs(s) < 1.0020841800044864E-292) && (coffset < 20));

        s = rt_hypotd_snf(K_0, xnrm2_nd(16 - j, b_A, ii + 2));
        if (K_0 >= 0.0) {
          s = -s;
        }

        C[j] = (s - K_0) / s;
        K_0 = 1.0 / (K_0 - s);
        for (aoffset = lastv; aoffset <= c_tmp; aoffset++) {
          b_A[aoffset - 1] *= K_0;
        }

        for (lastv = 0; lastv < coffset; lastv++) {
          s *= 1.0020841800044864E-292;
        }

        K_0 = s;
      } else {
        C[j] = (s - b_A_0) / s;
        K_0 = 1.0 / (b_A_0 - s);
        aoffset = (ii - j) + 17;
        for (coffset = lastv; coffset <= aoffset; coffset++) {
          b_A[coffset - 1] *= K_0;
        }

        K_0 = s;
      }
    }

    b_A[ii] = K_0;
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
          c_tmp = aoffset;
          do {
            exitg1 = 0;
            if (c_tmp + 1 <= aoffset + lastv) {
              if (b_A[c_tmp] != 0.0) {
                exitg1 = 1;
              } else {
                c_tmp++;
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
        xgemv_e(lastv, coffset, b_A, ii + 18, b_A, ii + 1, K);
        xgerc_n1(lastv, coffset, -C[j], ii + 1, K, b_A, ii + 18);
      }

      b_A[ii] = K_0;
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

// Function for MATLAB Function: '<S5>/Correct'
void talos_ekf::EKFCorrector_correct(real_T z, real_T Rs, real_T x[16], real_T
  S[256], real_T varargin_1)
{
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
    dHdx[f_k] = (varargin_1 * b_x[2] - varargin_1 * c_x[2]) / (2.0 * h);
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
  h = xnrm2_n(16, A, 2);
  if (h != 0.0) {
    h = rt_hypotd_snf(A[0], h);
    if (A[0] >= 0.0) {
      h = -h;
    }

    if (std::abs(h) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (i = 0; i < 16; i++) {
          A[i + 1] *= 9.9792015476736E+291;
        }

        h *= 9.9792015476736E+291;
        x_0 *= 9.9792015476736E+291;
      } while ((std::abs(h) < 1.0020841800044864E-292) && (knt < 20));

      h = rt_hypotd_snf(x_0, xnrm2_n(16, A, 2));
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

  EKFCorrector_correctStateAndS_h(x, S, z - varargin_1 * x[2], c_x, x_0, dHdx,
    Rs);
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
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::xaxpy_ckp(int32_T n, real_T a, const real_T x[16], int32_T ix0,
  real_T y[256], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
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
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - i) + 16;
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
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
          for (qjj = qp1; qjj < 17; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          for (qjj = qp1; qjj < 17; qjj++) {
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
      for (qjj = qq; qjj <= qq + 15; qjj++) {
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
        for (qjj = qq; qjj <= qq + 15; qjj++) {
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
        for (qjj = qp1; qjj <= qp1 + 15; qjj++) {
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
  real_T b_A[512];
  real_T y[256];
  real_T tau[16];
  real_T work[16];
  real_T atmp;
  real_T b_A_0;
  real_T s;
  int32_T aoffset;
  int32_T c_tmp;
  int32_T coffset;
  int32_T exitg1;
  int32_T ii;
  int32_T j;
  int32_T lastv;
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
        c_tmp = (ii - j) + 32;
        do {
          coffset++;
          for (aoffset = lastv; aoffset <= c_tmp; aoffset++) {
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
        for (aoffset = lastv; aoffset <= c_tmp; aoffset++) {
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
        for (coffset = lastv; coffset <= aoffset; coffset++) {
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
          c_tmp = aoffset;
          do {
            exitg1 = 0;
            if (c_tmp + 1 <= aoffset + lastv) {
              if (b_A[c_tmp] != 0.0) {
                exitg1 = 1;
              } else {
                c_tmp++;
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
void talos_ekf::trisolve_j(const real_T A[256], real_T B_b[256])
{
  for (int32_T j{0}; j < 16; j++) {
    int32_T jBcol;
    jBcol = j << 4;
    for (int32_T b_k{0}; b_k < 16; b_k++) {
      real_T B_c;
      int32_T B_tmp;
      int32_T kAcol;
      kAcol = b_k << 4;
      B_tmp = b_k + jBcol;
      B_c = B_b[B_tmp];
      if (B_c != 0.0) {
        B_b[B_tmp] = B_c / A[b_k + kAcol];
        for (int32_T i{b_k + 2}; i < 17; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          B_b[tmp] -= A[(i + kAcol) - 1] * B_b[B_tmp];
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S6>/Correct'
void talos_ekf::trisolve_jl(const real_T A[256], real_T B_d[256])
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
      tmp = B_d[tmp_0];
      if (tmp != 0.0) {
        B_d[tmp_0] = tmp / A[k + kAcol];
        for (int32_T i{0}; i < k; i++) {
          int32_T tmp_1;
          tmp_1 = i + jBcol;
          B_d[tmp_1] -= A[i + kAcol] * B_d[tmp_0];
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
void talos_ekf::xaxpy_d2(int32_T n, real_T a, int32_T ix0, real_T y[256],
  int32_T iy0)
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
void talos_ekf::xaxpy_d2s(int32_T n, real_T a, const real_T x[256], int32_T ix0,
  real_T y[16], int32_T iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::xaxpy_d2sz(int32_T n, real_T a, const real_T x[16], int32_T ix0,
  real_T y[256], int32_T iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    for (int32_T k{0}; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

// Function for MATLAB Function: '<S8>/Predict'
void talos_ekf::svd_n(const real_T A[256], real_T U[256], real_T s[16], real_T
                      V[256])
{
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
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - i) + 16;
        for (qp1jj = qq; qp1jj <= qjj; qp1jj++) {
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
        xaxpy_d2(16 - i, -(xdotc_e(16 - i, b_A, qq_tmp + 1, b_A, qjj + 1) /
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
          for (qjj = qp1; qjj < 17; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          for (qjj = qp1; qjj < 17; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 17; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 17; qq++) {
          xaxpy_d2s(15 - i, e[qq - 1], b_A, (i + ((qq - 1) << 4)) + 2, work, i +
                    2);
        }

        for (qq = qp1; qq < 17; qq++) {
          xaxpy_d2sz(15 - i, -e[qq - 1] / e[i + 1], work, i + 2, b_A, (i + ((qq
            - 1) << 4)) + 2);
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
        xaxpy_d2(16 - qp1, -(xdotc_e(16 - qp1, U, qq + 1, U, qjj) / U[qq]), qq +
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
        xaxpy_d2(15 - qp1, -(xdotc_e(15 - qp1, V, qq, V, qp1jj) / V[qq - 1]), qq,
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
      for (qjj = qq; qjj <= qq + 15; qjj++) {
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
        for (qjj = qq; qjj <= qq + 15; qjj++) {
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
        for (qjj = qp1; qjj <= qp1 + 15; qjj++) {
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
  for (int32_T i{0}; i < 3; i++) {
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

  real_T A_1[512];
  real_T C[256];
  real_T K[256];
  real_T K_0[256];
  real_T Rsqrt_0[256];
  real_T Ss_0[256];
  real_T A[225];
  real_T dHdx[144];
  real_T tmp_1[144];
  real_T Rsqrt[81];
  real_T Ss[81];
  real_T a[81];
  real_T A_0[57];
  real_T dHdx_0[48];
  real_T y[48];
  real_T rtb_xNew_k[16];
  real_T tmp_0[16];
  real_T xm[16];
  real_T s_2[12];
  real_T tmp[12];
  real_T s[9];
  real_T s_1[9];
  real_T work[9];
  real_T qRaw[4];
  real_T x_tmp[4];
  real_T s_0[3];
  real_T work_0[3];
  real_T Vf;
  real_T qNorm;
  real_T s_3;
  real_T s_4;
  real_T s_5;
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T i;
  int32_T i_k;
  int32_T i_k_tmp;
  int32_T lastv;
  int32_T m;
  int8_T b_I[16];
  boolean_T exitg2;
  boolean_T p;

  // Outputs for Enabled SubSystem: '<S1>/Correct1' incorporates:
  //   EnablePort: '<S2>/Enable'

  // Inport: '<Root>/enable_imu'
  if (rtU.enable_imu) {
    // MATLAB Function: '<S2>/Correct' incorporates:
    //   Constant: '<S1>/BlockOrdering'
    //   DataStoreRead: '<S2>/Data Store ReadX'
    //   DataStoreWrite: '<S2>/Data Store WriteP'
    //   Inport: '<Root>/R_imu'
    //   Inport: '<Root>/imu_context'
    //   Inport: '<Root>/imu_measurement'

    rtDW.blockOrdering_k = true;
    p = true;
    for (m = 0; m < 81; m++) {
      if (p && (std::isinf(rtU.R_imu[m]) || std::isnan(rtU.R_imu[m]))) {
        p = false;
      }
    }

    if (p) {
      svd(rtU.R_imu, Ss, s, a);
    } else {
      for (i = 0; i < 9; i++) {
        s[i] = (rtNaN);
      }

      for (lastv = 0; lastv < 81; lastv++) {
        a[lastv] = (rtNaN);
      }
    }

    std::memset(&Ss[0], 0, 81U * sizeof(real_T));
    for (m = 0; m < 9; m++) {
      Ss[m + 9 * m] = s[m];
    }

    for (m = 0; m < 81; m++) {
      Ss[m] = std::sqrt(Ss[m]);
    }

    for (lastv = 0; lastv < 9; lastv++) {
      for (m = 0; m < 9; m++) {
        qNorm = 0.0;
        for (i = 0; i < 9; i++) {
          qNorm += a[9 * i + m] * Ss[9 * lastv + i];
        }

        Rsqrt[m + 9 * lastv] = qNorm;
      }
    }

    x_tmp[0] = rtDW.x[3] * rtDW.x[3];
    x_tmp[1] = rtDW.x[4] * rtDW.x[4];
    x_tmp[2] = rtDW.x[5] * rtDW.x[5];
    x_tmp[3] = rtDW.x[6] * rtDW.x[6];
    qNorm = std::fmax(std::sqrt(((x_tmp[0] + x_tmp[1]) + x_tmp[2]) + x_tmp[3]),
                      1.0E-12);
    qRaw[0] = rtDW.x[3] / qNorm;
    qRaw[1] = rtDW.x[4] / qNorm;
    qRaw[2] = rtDW.x[5] / qNorm;
    qRaw[3] = rtDW.x[6] / qNorm;
    for (lastv = 0; lastv < 16; lastv++) {
      b_I[lastv] = 0;
    }

    b_I[0] = 1;
    b_I[5] = 1;
    b_I[10] = 1;
    b_I[15] = 1;
    std::memset(&dHdx[0], 0, 144U * sizeof(real_T));
    std::memset(&s[0], 0, 9U * sizeof(real_T));
    s[0] = rtU.imu_context[3];
    s[4] = rtU.imu_context[4];
    s[8] = rtU.imu_context[5];
    work[0] = 0.0;
    work[3] = -rtU.imu_context[2];
    work[6] = rtU.imu_context[1];
    work[1] = rtU.imu_context[2];
    work[4] = 0.0;
    work[7] = -rtU.imu_context[0];
    work[2] = -rtU.imu_context[1];
    work[5] = rtU.imu_context[0];
    work[8] = 0.0;
    tmp[0] = -2.0 * qRaw[2];
    tmp[3] = 2.0 * qRaw[3];
    tmp[6] = -2.0 * qRaw[0];
    tmp[9] = 2.0 * qRaw[1];
    tmp[1] = 2.0 * qRaw[1];
    tmp[4] = 2.0 * qRaw[0];
    tmp[7] = 2.0 * qRaw[3];
    tmp[10] = 2.0 * qRaw[2];
    tmp[2] = 0.0;
    tmp[5] = -4.0 * qRaw[1];
    tmp[8] = -4.0 * qRaw[2];
    tmp[11] = 0.0;
    for (lastv = 0; lastv < 3; lastv++) {
      Vf = s[lastv + 3];
      s_4 = s[lastv];
      s_5 = s[lastv + 6];
      for (m = 0; m < 3; m++) {
        s_1[lastv + 3 * m] = (work[3 * m + 1] * Vf + work[3 * m] * s_4) + work[3
          * m + 2] * s_5;
      }

      Vf = s_1[lastv + 3];
      s_4 = s_1[lastv];
      s_5 = s_1[lastv + 6];
      for (m = 0; m < 4; m++) {
        s_2[lastv + 3 * m] = (tmp[3 * m + 1] * Vf + tmp[3 * m] * s_4) + tmp[3 *
          m + 2] * s_5;
      }
    }

    for (lastv = 0; lastv < 4; lastv++) {
      m = lastv << 2;
      xm[m] = (static_cast<real_T>(b_I[m]) - qRaw[0] * qRaw[lastv]) / qNorm;
      xm[m + 1] = (static_cast<real_T>(b_I[m + 1]) - qRaw[1] * qRaw[lastv]) /
        qNorm;
      xm[m + 2] = (static_cast<real_T>(b_I[m + 2]) - qRaw[2] * qRaw[lastv]) /
        qNorm;
      xm[m + 3] = (static_cast<real_T>(b_I[m + 3]) - qRaw[3] * qRaw[lastv]) /
        qNorm;
    }

    dHdx[93] = rtU.imu_context[6];
    dHdx[103] = rtU.imu_context[7];
    dHdx[113] = rtU.imu_context[8];
    dHdx[123] = rtU.imu_context[9];
    dHdx[133] = rtU.imu_context[10];
    dHdx[143] = rtU.imu_context[11];
    qNorm = x_tmp[0];
    for (m = 0; m < 3; m++) {
      Vf = s_2[m + 3];
      s_4 = s_2[m];
      s_5 = s_2[m + 6];
      s_3 = s_2[m + 9];
      for (lastv = 0; lastv < 4; lastv++) {
        i = lastv << 2;
        dHdx[m + 9 * (lastv + 3)] = ((xm[i + 1] * Vf + xm[i] * s_4) + xm[i + 2] *
          s_5) + xm[i + 3] * s_3;
      }

      qNorm += x_tmp[m + 1];
    }

    qNorm = std::fmax(std::sqrt(qNorm), 1.0E-12);
    qRaw[0] = rtDW.x[3] / qNorm;
    qRaw[1] = rtDW.x[4] / qNorm;
    qRaw[2] = rtDW.x[5] / qNorm;
    qRaw[3] = rtDW.x[6] / qNorm;
    work_0[0] = (qRaw[1] * qRaw[3] - qRaw[0] * qRaw[2]) * 2.0;
    work_0[1] = (qRaw[2] * qRaw[3] + qRaw[0] * qRaw[1]) * 2.0;
    work_0[2] = 1.0 - (qRaw[1] * qRaw[1] + qRaw[2] * qRaw[2]) * 2.0;
    for (m = 0; m < 9; m++) {
      for (i = 0; i < 16; i++) {
        aoffset = i << 4;
        qNorm = 0.0;
        for (lastv = 0; lastv < 16; lastv++) {
          qNorm += dHdx[lastv * 9 + m] * rtDW.P_i[aoffset + lastv];
        }

        A[i + 25 * m] = qNorm;
      }

      for (lastv = 0; lastv < 9; lastv++) {
        A[(lastv + 25 * m) + 16] = Rsqrt[9 * lastv + m];
      }

      work[m] = 0.0;
    }

    for (m = 0; m < 9; m++) {
      coffset = m * 25 + m;
      Vf = A[coffset];
      lastv = coffset + 2;
      s[m] = 0.0;
      qNorm = xnrm2_ln(24 - m, A, coffset + 2);
      if (qNorm != 0.0) {
        s_4 = A[coffset];
        qNorm = rt_hypotd_snf(s_4, qNorm);
        if (s_4 >= 0.0) {
          qNorm = -qNorm;
        }

        if (std::abs(qNorm) < 1.0020841800044864E-292) {
          i = 0;
          i_k_tmp = (coffset - m) + 25;
          do {
            i++;
            for (aoffset = lastv; aoffset <= i_k_tmp; aoffset++) {
              A[aoffset - 1] *= 9.9792015476736E+291;
            }

            qNorm *= 9.9792015476736E+291;
            Vf *= 9.9792015476736E+291;
          } while ((std::abs(qNorm) < 1.0020841800044864E-292) && (i < 20));

          qNorm = rt_hypotd_snf(Vf, xnrm2_ln(24 - m, A, coffset + 2));
          if (Vf >= 0.0) {
            qNorm = -qNorm;
          }

          s[m] = (qNorm - Vf) / qNorm;
          Vf = 1.0 / (Vf - qNorm);
          for (i_k = lastv; i_k <= i_k_tmp; i_k++) {
            A[i_k - 1] *= Vf;
          }

          for (lastv = 0; lastv < i; lastv++) {
            qNorm *= 1.0020841800044864E-292;
          }

          Vf = qNorm;
        } else {
          s[m] = (qNorm - s_4) / qNorm;
          Vf = 1.0 / (s_4 - qNorm);
          i = (coffset - m) + 25;
          for (aoffset = lastv; aoffset <= i; aoffset++) {
            A[aoffset - 1] *= Vf;
          }

          Vf = qNorm;
        }
      }

      A[coffset] = Vf;
      if (m + 1 < 9) {
        A[coffset] = 1.0;
        if (s[m] != 0.0) {
          lastv = 25 - m;
          i = (coffset - m) + 24;
          while ((lastv > 0) && (A[i] == 0.0)) {
            lastv--;
            i--;
          }

          i = 8 - m;
          exitg2 = false;
          while ((!exitg2) && (i > 0)) {
            aoffset = ((i - 1) * 25 + coffset) + 25;
            i_k = aoffset;
            do {
              exitg1 = 0;
              if (i_k + 1 <= aoffset + lastv) {
                if (A[i_k] != 0.0) {
                  exitg1 = 1;
                } else {
                  i_k++;
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
          xgemv(lastv, i, A, coffset + 26, A, coffset + 1, work);
          xgerc(lastv, i, -s[m], coffset + 1, work, A, coffset + 26);
        }

        A[coffset] = Vf;
      }
    }

    for (m = 0; m < 9; m++) {
      for (coffset = 0; coffset <= m; coffset++) {
        Ss[coffset + 9 * m] = A[25 * m + coffset];
      }

      for (coffset = m + 2; coffset < 10; coffset++) {
        Ss[(coffset + 9 * m) - 1] = 0.0;
      }
    }

    std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
    std::memcpy(&Ss_0[0], &rtDW.P_i[0], sizeof(real_T) << 8U);
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          coffset = i << 4;
          qNorm += rtDW.P_i[coffset + lastv] * rtDW.P_i[coffset + m];
        }

        C[lastv + (m << 4)] = qNorm;
      }
    }

    work[0] = rtU.imu_measurement[0] - (rtU.imu_context[1] * work_0[2] - work_0
      [1] * rtU.imu_context[2]) * rtU.imu_context[3];
    work[1] = rtU.imu_measurement[1] - (work_0[0] * rtU.imu_context[2] -
      rtU.imu_context[0] * work_0[2]) * rtU.imu_context[4];
    work[2] = rtU.imu_measurement[2] - (rtU.imu_context[0] * work_0[1] - work_0
      [0] * rtU.imu_context[1]) * rtU.imu_context[5];
    work[3] = rtU.imu_measurement[3] - rtU.imu_context[6] * rtDW.x[10];
    work[6] = rtU.imu_measurement[6] - rtU.imu_context[9] * rtDW.x[13];
    work[4] = rtU.imu_measurement[4] - rtU.imu_context[7] * rtDW.x[11];
    work[7] = rtU.imu_measurement[7] - rtU.imu_context[10] * rtDW.x[14];
    work[5] = rtU.imu_measurement[5] - rtU.imu_context[8] * rtDW.x[12];
    work[8] = rtU.imu_measurement[8] - rtU.imu_context[11] * rtDW.x[15];
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 9; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          qNorm += C[(i << 4) + lastv] * dHdx[9 * i + m];
        }

        tmp_1[lastv + (m << 4)] = qNorm;
      }
    }

    for (lastv = 0; lastv < 9; lastv++) {
      for (m = 0; m < 9; m++) {
        a[m + 9 * lastv] = Ss[9 * m + lastv];
      }
    }

    EKFCorrector_correctStateAndSqr(rtb_xNew_k, Ss_0, work, tmp_1, a, dHdx,
      Rsqrt);
    std::memcpy(&rtDW.P_i[0], &Ss_0[0], sizeof(real_T) << 8U);

    // End of MATLAB Function: '<S2>/Correct'

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
    //   DataStoreRead: '<S3>/Data Store ReadX'
    //   DataStoreWrite: '<S3>/Data Store WriteP'
    //   Inport: '<Root>/R_fog'
    //   Inport: '<Root>/fog_mask'
    //   Inport: '<Root>/fog_measurement'

    rtDW.blockOrdering_n = rtDW.blockOrdering_k;
    p = true;
    for (m = 0; m < 9; m++) {
      if (p && (std::isinf(rtU.R_fog[m]) || std::isnan(rtU.R_fog[m]))) {
        p = false;
      }
    }

    if (p) {
      svd_a(rtU.R_fog, work, s_0, s);
    } else {
      s_0[0] = (rtNaN);
      s_0[1] = (rtNaN);
      s_0[2] = (rtNaN);
      for (i = 0; i < 9; i++) {
        s[i] = (rtNaN);
      }
    }

    std::memset(&work[0], 0, 9U * sizeof(real_T));
    work[0] = s_0[0];
    work[4] = s_0[1];
    work[8] = s_0[2];
    for (m = 0; m < 9; m++) {
      work[m] = std::sqrt(work[m]);
    }

    for (lastv = 0; lastv < 3; lastv++) {
      qNorm = work[3 * lastv + 1];
      Vf = work[3 * lastv];
      s_4 = work[3 * lastv + 2];
      for (m = 0; m < 3; m++) {
        s_1[m + 3 * lastv] = (s[m + 3] * qNorm + Vf * s[m]) + s[m + 6] * s_4;
      }
    }

    for (i_k = 0; i_k < 16; i_k++) {
      qNorm = 1.0E-6 * std::fmax(1.0, std::abs(rtDW.x[i_k]));
      std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
      std::memcpy(&xm[0], &rtDW.x[0], sizeof(real_T) << 4U);
      Vf = rtDW.x[i_k];
      rtb_xNew_k[i_k] = Vf + qNorm;
      xm[i_k] = Vf - qNorm;
      qNorm *= 2.0;
      dHdx_0[3 * i_k] = (rtU.fog_mask[0] * rtb_xNew_k[10] - rtU.fog_mask[0] *
                         xm[10]) / qNorm;
      dHdx_0[3 * i_k + 1] = (rtU.fog_mask[1] * rtb_xNew_k[11] - rtU.fog_mask[1] *
        xm[11]) / qNorm;
      dHdx_0[3 * i_k + 2] = (rtU.fog_mask[2] * rtb_xNew_k[12] - rtU.fog_mask[2] *
        xm[12]) / qNorm;
    }

    for (m = 0; m < 3; m++) {
      coffset = m << 4;
      for (i = 0; i < 16; i++) {
        aoffset = i << 4;
        qNorm = 0.0;
        for (lastv = 0; lastv < 16; lastv++) {
          qNorm += dHdx_0[lastv * 3 + m] * rtDW.P_i[aoffset + lastv];
        }

        y[coffset + i] = qNorm;
      }
    }

    for (lastv = 0; lastv < 16; lastv++) {
      A_0[lastv] = y[lastv];
      A_0[lastv + 19] = y[lastv + 16];
      A_0[lastv + 38] = y[lastv + 32];
    }

    for (i = 0; i < 3; i++) {
      A_0[19 * i + 16] = s_1[i];
      A_0[19 * i + 17] = s_1[i + 3];
      A_0[19 * i + 18] = s_1[i + 6];
      work_0[i] = 0.0;
    }

    for (m = 0; m < 3; m++) {
      coffset = m * 19 + m;
      Vf = A_0[coffset];
      lastv = coffset + 2;
      s_0[m] = 0.0;
      qNorm = xnrm2_hmd(18 - m, A_0, coffset + 2);
      if (qNorm != 0.0) {
        s_4 = A_0[coffset];
        qNorm = rt_hypotd_snf(s_4, qNorm);
        if (s_4 >= 0.0) {
          qNorm = -qNorm;
        }

        if (std::abs(qNorm) < 1.0020841800044864E-292) {
          i = 0;
          i_k_tmp = (coffset - m) + 19;
          do {
            i++;
            for (aoffset = lastv; aoffset <= i_k_tmp; aoffset++) {
              A_0[aoffset - 1] *= 9.9792015476736E+291;
            }

            qNorm *= 9.9792015476736E+291;
            Vf *= 9.9792015476736E+291;
          } while ((std::abs(qNorm) < 1.0020841800044864E-292) && (i < 20));

          qNorm = rt_hypotd_snf(Vf, xnrm2_hmd(18 - m, A_0, coffset + 2));
          if (Vf >= 0.0) {
            qNorm = -qNorm;
          }

          s_0[m] = (qNorm - Vf) / qNorm;
          Vf = 1.0 / (Vf - qNorm);
          for (i_k = lastv; i_k <= i_k_tmp; i_k++) {
            A_0[i_k - 1] *= Vf;
          }

          for (lastv = 0; lastv < i; lastv++) {
            qNorm *= 1.0020841800044864E-292;
          }

          Vf = qNorm;
        } else {
          s_0[m] = (qNorm - s_4) / qNorm;
          Vf = 1.0 / (s_4 - qNorm);
          i = (coffset - m) + 19;
          for (aoffset = lastv; aoffset <= i; aoffset++) {
            A_0[aoffset - 1] *= Vf;
          }

          Vf = qNorm;
        }
      }

      A_0[coffset] = Vf;
      if (m + 1 < 3) {
        A_0[coffset] = 1.0;
        if (s_0[m] != 0.0) {
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
            i_k = aoffset;
            do {
              exitg1 = 0;
              if (i_k + 1 <= aoffset + lastv) {
                if (A_0[i_k] != 0.0) {
                  exitg1 = 1;
                } else {
                  i_k++;
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
          xgemv_h(lastv, i, A_0, coffset + 20, A_0, coffset + 1, work_0);
          xgerc_f(lastv, i, -s_0[m], coffset + 1, work_0, A_0, coffset + 20);
        }

        A_0[coffset] = Vf;
      }
    }

    for (m = 0; m < 3; m++) {
      for (coffset = 0; coffset <= m; coffset++) {
        work[coffset + 3 * m] = A_0[19 * m + coffset];
      }

      for (coffset = m + 2; coffset < 4; coffset++) {
        work[(coffset + 3 * m) - 1] = 0.0;
      }
    }

    std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
    std::memcpy(&Ss_0[0], &rtDW.P_i[0], sizeof(real_T) << 8U);
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          coffset = i << 4;
          qNorm += rtDW.P_i[coffset + lastv] * rtDW.P_i[coffset + m];
        }

        C[lastv + (m << 4)] = qNorm;
      }
    }

    work_0[0] = rtU.fog_measurement[0] - rtU.fog_mask[0] * rtDW.x[10];
    work_0[1] = rtU.fog_measurement[1] - rtU.fog_mask[1] * rtDW.x[11];
    work_0[2] = rtU.fog_measurement[2] - rtU.fog_mask[2] * rtDW.x[12];
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 3; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          qNorm += C[(i << 4) + lastv] * dHdx_0[3 * i + m];
        }

        y[lastv + (m << 4)] = qNorm;
      }
    }

    for (lastv = 0; lastv < 3; lastv++) {
      s[3 * lastv] = work[lastv];
      s[3 * lastv + 1] = work[lastv + 3];
      s[3 * lastv + 2] = work[lastv + 6];
    }

    EKFCorrector_correctStateAndS_n(rtb_xNew_k, Ss_0, work_0, y, s, dHdx_0, s_1);
    std::memcpy(&rtDW.P_i[0], &Ss_0[0], sizeof(real_T) << 8U);

    // End of MATLAB Function: '<S3>/Correct'

    // DataStoreWrite: '<S3>/Data Store WriteX'
    std::memcpy(&rtDW.x[0], &rtb_xNew_k[0], sizeof(real_T) << 4U);
  }

  // End of Inport: '<Root>/enable_fog'
  // End of Outputs for SubSystem: '<S1>/Correct2'

  // Outputs for Enabled SubSystem: '<S1>/Correct3' incorporates:
  //   EnablePort: '<S4>/Enable'

  // Inport: '<Root>/enable_dvl'
  if (rtU.enable_dvl) {
    // MATLAB Function: '<S4>/Correct' incorporates:
    //   DataStoreRead: '<S4>/Data Store ReadX'
    //   DataStoreWrite: '<S4>/Data Store WriteP'
    //   Inport: '<Root>/R_dvl'
    //   Inport: '<Root>/dvl_context'
    //   Inport: '<Root>/dvl_measurement'

    rtDW.blockOrdering_p = rtDW.blockOrdering_n;
    p = true;
    for (m = 0; m < 9; m++) {
      if (p && (std::isinf(rtU.R_dvl[m]) || std::isnan(rtU.R_dvl[m]))) {
        p = false;
      }
    }

    if (p) {
      svd_a(rtU.R_dvl, work, s_0, s);
    } else {
      s_0[0] = (rtNaN);
      s_0[1] = (rtNaN);
      s_0[2] = (rtNaN);
      for (i = 0; i < 9; i++) {
        s[i] = (rtNaN);
      }
    }

    std::memset(&work[0], 0, 9U * sizeof(real_T));
    work[0] = s_0[0];
    work[4] = s_0[1];
    work[8] = s_0[2];
    for (m = 0; m < 9; m++) {
      work[m] = std::sqrt(work[m]);
    }

    for (lastv = 0; lastv < 3; lastv++) {
      qNorm = work[3 * lastv + 1];
      Vf = work[3 * lastv];
      s_4 = work[3 * lastv + 2];
      for (m = 0; m < 3; m++) {
        s_1[m + 3 * lastv] = (s[m + 3] * qNorm + Vf * s[m]) + s[m + 6] * s_4;
      }
    }

    std::memset(&dHdx_0[0], 0, 48U * sizeof(real_T));
    for (lastv = 0; lastv < 3; lastv++) {
      i = (lastv + 7) * 3;
      dHdx_0[i] = 0.0;
      dHdx_0[i + 1] = 0.0;
      dHdx_0[i + 2] = 0.0;
    }

    dHdx_0[21] = rtU.dvl_context[3];
    dHdx_0[25] = rtU.dvl_context[4];
    dHdx_0[29] = rtU.dvl_context[5];
    std::memset(&s[0], 0, 9U * sizeof(real_T));
    s[0] = rtU.dvl_context[3];
    s[4] = rtU.dvl_context[4];
    s[8] = rtU.dvl_context[5];
    work[0] = 0.0;
    work[3] = rtU.dvl_context[2];
    work[6] = -rtU.dvl_context[1];
    work[1] = -rtU.dvl_context[2];
    work[4] = 0.0;
    work[7] = rtU.dvl_context[0];
    work[2] = rtU.dvl_context[1];
    work[5] = -rtU.dvl_context[0];
    work[8] = 0.0;
    for (lastv = 0; lastv < 3; lastv++) {
      Vf = s[lastv + 3];
      s_4 = s[lastv];
      s_5 = s[lastv + 6];
      for (m = 0; m < 3; m++) {
        dHdx_0[lastv + 3 * (m + 10)] = (work[3 * m + 1] * Vf + work[3 * m] * s_4)
          + work[3 * m + 2] * s_5;
      }
    }

    for (m = 0; m < 3; m++) {
      coffset = m << 4;
      for (i = 0; i < 16; i++) {
        aoffset = i << 4;
        qNorm = 0.0;
        for (lastv = 0; lastv < 16; lastv++) {
          qNorm += dHdx_0[lastv * 3 + m] * rtDW.P_i[aoffset + lastv];
        }

        y[coffset + i] = qNorm;
      }
    }

    for (lastv = 0; lastv < 16; lastv++) {
      A_0[lastv] = y[lastv];
      A_0[lastv + 19] = y[lastv + 16];
      A_0[lastv + 38] = y[lastv + 32];
    }

    for (i = 0; i < 3; i++) {
      A_0[19 * i + 16] = s_1[i];
      A_0[19 * i + 17] = s_1[i + 3];
      A_0[19 * i + 18] = s_1[i + 6];
      work_0[i] = 0.0;
    }

    for (m = 0; m < 3; m++) {
      coffset = m * 19 + m;
      Vf = A_0[coffset];
      lastv = coffset + 2;
      s_0[m] = 0.0;
      qNorm = xnrm2_hmd(18 - m, A_0, coffset + 2);
      if (qNorm != 0.0) {
        s_4 = A_0[coffset];
        qNorm = rt_hypotd_snf(s_4, qNorm);
        if (s_4 >= 0.0) {
          qNorm = -qNorm;
        }

        if (std::abs(qNorm) < 1.0020841800044864E-292) {
          i = 0;
          i_k_tmp = (coffset - m) + 19;
          do {
            i++;
            for (aoffset = lastv; aoffset <= i_k_tmp; aoffset++) {
              A_0[aoffset - 1] *= 9.9792015476736E+291;
            }

            qNorm *= 9.9792015476736E+291;
            Vf *= 9.9792015476736E+291;
          } while ((std::abs(qNorm) < 1.0020841800044864E-292) && (i < 20));

          qNorm = rt_hypotd_snf(Vf, xnrm2_hmd(18 - m, A_0, coffset + 2));
          if (Vf >= 0.0) {
            qNorm = -qNorm;
          }

          s_0[m] = (qNorm - Vf) / qNorm;
          Vf = 1.0 / (Vf - qNorm);
          for (i_k = lastv; i_k <= i_k_tmp; i_k++) {
            A_0[i_k - 1] *= Vf;
          }

          for (lastv = 0; lastv < i; lastv++) {
            qNorm *= 1.0020841800044864E-292;
          }

          Vf = qNorm;
        } else {
          s_0[m] = (qNorm - s_4) / qNorm;
          Vf = 1.0 / (s_4 - qNorm);
          i = (coffset - m) + 19;
          for (aoffset = lastv; aoffset <= i; aoffset++) {
            A_0[aoffset - 1] *= Vf;
          }

          Vf = qNorm;
        }
      }

      A_0[coffset] = Vf;
      if (m + 1 < 3) {
        A_0[coffset] = 1.0;
        if (s_0[m] != 0.0) {
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
            i_k = aoffset;
            do {
              exitg1 = 0;
              if (i_k + 1 <= aoffset + lastv) {
                if (A_0[i_k] != 0.0) {
                  exitg1 = 1;
                } else {
                  i_k++;
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
          xgemv_h(lastv, i, A_0, coffset + 20, A_0, coffset + 1, work_0);
          xgerc_f(lastv, i, -s_0[m], coffset + 1, work_0, A_0, coffset + 20);
        }

        A_0[coffset] = Vf;
      }
    }

    for (m = 0; m < 3; m++) {
      for (coffset = 0; coffset <= m; coffset++) {
        work[coffset + 3 * m] = A_0[19 * m + coffset];
      }

      for (coffset = m + 2; coffset < 4; coffset++) {
        work[(coffset + 3 * m) - 1] = 0.0;
      }
    }

    std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
    std::memcpy(&Ss_0[0], &rtDW.P_i[0], sizeof(real_T) << 8U);
    work_0[0] = rtU.dvl_measurement[0] - ((rtU.dvl_context[2] * rtDW.x[11] -
      rtU.dvl_context[1] * rtDW.x[12]) + rtDW.x[7]) * rtU.dvl_context[3];
    work_0[1] = rtU.dvl_measurement[1] - ((rtU.dvl_context[0] * rtDW.x[12] -
      rtU.dvl_context[2] * rtDW.x[10]) + rtDW.x[8]) * rtU.dvl_context[4];
    work_0[2] = rtU.dvl_measurement[2] - ((rtU.dvl_context[1] * rtDW.x[10] -
      rtU.dvl_context[0] * rtDW.x[11]) + rtDW.x[9]) * rtU.dvl_context[5];
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          coffset = i << 4;
          qNorm += rtDW.P_i[coffset + lastv] * rtDW.P_i[coffset + m];
        }

        C[lastv + (m << 4)] = qNorm;
      }

      for (m = 0; m < 3; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          qNorm += C[(i << 4) + lastv] * dHdx_0[3 * i + m];
        }

        y[lastv + (m << 4)] = qNorm;
      }
    }

    for (lastv = 0; lastv < 3; lastv++) {
      s[3 * lastv] = work[lastv];
      s[3 * lastv + 1] = work[lastv + 3];
      s[3 * lastv + 2] = work[lastv + 6];
    }

    EKFCorrector_correctStateAndS_n(rtb_xNew_k, Ss_0, work_0, y, s, dHdx_0, s_1);
    std::memcpy(&rtDW.P_i[0], &Ss_0[0], sizeof(real_T) << 8U);

    // End of MATLAB Function: '<S4>/Correct'

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
      qNorm = rtU.R_depth;
      Vf = 1.0;
      if (rtU.R_depth != 0.0) {
        qNorm = std::abs(rtU.R_depth);
      }

      if (qNorm < 0.0) {
        qNorm = -qNorm;
        Vf = -1.0;
      }
    } else {
      qNorm = (rtNaN);
      Vf = (rtNaN);
    }

    // DataStoreWrite: '<S5>/Data Store WriteX' incorporates:
    //   DataStoreWrite: '<S5>/Data Store WriteP'
    //   Inport: '<Root>/depth_mask'
    //   Inport: '<Root>/depth_measurement'
    //   MATLAB Function: '<S5>/Correct'

    EKFCorrector_correct(rtU.depth_measurement, Vf * std::sqrt(qNorm), rtDW.x,
                         rtDW.P_i, rtU.depth_mask);
  }

  // End of Inport: '<Root>/enable_depth'
  // End of Outputs for SubSystem: '<S1>/Correct4'

  // Outputs for Enabled SubSystem: '<S1>/Correct5' incorporates:
  //   EnablePort: '<S6>/Enable'

  // Inport: '<Root>/enable_reset'
  if (rtU.enable_reset) {
    // MATLAB Function: '<S6>/Correct' incorporates:
    //   DataStoreRead: '<S6>/Data Store ReadX'
    //   DataStoreWrite: '<S6>/Data Store WriteP'
    //   Inport: '<Root>/R_reset'
    //   Inport: '<Root>/reset_state'

    p = true;
    for (m = 0; m < 256; m++) {
      if (p && (std::isinf(rtU.R_reset[m]) || std::isnan(rtU.R_reset[m]))) {
        p = false;
      }
    }

    if (p) {
      svd_d(rtU.R_reset, Ss_0, rtb_xNew_k, K);
    } else {
      for (i = 0; i < 16; i++) {
        rtb_xNew_k[i] = (rtNaN);
      }

      for (lastv = 0; lastv < 256; lastv++) {
        K[lastv] = (rtNaN);
      }
    }

    std::memset(&Ss_0[0], 0, sizeof(real_T) << 8U);
    for (m = 0; m < 16; m++) {
      Ss_0[m + (m << 4)] = rtb_xNew_k[m];
    }

    for (m = 0; m < 256; m++) {
      Ss_0[m] = std::sqrt(Ss_0[m]);
    }

    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          qNorm += K[(i << 4) + m] * Ss_0[(lastv << 4) + i];
        }

        Rsqrt_0[m + (lastv << 4)] = qNorm;
      }
    }

    std::memcpy(&K[0], &rtDW.P_i[0], sizeof(real_T) << 8U);
    qrFactor(b, K, Rsqrt_0);
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          coffset = i << 4;
          qNorm += rtDW.P_i[coffset + lastv] * rtDW.P_i[coffset + m];
        }

        C[lastv + (m << 4)] = qNorm;
      }
    }

    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          qNorm += C[(i << 4) + m] * b[(lastv << 4) + i];
        }

        Ss_0[lastv + (m << 4)] = qNorm;
      }
    }

    std::memcpy(&C[0], &Ss_0[0], sizeof(real_T) << 8U);
    trisolve_j(K, C);
    for (m = 0; m < 16; m++) {
      std::memcpy(&Ss_0[m << 4], &C[m << 4], sizeof(real_T) << 4U);
      for (coffset = 0; coffset < 16; coffset++) {
        K_0[(m << 4) + coffset] = K[(coffset << 4) + m];
      }
    }

    trisolve_jl(K_0, Ss_0);
    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        K[m + (lastv << 4)] = Ss_0[(m << 4) + lastv];
      }
    }

    for (lastv = 0; lastv < 256; lastv++) {
      K_0[lastv] = -K[lastv];
    }

    for (lastv = 0; lastv < 16; lastv++) {
      for (m = 0; m < 16; m++) {
        qNorm = 0.0;
        for (i = 0; i < 16; i++) {
          qNorm += K_0[(i << 4) + m] * b[(lastv << 4) + i];
        }

        Ss_0[m + (lastv << 4)] = qNorm;
      }
    }

    for (i = 0; i < 16; i++) {
      m = (i << 4) + i;
      Ss_0[m]++;
      for (lastv = 0; lastv < 16; lastv++) {
        Vf = 0.0;
        for (m = 0; m < 16; m++) {
          Vf += K[(m << 4) + i] * Rsqrt_0[(lastv << 4) + m];
        }

        K_0[i + (lastv << 4)] = Vf;
      }
    }

    qrFactor(Ss_0, rtDW.P_i, K_0);
    for (lastv = 0; lastv < 16; lastv++) {
      tmp_0[lastv] = rtU.reset_state[lastv] - rtDW.x[lastv];
    }

    // DataStoreWrite: '<S6>/Data Store WriteX' incorporates:
    //   DataStoreRead: '<S6>/Data Store ReadX'
    //   MATLAB Function: '<S6>/Correct'

    for (lastv = 0; lastv < 16; lastv++) {
      qNorm = 0.0;
      for (m = 0; m < 16; m++) {
        qNorm += K[(m << 4) + lastv] * tmp_0[m];
      }

      rtDW.x[lastv] += qNorm;
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
      qNorm = 0.0;

      // End of Outputs for SubSystem: '<S1>/Output'
      for (m = 0; m < 16; m++) {
        // Outputs for Atomic SubSystem: '<S1>/Output'
        // MATLAB Function: '<S7>/MATLAB Function'
        coffset = m << 4;
        qNorm += rtDW.P_i[coffset + i] * rtDW.P_i[coffset + lastv];

        // End of Outputs for SubSystem: '<S1>/Output'
      }

      // Outputs for Atomic SubSystem: '<S1>/Output'
      // MATLAB Function: '<S7>/MATLAB Function' incorporates:
      //   DataStoreRead: '<S7>/Data Store Read1'

      rtY.covariance[i + (lastv << 4)] = qNorm;

      // End of Outputs for SubSystem: '<S1>/Output'
    }

    // End of Outport: '<Root>/covariance'
  }

  // Outputs for Atomic SubSystem: '<S1>/Predict'
  // MATLAB Function: '<S8>/Predict' incorporates:
  //   DataStoreRead: '<S8>/Data Store ReadX'
  //   DataStoreWrite: '<S8>/Data Store WriteP'
  //   DataStoreWrite: '<S8>/Data Store WriteX'
  //   Inport: '<Root>/Q'
  //   Inport: '<Root>/dt'

  p = true;
  for (lastv = 0; lastv < 256; lastv++) {
    if (p && (std::isinf(rtU.Q[lastv]) || std::isnan(rtU.Q[lastv]))) {
      p = false;
    }
  }

  if (p) {
    svd_n(rtU.Q, Ss_0, rtb_xNew_k, K);
  } else {
    for (i = 0; i < 16; i++) {
      rtb_xNew_k[i] = (rtNaN);
    }

    for (lastv = 0; lastv < 256; lastv++) {
      K[lastv] = (rtNaN);
    }
  }

  std::memset(&Ss_0[0], 0, sizeof(real_T) << 8U);
  for (m = 0; m < 16; m++) {
    Ss_0[m + (m << 4)] = rtb_xNew_k[m];
  }

  for (m = 0; m < 256; m++) {
    Ss_0[m] = std::sqrt(Ss_0[m]);
  }

  for (m = 0; m < 16; m++) {
    qNorm = 1.0E-6 * std::fmax(1.0, std::abs(rtDW.x[m]));
    std::memcpy(&rtb_xNew_k[0], &rtDW.x[0], sizeof(real_T) << 4U);
    std::memcpy(&xm[0], &rtDW.x[0], sizeof(real_T) << 4U);
    Vf = rtDW.x[m];
    rtb_xNew_k[m] = Vf + qNorm;
    xm[m] = Vf - qNorm;
    talos_state_transition(rtb_xNew_k, rtU.dt, tmp_0);
    talos_state_transition(xm, rtU.dt, rtb_xNew_k);
    qNorm *= 2.0;
    for (lastv = 0; lastv < 16; lastv++) {
      Rsqrt_0[lastv + (m << 4)] = (tmp_0[lastv] - rtb_xNew_k[lastv]) / qNorm;
    }
  }

  for (m = 0; m < 16; m++) {
    coffset = m << 4;
    for (i = 0; i < 16; i++) {
      i_k = i << 4;
      qNorm = 0.0;
      Vf = 0.0;
      for (lastv = 0; lastv < 16; lastv++) {
        aoffset = lastv << 4;
        qNorm += Rsqrt_0[aoffset + m] * rtDW.P_i[i_k + lastv];
        Vf += K[aoffset + i] * Ss_0[coffset + lastv];
      }

      K_0[m + i_k] = Vf;
      C[coffset + i] = qNorm;
    }
  }

  for (i = 0; i < 16; i++) {
    for (lastv = 0; lastv < 16; lastv++) {
      m = (i << 4) + lastv;
      coffset = (i << 5) + lastv;
      A_1[coffset] = C[m];
      A_1[coffset + 16] = K_0[m];
    }

    xm[i] = 0.0;
  }

  for (m = 0; m < 16; m++) {
    coffset = (m << 5) + m;
    Vf = A_1[coffset];
    lastv = coffset + 2;
    rtb_xNew_k[m] = 0.0;
    qNorm = xnrm2_dzn(31 - m, A_1, coffset + 2);
    if (qNorm != 0.0) {
      s_4 = A_1[coffset];
      qNorm = rt_hypotd_snf(s_4, qNorm);
      if (s_4 >= 0.0) {
        qNorm = -qNorm;
      }

      if (std::abs(qNorm) < 1.0020841800044864E-292) {
        i = 0;
        i_k_tmp = (coffset - m) + 32;
        do {
          i++;
          for (aoffset = lastv; aoffset <= i_k_tmp; aoffset++) {
            A_1[aoffset - 1] *= 9.9792015476736E+291;
          }

          qNorm *= 9.9792015476736E+291;
          Vf *= 9.9792015476736E+291;
        } while ((std::abs(qNorm) < 1.0020841800044864E-292) && (i < 20));

        qNorm = rt_hypotd_snf(Vf, xnrm2_dzn(31 - m, A_1, coffset + 2));
        if (Vf >= 0.0) {
          qNorm = -qNorm;
        }

        rtb_xNew_k[m] = (qNorm - Vf) / qNorm;
        Vf = 1.0 / (Vf - qNorm);
        for (i_k = lastv; i_k <= i_k_tmp; i_k++) {
          A_1[i_k - 1] *= Vf;
        }

        for (aoffset = 0; aoffset < i; aoffset++) {
          qNorm *= 1.0020841800044864E-292;
        }

        Vf = qNorm;
      } else {
        rtb_xNew_k[m] = (qNorm - s_4) / qNorm;
        Vf = 1.0 / (s_4 - qNorm);
        i = (coffset - m) + 32;
        for (i_k = lastv; i_k <= i; i_k++) {
          A_1[i_k - 1] *= Vf;
        }

        Vf = qNorm;
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
          i_k = aoffset;
          do {
            exitg1 = 0;
            if (i_k + 1 <= aoffset + lastv) {
              if (A_1[i_k] != 0.0) {
                exitg1 = 1;
              } else {
                i_k++;
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
      Ss_0[coffset + (m << 4)] = A_1[(m << 5) + coffset];
    }

    for (coffset = m + 2; coffset < 17; coffset++) {
      Ss_0[(coffset + (m << 4)) - 1] = 0.0;
    }
  }

  for (lastv = 0; lastv < 16; lastv++) {
    for (m = 0; m < 16; m++) {
      rtDW.P_i[m + (lastv << 4)] = Ss_0[(m << 4) + lastv];
    }
  }

  std::memcpy(&tmp_0[0], &rtDW.x[0], sizeof(real_T) << 4U);
  talos_state_transition(tmp_0, rtU.dt, rtDW.x);

  // End of MATLAB Function: '<S8>/Predict'
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
