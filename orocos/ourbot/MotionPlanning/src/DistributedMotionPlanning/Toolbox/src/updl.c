/* This function was automatically generated by CasADi */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else /* CODEGEN_PREFIX */
  #define CASADI_PREFIX(ID) updl_ ## ID
#endif /* CODEGEN_PREFIX */

#include <math.h>

#ifndef real_t
#define real_t double
#endif /* real_t */

#define to_double(x) (double) x
#define to_int(x) (int) x
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
real_t CASADI_PREFIX(fmin)(real_t x, real_t y) { return x<y ? x : y;}
#define fmin(x,y) CASADI_PREFIX(fmin)(x,y)
real_t CASADI_PREFIX(fmax)(real_t x, real_t y) { return x>y ? x : y;}
#define fmax(x,y) CASADI_PREFIX(fmax)(x,y)
#endif

#define PRINTF printf
real_t CASADI_PREFIX(sq)(real_t x) { return x*x;}
#define sq(x) CASADI_PREFIX(sq)(x)

real_t CASADI_PREFIX(sign)(real_t x) { return x<0 ? -1 : x>0 ? 1 : x;}
#define sign(x) CASADI_PREFIX(sign)(x)

static const int CASADI_PREFIX(s0)[30] = {26, 1, 0, 26, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};
#define s0 CASADI_PREFIX(s0)
static const int CASADI_PREFIX(s1)[56] = {52, 1, 0, 52, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};
#define s1 CASADI_PREFIX(s1)
static const int CASADI_PREFIX(s2)[5] = {1, 1, 0, 1, 0};
#define s2 CASADI_PREFIX(s2)
/* upd_l_0 */
int upd_l_0(const real_t** arg, real_t** res, int* iw, real_t* w, int mem) {
  real_t a0=arg[0] ? arg[0][0] : 0;
  real_t a1=arg[1] ? arg[1][0] : 0;
  a0=(a0-a1);
  a1=arg[6] ? arg[6][0] : 0;
  a0=(a1*a0);
  real_t a2=arg[3] ? arg[3][0] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][0]=a2;
  a2=arg[0] ? arg[0][1] : 0;
  a0=arg[1] ? arg[1][1] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][1] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0] ? arg[0][2] : 0;
  a2=arg[1] ? arg[1][2] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][2] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][2]=a2;
  a2=arg[0] ? arg[0][3] : 0;
  a0=arg[1] ? arg[1][3] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][3] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[0] ? arg[0][4] : 0;
  a2=arg[1] ? arg[1][4] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][4] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][4]=a2;
  a2=arg[0] ? arg[0][5] : 0;
  a0=arg[1] ? arg[1][5] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][5] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[0] ? arg[0][6] : 0;
  a2=arg[1] ? arg[1][6] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][6] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][6]=a2;
  a2=arg[0] ? arg[0][7] : 0;
  a0=arg[1] ? arg[1][7] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][7] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][7]=a0;
  a0=arg[0] ? arg[0][8] : 0;
  a2=arg[1] ? arg[1][8] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][8] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][8]=a2;
  a2=arg[0] ? arg[0][9] : 0;
  a0=arg[1] ? arg[1][9] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][9] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][9]=a0;
  a0=arg[0] ? arg[0][10] : 0;
  a2=arg[1] ? arg[1][10] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][10] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][10]=a2;
  a2=arg[0] ? arg[0][11] : 0;
  a0=arg[1] ? arg[1][11] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][11] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][11]=a0;
  a0=arg[0] ? arg[0][12] : 0;
  a2=arg[1] ? arg[1][12] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][12] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][12]=a2;
  a2=arg[0] ? arg[0][13] : 0;
  a0=arg[1] ? arg[1][13] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][13] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][13]=a0;
  a0=arg[0] ? arg[0][14] : 0;
  a2=arg[1] ? arg[1][14] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][14] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][14]=a2;
  a2=arg[0] ? arg[0][15] : 0;
  a0=arg[1] ? arg[1][15] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][15] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][15]=a0;
  a0=arg[0] ? arg[0][16] : 0;
  a2=arg[1] ? arg[1][16] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][16] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][16]=a2;
  a2=arg[0] ? arg[0][17] : 0;
  a0=arg[1] ? arg[1][17] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][17] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][17]=a0;
  a0=arg[0] ? arg[0][18] : 0;
  a2=arg[1] ? arg[1][18] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][18] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][18]=a2;
  a2=arg[0] ? arg[0][19] : 0;
  a0=arg[1] ? arg[1][19] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][19] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][19]=a0;
  a0=arg[0] ? arg[0][20] : 0;
  a2=arg[1] ? arg[1][20] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][20] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][20]=a2;
  a2=arg[0] ? arg[0][21] : 0;
  a0=arg[1] ? arg[1][21] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][21] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][21]=a0;
  a0=arg[0] ? arg[0][22] : 0;
  a2=arg[1] ? arg[1][22] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][22] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][22]=a2;
  a2=arg[0] ? arg[0][23] : 0;
  a0=arg[1] ? arg[1][23] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][23] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][23]=a0;
  a0=arg[0] ? arg[0][24] : 0;
  a2=arg[1] ? arg[1][24] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[3] ? arg[3][24] : 0;
  a2=(a2+a0);
  if (res[0]!=0) res[0][24]=a2;
  a2=arg[0] ? arg[0][25] : 0;
  a0=arg[1] ? arg[1][25] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[3] ? arg[3][25] : 0;
  a0=(a0+a2);
  if (res[0]!=0) res[0][25]=a0;
  a0=arg[5] ? arg[5][0] : 0;
  a2=arg[2] ? arg[2][0] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][0] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][0]=a2;
  a2=arg[5] ? arg[5][1] : 0;
  a0=arg[2] ? arg[2][1] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][1] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][1]=a0;
  a0=arg[5] ? arg[5][2] : 0;
  a2=arg[2] ? arg[2][2] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][2] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][2]=a2;
  a2=arg[5] ? arg[5][3] : 0;
  a0=arg[2] ? arg[2][3] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][3] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][3]=a0;
  a0=arg[5] ? arg[5][4] : 0;
  a2=arg[2] ? arg[2][4] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][4] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][4]=a2;
  a2=arg[5] ? arg[5][5] : 0;
  a0=arg[2] ? arg[2][5] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][5] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][5]=a0;
  a0=arg[5] ? arg[5][6] : 0;
  a2=arg[2] ? arg[2][6] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][6] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][6]=a2;
  a2=arg[5] ? arg[5][7] : 0;
  a0=arg[2] ? arg[2][7] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][7] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][7]=a0;
  a0=arg[5] ? arg[5][8] : 0;
  a2=arg[2] ? arg[2][8] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][8] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][8]=a2;
  a2=arg[5] ? arg[5][9] : 0;
  a0=arg[2] ? arg[2][9] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][9] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][9]=a0;
  a0=arg[5] ? arg[5][10] : 0;
  a2=arg[2] ? arg[2][10] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][10] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][10]=a2;
  a2=arg[5] ? arg[5][11] : 0;
  a0=arg[2] ? arg[2][11] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][11] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][11]=a0;
  a0=arg[5] ? arg[5][12] : 0;
  a2=arg[2] ? arg[2][12] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][12] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][12]=a2;
  a2=arg[5] ? arg[5][13] : 0;
  a0=arg[2] ? arg[2][13] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][13] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][13]=a0;
  a0=arg[5] ? arg[5][14] : 0;
  a2=arg[2] ? arg[2][14] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][14] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][14]=a2;
  a2=arg[5] ? arg[5][15] : 0;
  a0=arg[2] ? arg[2][15] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][15] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][15]=a0;
  a0=arg[5] ? arg[5][16] : 0;
  a2=arg[2] ? arg[2][16] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][16] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][16]=a2;
  a2=arg[5] ? arg[5][17] : 0;
  a0=arg[2] ? arg[2][17] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][17] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][17]=a0;
  a0=arg[5] ? arg[5][18] : 0;
  a2=arg[2] ? arg[2][18] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][18] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][18]=a2;
  a2=arg[5] ? arg[5][19] : 0;
  a0=arg[2] ? arg[2][19] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][19] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][19]=a0;
  a0=arg[5] ? arg[5][20] : 0;
  a2=arg[2] ? arg[2][20] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][20] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][20]=a2;
  a2=arg[5] ? arg[5][21] : 0;
  a0=arg[2] ? arg[2][21] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][21] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][21]=a0;
  a0=arg[5] ? arg[5][22] : 0;
  a2=arg[2] ? arg[2][22] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][22] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][22]=a2;
  a2=arg[5] ? arg[5][23] : 0;
  a0=arg[2] ? arg[2][23] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][23] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][23]=a0;
  a0=arg[5] ? arg[5][24] : 0;
  a2=arg[2] ? arg[2][24] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][24] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][24]=a2;
  a2=arg[5] ? arg[5][25] : 0;
  a0=arg[2] ? arg[2][25] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][25] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][25]=a0;
  a0=arg[5] ? arg[5][26] : 0;
  a2=arg[2] ? arg[2][26] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][26] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][26]=a2;
  a2=arg[5] ? arg[5][27] : 0;
  a0=arg[2] ? arg[2][27] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][27] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][27]=a0;
  a0=arg[5] ? arg[5][28] : 0;
  a2=arg[2] ? arg[2][28] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][28] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][28]=a2;
  a2=arg[5] ? arg[5][29] : 0;
  a0=arg[2] ? arg[2][29] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][29] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][29]=a0;
  a0=arg[5] ? arg[5][30] : 0;
  a2=arg[2] ? arg[2][30] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][30] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][30]=a2;
  a2=arg[5] ? arg[5][31] : 0;
  a0=arg[2] ? arg[2][31] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][31] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][31]=a0;
  a0=arg[5] ? arg[5][32] : 0;
  a2=arg[2] ? arg[2][32] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][32] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][32]=a2;
  a2=arg[5] ? arg[5][33] : 0;
  a0=arg[2] ? arg[2][33] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][33] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][33]=a0;
  a0=arg[5] ? arg[5][34] : 0;
  a2=arg[2] ? arg[2][34] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][34] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][34]=a2;
  a2=arg[5] ? arg[5][35] : 0;
  a0=arg[2] ? arg[2][35] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][35] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][35]=a0;
  a0=arg[5] ? arg[5][36] : 0;
  a2=arg[2] ? arg[2][36] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][36] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][36]=a2;
  a2=arg[5] ? arg[5][37] : 0;
  a0=arg[2] ? arg[2][37] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][37] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][37]=a0;
  a0=arg[5] ? arg[5][38] : 0;
  a2=arg[2] ? arg[2][38] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][38] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][38]=a2;
  a2=arg[5] ? arg[5][39] : 0;
  a0=arg[2] ? arg[2][39] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][39] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][39]=a0;
  a0=arg[5] ? arg[5][40] : 0;
  a2=arg[2] ? arg[2][40] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][40] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][40]=a2;
  a2=arg[5] ? arg[5][41] : 0;
  a0=arg[2] ? arg[2][41] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][41] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][41]=a0;
  a0=arg[5] ? arg[5][42] : 0;
  a2=arg[2] ? arg[2][42] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][42] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][42]=a2;
  a2=arg[5] ? arg[5][43] : 0;
  a0=arg[2] ? arg[2][43] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][43] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][43]=a0;
  a0=arg[5] ? arg[5][44] : 0;
  a2=arg[2] ? arg[2][44] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][44] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][44]=a2;
  a2=arg[5] ? arg[5][45] : 0;
  a0=arg[2] ? arg[2][45] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][45] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][45]=a0;
  a0=arg[5] ? arg[5][46] : 0;
  a2=arg[2] ? arg[2][46] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][46] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][46]=a2;
  a2=arg[5] ? arg[5][47] : 0;
  a0=arg[2] ? arg[2][47] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][47] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][47]=a0;
  a0=arg[5] ? arg[5][48] : 0;
  a2=arg[2] ? arg[2][48] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][48] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][48]=a2;
  a2=arg[5] ? arg[5][49] : 0;
  a0=arg[2] ? arg[2][49] : 0;
  a2=(a2-a0);
  a2=(a1*a2);
  a0=arg[4] ? arg[4][49] : 0;
  a0=(a0+a2);
  if (res[1]!=0) res[1][49]=a0;
  a0=arg[5] ? arg[5][50] : 0;
  a2=arg[2] ? arg[2][50] : 0;
  a0=(a0-a2);
  a0=(a1*a0);
  a2=arg[4] ? arg[4][50] : 0;
  a2=(a2+a0);
  if (res[1]!=0) res[1][50]=a2;
  a2=arg[5] ? arg[5][51] : 0;
  a0=arg[2] ? arg[2][51] : 0;
  a2=(a2-a0);
  a1=(a1*a2);
  a2=arg[4] ? arg[4][51] : 0;
  a2=(a2+a1);
  if (res[1]!=0) res[1][51]=a2;
  return 0;
}

void upd_l_0_incref(void) {
}

void upd_l_0_decref(void) {
}

int upd_l_0_n_in(void) { return 7;}

int upd_l_0_n_out(void) { return 2;}

const char* upd_l_0_name_in(int i){
  switch (i) {
  case 0: return "i0";
  case 1: return "i1";
  case 2: return "i2";
  case 3: return "i3";
  case 4: return "i4";
  case 5: return "i5";
  case 6: return "i6";
  default: return 0;
  }
}

const char* upd_l_0_name_out(int i){
  switch (i) {
  case 0: return "o0";
  case 1: return "o1";
  default: return 0;
  }
}

const int* upd_l_0_sparsity_in(int i) {
  switch (i) {
  case 0: return s0;
  case 1: return s0;
  case 2: return s1;
  case 3: return s0;
  case 4: return s1;
  case 5: return s1;
  case 6: return s2;
  default: return 0;
  }
}

const int* upd_l_0_sparsity_out(int i) {
  switch (i) {
  case 0: return s0;
  case 1: return s1;
  default: return 0;
  }
}

int upd_l_0_work(int *sz_arg, int* sz_res, int *sz_iw, int *sz_w) {
  if (sz_arg) *sz_arg = 7;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 3;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
