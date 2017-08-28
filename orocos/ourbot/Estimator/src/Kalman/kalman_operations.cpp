/*
 *    This file is part of async_kalman.
 *
 *    async_kalman -- an synchronous kalman filter implementation
 *    Copyright (C) 2016 Joris Gillis,
 *                            K.U. Leuven. All rights reserved.
 *
 *    async_kalman is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    async_kalman is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with async_kalman; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include "kalman_operations.hpp"
#include <exception>
#include <iostream>

extern "C" {
  int mb05nd_(f_int* n, double* delta, const double*A, f_int* lda,
                      double* ex, f_int* ldex, double* exint, f_int* ldexin,
                      double* tol, f_int *iwork, double* dwork, f_int *ld_work, f_int *info);
}

void slicot_mb05nd(int n, double delta, const double* A, int lda,
                   double* ex, int ldex, double* exint, int ldexin,
                   double tol, f_int* iwork, double* dwork, int ld_work, int& info) {
   f_int n_ = n;
   f_int lda_ = lda;
   f_int ldex_ = ldex;
   f_int ldexin_ = ldexin;
   f_int info_ = 0;
   f_int ld_work_ = ld_work;

   mb05nd_(&n_, &delta, A, &lda_, ex, &ldex_, exint, &ldexin_, &tol, iwork, dwork, &ld_work_,
     &info_);

   info = info_;

   if (info_!=0) {
     std::cerr << "mb05nd error:" << info_ << std::endl;
   }

}
