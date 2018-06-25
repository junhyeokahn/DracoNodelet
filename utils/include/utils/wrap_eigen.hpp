/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file sejong/wrap_eigen.hpp
   \author Roland Philippsen
*/

#ifndef SEJONG_WRAP_EIGEN_HPP
#define SEJONG_WRAP_EIGEN_HPP


#define SJ_SAFE_DELETE(p)			if(p) { delete (p); (p) = NULL; }
#define SJ_SAFE_DESTROY_WINDOW(p) if(p) { p->DestroyWindow(); delete (p); (p) = NULL; }
#define SJ_SAFE_DELETE_AR(p)		if(p) { delete [] p; (p) = NULL; }
#define SJ_SAFE_RELEASE(p)		if(p) { (p)->Release(); (p) = NULL; }

#define SJ_ISZERO(x)	(fabs(x) < 1.e-6)			// zero test for floating point numbers
#define SJ_ISEQUAL(x,y)	(fabs((x) - (y)) < 1.e-6) // test for equality of float numbers
#define SJ_ROUND(x)		(floor((x) + 0.5))			// floating point number rounding
#define SJ_RAND(l,u)	((double)rand() / RAND_MAX * ((u) - (l)) + (l))	// float random number from interval < l ; u >
#define SJ_RAND_INT(l,u)  (rand() % (u - l) + l)  // int random number in interval [l,u) including l, excluding u


#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <list>
#include <string>
#include <cmath>
#include <ctime>

namespace sejong {
  typedef Eigen::Transform<double, 3, Eigen::Affine> Transform;
  typedef Eigen::Translation3d Translation;
  typedef Eigen::Quaternion<double> Quaternion;
  typedef Eigen::Matrix<double,2,1> Vect2;
  typedef Eigen::Matrix<double,3,1> Vect3;
  typedef Eigen::Matrix<double,4,1> Vect4;
  typedef Eigen::VectorXd Vector;
  typedef Eigen::MatrixXd Matrix;
  // Euler ange (Yaw, Pitch, Roll) to Quaternion
  void convert(double yaw, double pitch, double roll, sejong::Quaternion& to);
  // Quaternion to Euler ZYX
  void convert(const sejong::Quaternion& from, double & yaw, double & pitch, double & roll);
  // Quaternion to so(3)
  void convert(sejong::Quaternion const & from, sejong::Vector & to);

  // so(3) to Quaternion
  void convert(sejong::Vector const & from, sejong::Quaternion & to);
  // sejong::Vector to std::vector
  void convert(sejong::Vector const & from, std::vector<double> & to);
  // std::vector to sejong::Vector
  void convert(std::vector<double> const & from, sejong::Vector & to);
  // double array to sejong::Vector
  void convert(double const * from, size_t length, sejong::Vector & to);

  Quaternion QuatMultiply(const Quaternion & q1, const Quaternion & q2);

  bool compare(sejong::Matrix const & lhs, sejong::Matrix const & rhs, double precision);
  bool compare(sejong::Quaternion const & lhs, sejong::Quaternion const & rhs, double precision);

  double _bind_half_pi(double);

  void Copy(const sejong::Vector & sub, double* obj);
  void Copy(const double* sub, double* obj, int dim);
  void SetArrayZero(double* array, int dim);
  double Dot(const double* a, const double * b, int dim);
  // Signum returns (-1, 0, or 1) depending on the sign of the argument
  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0)) ;
  }
}

#endif
