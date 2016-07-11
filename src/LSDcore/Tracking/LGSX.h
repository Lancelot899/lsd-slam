/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "util/EigenCoreInclude.h"
#include <opencv2/core/core.hpp>
#include "util/settings.h"

#include <eigen3/Eigen/Dense>

namespace lsd_slam
{


typedef Eigen::Matrix<float, 6, 1> Vector6;
typedef Eigen::Matrix<float, 6, 6> Matrix6x6;

typedef Eigen::Matrix<float, 7, 1> Vector7;
typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

typedef Eigen::Matrix<float, 4, 1> Vector4;
typedef Eigen::Matrix<float, 4, 4> Matrix4x4;




class LGS4
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Matrix4x4 A;
  Vector4 b;

  float error;
  size_t num_constraints;

  inline void initialize(const size_t)
  {
    A.setZero();
    b.setZero();
    memset(SSEData,0, sizeof(float)*4*15);
    error = 0;
    this->num_constraints = 0;
  }

  inline void update(const Vector4& J, const float& res, const float& weight)
  {
    A.noalias() += J * J.transpose() * weight;
    b.noalias() -= J * (res * weight);
    error += res * res * weight;
    num_constraints += 1;
  }

private:
  EIGEN_ALIGN16 float SSEData[4*15];
};



/**
 * Builds 4dof LGS (used for depth-lgs, at it has only 7 non-zero entries in jacobian)
 * only used to accumulate data, NOT really as LGS
 */
class LGS6
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Matrix6x6 A;
  Vector6 b;

  float error;
  size_t num_constraints;

  inline void initialize(const size_t)
  {
    A.setZero();
    b.setZero();
    memset(SSEData,0, sizeof(float)*4*28);

    error = 0;
    this->num_constraints = 0;
  }

  void finish()
  {
    A /= (float) num_constraints;
    b /= (float) num_constraints;
    error /= (float) num_constraints;
  }

  inline void update(const Vector6& J, const float& res, const float& weight)
  {
    A.noalias() += J * J.transpose() * weight;
    b.noalias() -= J * (res * weight);
    error += res * res * weight;
    num_constraints += 1;
  }



private:
  EIGEN_ALIGN16 float SSEData[4*28];
};


class LGS7
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Matrix7x7 A;
  Vector7 b;

  float error;
  size_t num_constraints;

  void initializeFrom(const LGS6& ls6, const LGS4& ls4)
  {
  	// set zero
  	A.setZero();
  	b.setZero();

  	// add ls6
  	A.topLeftCorner<6,6>() = ls6.A;
  	b.head<6>() = ls6.b;

  	// add ls4
  	int remap[4] = {2,3,4,6};
  	for(int i=0;i<4;i++)
  	{
  		b[remap[i]] += ls4.b[i];
  		for(int j=0;j<4;j++)
  			A(remap[i], remap[j]) += ls4.A(i,j);
  	}

  	num_constraints = ls6.num_constraints + ls4.num_constraints;
  }
};

}
