/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#ifndef HPP_FCL_DATA_TYPES_H
#define HPP_FCL_DATA_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <hpp/fcl/config.hh>

namespace hpp
{

#ifdef HPP_FCL_HAVE_OCTOMAP
  #define OCTOMAP_VERSION_AT_LEAST(x,y,z) \
    (OCTOMAP_MAJOR_VERSION > x || (OCTOMAP_MAJOR_VERSION >= x && \
    (OCTOMAP_MINOR_VERSION > y || (OCTOMAP_MINOR_VERSION >= y && \
    OCTOMAP_PATCH_VERSION >= z))))

  #define OCTOMAP_VERSION_AT_MOST(x,y,z) \
    (OCTOMAP_MAJOR_VERSION < x || (OCTOMAP_MAJOR_VERSION <= x && \
    (OCTOMAP_MINOR_VERSION < y || (OCTOMAP_MINOR_VERSION <= y && \
    OCTOMAP_PATCH_VERSION <= z))))
#endif // HPP_FCL_HAVE_OCTOMAP
}

namespace hpp
{
namespace fcl
{
typedef double FCL_REAL;
typedef Eigen::Matrix<FCL_REAL, 3, 1> Vec3f;
typedef Eigen::Matrix<FCL_REAL, 3, 3> Matrix3f;
typedef Eigen::Vector2i support_func_guess_t;

/// @brief Triangle with 3 indices for points
class HPP_FCL_DLLAPI Triangle
{
public:
  typedef std::size_t index_type;
  typedef int size_type;

  /// @brief Default constructor
  Triangle() {}

  /// @brief Create a triangle with given vertex indices
  Triangle(index_type p1, index_type p2, index_type p3)
  {
    set(p1, p2, p3);
  }

  /// @brief Set the vertex indices of the triangle
  inline void set(index_type p1, index_type p2, index_type p3)
  {
    vids[0] = p1; vids[1] = p2; vids[2] = p3;
  }

  /// @brief Access the triangle index
  inline index_type operator[](int i) const { return vids[i]; }

  inline index_type& operator[](int i) { return vids[i]; }

  static inline size_type size() { return 3; }

  bool operator== (const Triangle& other) const
  {
    return vids[0] == other.vids[0]
      &&   vids[1] == other.vids[1]
      &&   vids[2] == other.vids[2];
  }

private:
  /// @brief indices for each vertex of triangle
  index_type vids[3];
};

}

} // namespace hpp


#endif
