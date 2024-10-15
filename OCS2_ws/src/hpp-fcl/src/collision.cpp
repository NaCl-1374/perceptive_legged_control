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


#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_func_matrix.h>
#include <hpp/fcl/narrowphase/narrowphase.h>

#include <iostream>

namespace hpp
{
namespace fcl
{

CollisionFunctionMatrix& getCollisionFunctionLookTable()
{
  static CollisionFunctionMatrix table;
  return table;
}

// reorder collision results in the order the call has been made.
void CollisionResult::swapObjects()
{
  for(std::vector<Contact>::iterator it = contacts.begin();
      it != contacts.end(); ++it)
  {
    std::swap(it->o1, it->o2);
    std::swap(it->b1, it->b2);
  }
}

std::size_t collide(const CollisionObject* o1, const CollisionObject* o2,
                    const CollisionRequest& request, CollisionResult& result)
{
  return collide(
      o1->collisionGeometry().get(), o1->getTransform(),
      o2->collisionGeometry().get(), o2->getTransform(),
      request, result);
}

std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1,
                    const CollisionGeometry* o2, const Transform3f& tf2,
                    const CollisionRequest& request, CollisionResult& result)
{
  GJKSolver solver;
  solver.enable_cached_guess = request.enable_cached_gjk_guess;
  if (solver.enable_cached_guess) {
    solver.cached_guess = request.cached_gjk_guess;
    solver.support_func_cached_guess = request.cached_support_func_guess;
  }

  const CollisionFunctionMatrix& looktable = getCollisionFunctionLookTable();
  std::size_t res;
  if(request.num_max_contacts == 0)
  {
    std::cerr << "Warning: should stop early as num_max_contact is " << request.num_max_contacts << " !" << std::endl;
    res = 0;
  }
  else
  {
    OBJECT_TYPE object_type1 = o1->getObjectType();
    OBJECT_TYPE object_type2 = o2->getObjectType();
    NODE_TYPE node_type1 = o1->getNodeType();
    NODE_TYPE node_type2 = o2->getNodeType();

    if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
    {
      if(!looktable.collision_matrix[node_type2][node_type1])
      {
        std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
        res = 0;
      }
      else
      {
        res = looktable.collision_matrix[node_type2][node_type1](o2, tf2, o1, tf1, &solver, request, result);
        result.swapObjects();
      }
    }
    else
    {
      if(!looktable.collision_matrix[node_type1][node_type2])
      {
        std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
        res = 0;
      }
      else
        res = looktable.collision_matrix[node_type1][node_type2](o1, tf1, o2, tf2, &solver, request, result);
    }
  }
  if (solver.enable_cached_guess) {
    result.cached_gjk_guess = solver.cached_guess;
    result.cached_support_func_guess = solver.support_func_cached_guess;
  }

  return res;
}

ComputeCollision::ComputeCollision(const CollisionGeometry* o1,
    const CollisionGeometry* o2)
  : o1(o1), o2(o2)
{
  const CollisionFunctionMatrix& looktable = getCollisionFunctionLookTable();

  OBJECT_TYPE object_type1 = o1->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type2 = o2->getNodeType();

  swap_geoms = object_type1 == OT_GEOM && object_type2 == OT_BVH;

  if(   ( swap_geoms && !looktable.collision_matrix[node_type2][node_type1])
     || (!swap_geoms && !looktable.collision_matrix[node_type1][node_type2]))
  {
    std::ostringstream oss;
    oss << "Warning: collision function between node type " << node_type1 <<
      " and node type " << node_type2 << " is not supported";
    throw std::invalid_argument(oss.str());
  }
  if (swap_geoms)
    func = looktable.collision_matrix[node_type2][node_type1];
  else
    func = looktable.collision_matrix[node_type1][node_type2];
}

} // namespace fcl
} // namespace hpp
