/**
 * @file tesseract_environment_python.i
 * @brief The tesseract_environment_python SWIG master file.
 *
 * @author John Wason
 * @date December 8, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

%module(directors="1", package="tesseract.tesseract_environment") tesseract_environment_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_kinematics_python.i"
%import "tesseract_collision_python.i"

%{

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>

#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>

// tesseract_environment
#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/commands.h>
#include <tesseract_environment/core/state_solver.h>
#include <tesseract_environment/core/environment.h>

#include "tesseract_scene_graph_python_std_functions.h"
#include "tesseract_collisions_python_std_functions.h"
#include "tesseract_environment/kdl/kdl_state_solver.h"

#include "tesseract_environment_python_std_functions.h"
%}

%tesseract_std_function(FindTCPCallbackFn,tesseract,Eigen::Isometry3d,const tesseract_common::ManipulatorInfo&,a);

// tesseract_environment
#define TESSERACT_ENVIRONMENT_CORE_PUBLIC
%include "tesseract_environment/core/types.h"
%include "tesseract_environment/core/commands.h"
%include "tesseract_environment/core/manipulator_manager.h"
%include "tesseract_environment/core/state_solver.h"
%include "tesseract_environment/core/environment.h"

%extend tesseract_environment::Environment {
    bool init(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_scene_graph::SRDFModel::ConstPtr& srdf_model = nullptr)
    {
        $self->init<tesseract_environment::KDLStateSolver>(scene_graph,srdf_model);
    }
}