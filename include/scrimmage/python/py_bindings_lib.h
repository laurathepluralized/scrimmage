/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PYTHON_PY_BINDINGS_LIB_H_
#define INCLUDE_SCRIMMAGE_PYTHON_PY_BINDINGS_LIB_H_

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <vector>
#include <utility>
#include <string>

namespace py = pybind11;
namespace scrimmage {


struct EnvParams {
    std::vector<double> discrete_count;
    std::vector<std::pair<double, double>> continuous_extrema;
};

struct EnvValues {
    std::vector<int> discrete;
    std::vector<double> continuous;
};

// namespace py_bindings_lib {

//void to_continuous(std::vector<std::pair<double, double>> &p,
//                                       pybind11::list &minima,
//                                       pybind11::list &maxima) {
//    for (auto &value : p) {
//        pybind11::list min_max;
//        minima.append(value.first);
//        maxima.append(value.second);
//    }
//}
//
//
//void to_discrete(std::vector<double> &p, py::list &maxima) {
//    for (auto &value : p) {
//        maxima.append(value);
//    }
//}


//py::object create_obs(py::list &discrete_count, py::list &continuous_maxima) {
//    int len_discrete = py::len(discrete_count);
//    int len_continuous = py::len(continuous_maxima);
//
//    py::array_t<int> discrete_array(len_discrete);
//    py::array_t<double> continuous_array(len_continuous);
//
//    if (len_discrete > 0 && len_continuous > 0) {
//        py::list obs;
//        obs.append(discrete_array);
//        obs.append(continuous_array);
//        return obs;
//    } else if (len_continuous > 0) {
//        return continuous_array;
//    } else {
//        return discrete_array;
//    }
//}


// }  // namespace py_bindings_lib

namespace autonomy {
class ScrimmageOpenAIAutonomy;
// using ScrimmageOpenAIAutonomyPtr = std::shared_ptr<scrimmage::autonomy::ScrimmageOpenAIAutonomy>;
}  // namespace autonomy

namespace sensor {
class ScrimmageOpenAISensor;
// using ScrimmageOpenAISensorPtr = std::shared_ptr<scrimmage::sensor::ScrimmageOpenAISensor>;
}  // namespace sensor

}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_PYTHON_PY_BINDINGS_LIB_H_
