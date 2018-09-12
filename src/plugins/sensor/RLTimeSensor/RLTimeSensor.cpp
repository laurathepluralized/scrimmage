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

#include <scrimmage/plugins/sensor/RLTimeSensor/RLTimeSensor.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>

#include <iostream>

REGISTER_PLUGIN(scrimmage::Sensor, scrimmage::sensor::RLTimeSensor, RLTimeSensor_plugin)

namespace scrimmage {
namespace sensor {

void RLTimeSensor::get_observation(double *data, uint32_t beg_idx, uint32_t /*end_idx*/) {
    data[beg_idx] = time_->t();
    std::cout << "time is " << time_->t() << std::endl;
}

void RLTimeSensor::set_observation_space() {
    const double inf = std::numeric_limits<double>::infinity();
    observation_space.continuous_extrema.push_back(std::make_pair(0, inf));
}
} // namespace sensor
} // namespace scrimmage
