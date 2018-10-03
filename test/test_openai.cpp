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

#include <gtest/gtest.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <scrimmage/python/py_bindings_lib.h>
#include <scrimmage/plugins/sensor/ScrimmageOpenAISensor/ScrimmageOpenAISensor.h>
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/common/CSV.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <iostream>
#include <fstream>
#include <string>

#include <boost/optional.hpp>
#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_print.hpp>

namespace sc = scrimmage;
using std::cout;
using std::endl;

class OpenAIPybindTesting : public testing::Test {
 public:
    // TODO: Replace MissionParse with Pybind-ed (Pybound?) ElementTree XML I/O
    // since RapidXML's rapidxml_print.hpp requires awkward patching
    scrimmage::MissionParse mp_;

//    // Pybind stuff
//    py::module np = py::module::import("numpy");
//    pybind11::object py_act_fcn;
//    pybind11::object asarray = np.attr("asarray");
//    py::object np_array = np.attr("array");
//    py::object np_float32 = np.attr("float32");

//    // get model python module
//    const std::string module = params.at("module");
//    py::object py_module = py::module::import(module.c_str());
//
//    // model's action function
//    const std::string py_act_fcn_str = params.at("act_fcn");
//    py_act_fcn = py_module.attr(py_act_fcn_str.c_str());

 protected:
    rapidxml::xml_document<> doc_;
    std::string xml_content_ = "";
    // The intention here is to read in the original base mission file,
    // then get the name of the module and action function we want to test
    // and change those params in a new temporary mission file based on the
    // old one. Then run a mission test for that file and check that metrics'
    // results are as expected.

//    void read_original_mission(std::string mission) {
//        auto res = sc::FileSearch().find_mission(mission);
//        if (!res) {
//            cout << "Mission " << mission << " not found!" << endl;
//        } else {
//            cout << "Mission is " << mission << endl;
//            cout << "at " << *res << endl;
//        }
//
//        // TODO: Replace this to avoid awkward RapidXML print patching
//        if (!mp_.parse(*res)) {
//            cout << "Mission parsing failed!" << endl;
//            return;
//        }
//    }
//
//    void rewrite_mission(const std::string& newmission) {
//        // TODO: Replace this to avoid awkward RapidXML print patching
//        std::ofstream temp_mission(newmission);
//        cout << "newmission is " << newmission << endl;
//
//        mp_.xml_doc(doc_, xml_content_);
//        std::cout << "xml content is " << xml_content_;
//        temp_mission << doc_;
//        cout << "Check output file!" << endl;
//        temp_mission.close();
//    }
//
//    void get_module_name_from_test() {
//        // this neat trick courtesy of
//        // https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#getting-the-current-tests-name
//        // plan is to have the test name correspond to the python module name
//        module_ = ::testing::UnitTest::GetInstance()->current_test_info()->name();
//    }

    std::string module_;
    std::string act_fcn_;
    bool x_discrete_;
    int num_actors_;
    double end_time_;
};


// TEST_F(OpenAIPybindTesting, test_write_file) {
//    const std::string mission = "straight.xml";
//    read_original_mission(mission);
//    const std::string newmission = ".straight.xml";
//    // modify stuff in mp_ for the new mission we will write in next line
//    rewrite_mission(newmission);
//}


TEST_F(OpenAIPybindTesting, test_one_dim_discrete) {
    //  A single agent along the x-axis.
    auto log_dir = sc::run_test("test_one_dim_discrete.xml");
    bool success = log_dir ? true : false;
    EXPECT_TRUE(success);
    if (!log_dir) return;

    sc::CSV csv;
    bool summary_found = csv.read_csv(*log_dir + "/summary.csv");
    EXPECT_TRUE(summary_found);
    if (!summary_found) return;

    const int row = csv.rows() - 1;
    double total_reward = csv.at(row, "reward");
    EXPECT_EQ(total_reward, 4);
    // TODO: either output other metrics to summary.csv from RLSimple,
    // or have RLSimple send message with reward and other metrics to
    // RLSimpleMetrics and have it output those to the summary file.
    // Then compare summary contents to expected values.
    // Metrics:
    // - total reward (done)
    // - size of first observation
    // - value of first observation (may need to check [0][0] and [0][1],
    //       depending on observation space type)
    // - What gym.spaces type is the action space
    // - What gym.spaces type is the observation space
    // - contents of action space nparray
}

