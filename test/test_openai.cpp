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
    // since RapidXML requires awkward patching
    scrimmage::MissionParse mp_;

    // Pybind stuff
    // py::module np = py::module::import("numpy");
    /* pybind11::object py_act_fcn; */
    /* pybind11::object asarray = np.attr("asarray"); */
    /* py::object np_array = np.attr("array"); */
    /* py::object np_float32 = np.attr("float32"); */

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

//    void read_original_mission(std::string mission) {
//        auto res = sc::FileSearch().find_mission(mission);
//        if (!res) {
//            cout << "Mission " << mission << " not found!" << endl;
//        } else {
//            cout << "Mission is " << mission << endl;
//            cout << "at " << *res << endl;
//        }
//
//        // TODO: Replace this with pybind ElementTree
//        if (!mp_.parse(*res)) {
//            cout << "Mission parsing failed!" << endl;
//            return;
//        }
//    }
//
//    void rewrite_mission(const std::string& newmission) {
//        // TODO: Replace this with pybind ElementTree
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
//    void write_temp_mission(const std::string& mission, bool x_discrete_,
//            bool ctrl_y_, bool y_discrete_, int num_actors_, double end_) {
//        tree = ET.parse(scrimmage.find_mission(MISSION_FILE))
//        root = tree.getroot()
//
//        run_node = root.find("run")
//        run_node.attrib['end'] = str(end)
//
//        entity_common_node = root.find('entity_common')
//        autonomy_node = entity_common_node.find('autonomy')
//        autonomy_node.attrib['x_discrete'] = str(x_discrete_)
//        autonomy_node.attrib['y_discrete'] = str(y_discrete_)
//        autonomy_node.attrib['ctrl_y'] = str(ctrl_y_)
//
//        if (num_actors_ == 2) {
//            entity_node2 = copy.deepcopy(root.find('entity'))
//            root.append(entity_node2)
//        }
//
//        tree.write(TEMP_MISSION_FILE)
//    }

//    void get_module_name_from_test() {
//        // this neat trick courtesy of
//        // https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#getting-the-current-tests-name
//        module_ = ::testing::UnitTest::GetInstance()->current_test_info()->name();
//    }


    std::string module_;
    std::string act_fcn_;
    bool x_discrete_;
    int num_actors_;
    double end_time_;

};



// Create test fixture
// Maybe derived test fixtures to modularize different aspects to test, though probably not necessary
// https://github.com/google/googletest/blob/master/googletest/docs/faq.md#can-i-derive-a-test-fixture-from-another


//TEST_F(OpenAIPybindTesting, test_write_file) {
//    const std::string mission = "straight.xml";
//    read_original_mission(mission);
//    const std::string newmission = ".straight.xml";
//    rewrite_mission(newmission);

    // auto log_dir = sc::run_test(mission);
    //
    // bool success = log_dir ? true : false;
    // EXPECT_TRUE(success);
    // if (!log_dir) return;
    //
    // sc::CSV csv;
    // bool summary_found = csv.read_csv(*log_dir + "/summary.csv");
    // EXPECT_TRUE(summary_found);
    // if (!summary_found) return;
    //
    // const int row = csv.rows() - 1;
    // double collisions = csv.at(row, "team_coll");
    // EXPECT_GT(collisions, 0); // expect collisions
//}


TEST_F(OpenAIPybindTesting, test_one_dim_discrete) {

    std::cout << "starting test" << std::endl;
    auto log_dir = sc::run_test("test_one_dim_discrete.xml");
//    """A single agent along the x-axis."""
    std::cout << "ran test" << std::endl;
    bool success = log_dir ? true : false;
    std::cout << "checked for log dir" << std::endl;
    EXPECT_TRUE(success);
    if (!log_dir) return;
    std::cout << "found log dir" << std::endl;

    sc::CSV csv;
    bool summary_found = csv.read_csv(*log_dir + "/summary.csv");
    EXPECT_TRUE(summary_found);
    std::cout << "checked for summary" << std::endl;
    if (!summary_found) return;
    std::cout << "found summary" << std::endl;

    const int row = csv.rows() - 1;
    double total_reward = csv.at(row, "reward");
    EXPECT_EQ(total_reward, 4);
}

/*
TEST_F(OpenAIPybindTesting, test_two_dim_discrete) {
    """A single agent along the x and y-axis."""
    def _get_action(i):
        return np.array([1, 1] if i < 100 else [0, 0], dtype=int)

    std::cout << "scrimmage-v1" << std::endl;
    _write_temp_mission(x_discrete=True, ctrl_y=True, y_discrete=True,
                        num_actors=1, end=1000)
    combine_actors = False
    global_sensor = False
    env, obs, total_reward = \
        _run_test(VERSION, combine_actors, global_sensor, _get_action)

    assert len(obs[0]) == 1
    assert obs[0] == 0
    assert isinstance(env.action_space, gym.spaces.MultiDiscrete)
    assert isinstance(env.observation_space, gym.spaces.Box)
    assert np.array_equal(env.action_space.nvec, np.array([2, 2], dtype=int))
    // EXPECT_EQ(total_reward, 4);
}


TEST_F(OpenAIPybindTesting, test_two_dim_continuous) {
    """A single agent along the x and y-axis with continuous input."""
    def _get_action(i):
        return np.array([1.0, 1.0] if i < 100 else [-1.0, -1.0], dtype=float)

    std::cout << "scrimmage-v2" << std::endl;
    _write_temp_mission(x_discrete=False, ctrl_y=True, y_discrete=False,
                        num_actors=1, end=1000)
    combine_actors = False
    global_sensor = False
    env, obs, total_reward = \
        _run_test(VERSION, combine_actors, global_sensor, _get_action)

    assert len(obs[0]) == 1
    assert obs[0] == 0
    assert isinstance(env.action_space, gym.spaces.Box)
    assert isinstance(env.observation_space, gym.spaces.Box)
    // EXPECT_EQ(total_reward, 4);
}


TEST_F(OpenAIPybindTesting, test_two_dim_tuple) {
    """Single agent with discrete and continuous input."""
    def _get_action(i):
        return np.array([[1], [1.0]] if i < 100 else [[0], [-1.0]])

    std::cout << "scrimmage-v3" << std::endl;
    _write_temp_mission(x_discrete=False, ctrl_y=True, y_discrete=True,
                        num_actors=1, end=1000)
    combine_actors = False
    global_sensor = False
    env, obs, total_reward = \
        _run_test(VERSION, combine_actors, global_sensor, _get_action)

    assert len(obs[0]) == 1
    assert obs[0] == 0
    assert isinstance(env.action_space, gym.spaces.Tuple)
    assert isinstance(env.observation_space, gym.spaces.Box)
    // EXPECT_EQ(total_reward, 4);
}


TEST_F(OpenAIPybindTesting, test_one_dim_continuous) {
    """A single agent along the x-axis with continuous input."""
    def _get_action(i):
        return 1.0 if i < 100 else -1.0

    std::cout << "scrimmage-v4" << std::endl;
    _write_temp_mission(x_discrete=False, ctrl_y=False, y_discrete=False,
                        num_actors=1, end=1000)
    combine_actors = False
    global_sensor = False
    env, obs, total_reward = \
        _run_test(VERSION, combine_actors, global_sensor, _get_action)

    assert len(obs[0]) == 1
    assert obs[0] == 0
    assert isinstance(env.action_space, gym.spaces.Box)
    assert isinstance(env.observation_space, gym.spaces.Box)
    // EXPECT_EQ(total_reward, 4);
}


TEST_F(OpenAIPybindTesting, test_two_combined_veh_dim_discrete) {
    """Two agents, each with one discrete input, treated as a single agent."""
    def _get_action(i):
        return [1, 0] if i < 100 else [0, 1]

    std::cout << "scrimmage-v5" << std::endl;
    _write_temp_mission(x_discrete=True, ctrl_y=False, y_discrete=False,
                        num_actors=2, end=1000)
    combine_actors = True
    global_sensor = False
    env, obs, total_reward = \
        _run_test(VERSION, combine_actors, global_sensor, _get_action)

    assert len(obs[0]) == 2
    assert np.array_equal(obs[0], np.array([0., 0.]))
    assert isinstance(env.action_space, gym.spaces.MultiDiscrete)
    assert isinstance(env.observation_space, gym.spaces.Box)
    // EXPECT_EQ(total_reward, 8);
}


TEST_F(OpenAIPybindTesting, test_two_not_combined_veh_dim_discrete) {
    """Two agents, each with discrete input, treated as separate agents."""
    def _get_action(i):
        return [[1], [0]] if i < 100 else [[0], [1]]

    std::cout << "scrimmage-v6" << std::endl;
    _write_temp_mission(x_discrete=True, ctrl_y=False, y_discrete=False,
                        num_actors=2, end=1000)
    combine_actors = False
    global_sensor = False
    env, obs, total_reward = \
        _run_test(VERSION, combine_actors, global_sensor, _get_action)

    assert len(obs[0]) == 2
    assert obs[0][0] == 0
    assert obs[0][1] == 0
    assert isinstance(env.action_space, gym.spaces.Tuple)
    assert isinstance(env.observation_space, gym.spaces.Tuple)
    // EXPECT_EQ(total_reward, 8);
}


TEST_F(OpenAIPybindTesting, test_two_combined_veh_dim_discrete_global_sensor) {
    """Two agents, each with discrete input, treated as a single agent.

    global_sensor means that the sensor of the first agent will define the
    state.
    """
    def _get_action(i):
        return [1, 0] if i < 100 else [0, 1]

    std::cout << "scrimmage-v7" << std::endl;
    _write_temp_mission(x_discrete=True, ctrl_y=False, y_discrete=False,
                        num_actors=2, end=1000)
    combine_actors = True
    global_sensor = True
    env, obs, total_reward = \
        _run_test(VERSION, combine_actors, global_sensor, _get_action)

    assert len(obs[0]) == 1
    assert np.array_equal(obs[0], np.array([0.]))
    assert isinstance(env.action_space, gym.spaces.MultiDiscrete)
    assert isinstance(env.observation_space, gym.spaces.Box)
    // EXPECT_EQ(total_reward, 8);
}
*/


