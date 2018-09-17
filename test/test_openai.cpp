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
    scrimmage::MissionParse mp_;

 protected:
    rapidxml::xml_document<> doc_;
    std::string xml_content_ = "";

    void read_original_mission(std::string mission) {
        auto res = sc::FileSearch().find_mission(mission);
        if (!res) {
            cout << "Mission " << mission << " not found!" << endl;
        } else {
            cout << "Mission is " << mission << endl;
            cout << "at " << *res << endl;
        }

        if (!mp_.parse(*res)) {
            cout << "Mission parsing failed!" << endl;
            return;
        }
    }

    void rewrite_mission(const std::string& newmission) {
        std::ofstream temp_mission(newmission);
        cout << "newmission is " << newmission << endl;

        mp_.xml_doc(doc_, xml_content_);
        std::cout << "what's up " << xml_content_;
        temp_mission << doc_;
        cout << "Check output file!" << endl;
        temp_mission.close();
    }

    bool x_discrete_;
    int num_actors_;
    double end_time_;

};



// Create test fixture
// Maybe derived test fixtures to modularize different aspects to test, though probably not necessary
// https://github.com/google/googletest/blob/master/googletest/docs/faq.md#can-i-derive-a-test-fixture-from-another


TEST_F(OpenAIPybindTesting, test_write_file) {
    const std::string mission = "straight.xml";
    read_original_mission(mission);
    const std::string newmission = ".straight.xml";
    rewrite_mission(newmission);

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
}
