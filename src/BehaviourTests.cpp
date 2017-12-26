/*
 * BehaviourTests.cpp
 *
 *  Created on: Dec 24, 2017
 *      Author: jyarde
 */

#include "gtest/gtest.h"
#include "Behavior.h"
#include "Trajectory.h"
#include <iostream>
#include <fstream>
#include "matplotlibcpp.h"

using namespace std;

namespace plt = matplotlibcpp;

inline void plot_vector(vector<double> v, string title) {
  plt::subplot(1,1,1);
  plt::title(title);
  plt::plot(v);
  plt::show();
}

namespace {
  class BehaviorTest : public testing::Test {
    public:

    TrajectoryPlanner trajectory;
    Behavior behavior;

    BehaviorTest() {}

    void SetUp( ) {
    }

    void TearDown( ) {
    }
  };

  TEST_F(BehaviorTest, Behavior) {
    ASSERT_TRUE((int)FSM::KE == 0);
    //ASSERT_TRUE(classifier.label_index("left") == 0);
  }

}
