/*
 * Behavior.h
 *
 *  Created on: Oct 16, 2017
 *      Author: jyarde
 */

#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>

using namespace std;

/**
 * Use Process Model driven prediction to assess what each vehicle on the highway will do.
 * Vehicle Behaviours to be considered: keep lane, turn left, turn right
 * Use Naive Bayes as multi-modal estimator to update predictions on each vehicle and each behavior
 *
 * Process:
 * - Identify behaviors
 * - define a process model for each behavior
 * - Use process model to compute the probability of each behavior:
 * 		- use observations at t-1 in process models to predict expected state
 * 		- compare expected state to observed state at time to update belief of each state
 * -Use Multi-modal estimation algorithm to derive probability of each behavior
 * -Predict a trajectory for each behavior
 */
class Predictor {

public:
	Predictor();

	virtual ~Predictor();

	/**
	 * Multimodal Estimator
	 * Updates predictions of environmental actions (cars, pedestrians etc)
	 * based on prior belief (and inherent uncertainty) and current observations.
	 */
	void multimodal_estimator();

	/**
	 * Naive Bayes Estimator
	 * Uses prior knowledge of action probabilities to update probabilities
	 */
	void naivebayes_esimator();

};



#endif /* PREDICTOR_H_ */
