#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse = VectorXd::Zero(4);
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
  	return rmse;
  }

  int est_size = estimations.size();
  for (int i = 0; i < est_size; i++) {
  	VectorXd err = estimations[i] - ground_truth[i];
  	err = err.array() * err.array();
  	rmse += err;
  }
  
  rmse /= est_size;
  rmse = rmse.array().sqrt();
  return rmse;
}