#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools
{
public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                       const std::vector<Eigen::VectorXd> &ground_truth);

  static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);
  static Eigen::MatrixXd CalculateJacobian_2(const Eigen::VectorXd &x_state);
  static Eigen::VectorXd Hf(const Eigen::VectorXd &measurement);
};

#endif // TOOLS_H_
