#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
   VectorXd rmse(4);
   rmse << 0, 0, 0, 0;

   // TODO: YOUR CODE HERE
   // check the validity of the following inputs:
   //  * the estimation vector size should not be zero
   //  * the estimation vector size should equal ground truth vector size

   if (estimations.size() == 0 || ground_truth.size() == 0)
   {
      std::cout << "vector size is zero" << std::endl;
      return rmse;
   }

   if (estimations.size() != ground_truth.size())
   {
      std::cout << "vector sizes do not agree" << std::endl;
      return rmse;
   }

   // TODO: accumulate squared residuals
   for (uint i = 0; i < estimations.size(); i++)
   {
      const VectorXd &es = estimations[i];
      const VectorXd &gt = ground_truth[i];

      VectorXd d = es.array() - gt.array();
      VectorXd dd = d.array() * d.array();
      // ... your code here

      rmse += dd;
   }

   rmse /= (float)estimations.size();
   rmse = rmse.array().sqrt();

   // TODO: calculate the mean

   // TODO: calculate the squared root

   // return the result
   return rmse;
}

Eigen::VectorXd Tools::Hf(const Eigen::VectorXd &mnt_laser)
{

   double px = mnt_laser[0];
   double py = mnt_laser[1];

   double vx = mnt_laser[2];
   double vy = mnt_laser[3];

   Eigen::VectorXd polar(3);

   double angle = atan2(py, px);

   //while(angle >  M_PI) angle -= 2.0 * M_PI;
   //while(angle < -M_PI) angle += 2.0 * M_PI;

   /* while ( angle > M_PI || angle < -M_PI )
  {
    if (angle > M_PI ) 
    {
     	angle -= M_PI;
    } 
    else 
    {
        angle += M_PI;
    }
  }*/

   float dd = sqrt(px * px + py * py);

   polar(0) = dd;
   polar(1) = angle;

   if (dd == 0)
   {
      std::cout << "error: division by zero" << std::endl;
      polar(2) = 0;
   }
   else
   {
      polar(2) = (px * vx + py * vy) / dd;
   }

   return polar;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{

   Eigen::MatrixXd Hj(3, 4);
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // check division by zero
   double pp = px * px + py * py;

   if (pp <= 0)
   {
      std::cout << "error: division by zero" << std::endl;
      return Hj;
   }

   // compute the Jacobian matrix
   Hj << 1, 1, 0, 0,
       1, 1, 0, 0,
       1, 1, 1, 1;

   Hj(0, 0) = px / sqrt(pp);
   Hj(0, 1) = py / sqrt(pp);

   Hj(1, 0) = -py / pp;
   Hj(1, 1) = px / pp;

   Hj(2, 0) = py * (vx * py - vy * px) / pow(pp, 3 / 2.0);
   Hj(2, 1) = px * (vy * px - vx * py) / pow(pp, 3 / 2.0);

   Hj(2, 2) = px / sqrt(pp);
   Hj(2, 3) = py / sqrt(pp);

   return Hj;
}

