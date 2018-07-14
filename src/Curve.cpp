/*
 * Curve.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: dumbledore
 */
#include <vector>
#include <cmath>
#include "Curve.hpp"
#include "Eigen-3.3/Core"
#include "Eigen-3.3/Dense"
#include "Eigen-3.3/LU"
Curve::Curve() {

}

Curve::~Curve() {

}

std::vector<double> Curve::getJerkOptimalCurve(std::vector<double> start, std::vector<double> end, double span, unsigned nSteps)
    {
  // alpha_0 through alpha_2 are known from the initial conditions
    double alpha_0 = start[0]; //si
    double alpha_1 = start[1]; // sidot
    double alpha_2 = 0.5 * start[2]; // sidotdot

    // initialize alpha_3, alpha_4, and alpha_5
    double alpha_3 = 0.0;
    double alpha_4 = 0.0;
    double alpha_5 = 0.0;

    Eigen::Vector3d B(alpha_3, alpha_4, alpha_5);
    Eigen::MatrixXd A(3, 3); // a 3x3 matrix

    double TPower2 = std::pow(span, 2.0);
    double TPower3 = TPower2 * span;
    double TPower4 = TPower3 * span;
    double TPower5 = TPower4 * span;

    A << TPower3, TPower4, TPower5, 3*TPower2, 4*TPower3, 5*TPower4, 6*span, 12*TPower2, 20*TPower3;

    double sf= end[0];
    double sfdot = end[1];
    double sfdotdot = end[2];

    Eigen::Vector3d C(3, 1); // a 3x1 matrix
    C << sf - ( alpha_0 + alpha_1 * span + alpha_2 *  TPower2 ), sfdot - (alpha_1 + 2 * alpha_2 * span), sfdotdot - 2*alpha_2 ;

    // Solve for B: A*B = C
    B = A.lu().solve(C);

    alpha_3 = B(0);
    alpha_4 = B(1);
    alpha_5 = B(2);



    /*std::vector<double> result{alpha_0, alpha_1, alpha_2, B(0), B(1), B(2)};*/
    std::vector<double> result;
    double currentS;
    double sampleTime = span/static_cast<double>(nSteps);
    double currentTimeStep = 0;
    double tPower2;
    double tPower3;
    double tPower4;
    double tPower5;
    for(unsigned i = 0; i < nSteps; ++i)
    {

      tPower2 = std::pow(currentTimeStep, 2.0);
      tPower3 = tPower2 * currentTimeStep;
      tPower4 = tPower3 * currentTimeStep;
      tPower5 = tPower4 * currentTimeStep;

      currentS = alpha_0 + (alpha_1 * currentTimeStep) + (alpha_2 * tPower2) + (alpha_3 * tPower3) + (alpha_4 * tPower4) + (alpha_5 * tPower5);
      result.push_back(currentS);
      currentTimeStep = currentTimeStep + sampleTime;
    }


    // compute array of points in the domain of the problem, e.g., s[0], s[1], s[2], until the horizon

   return (result);
    }
