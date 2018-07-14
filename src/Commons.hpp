/*
 * Commons.hpp
 *
 *  Created on: Mar 25, 2018
 *      Author: dumbledore
 */

#ifndef COMMONS_HPP_
#define COMMONS_HPP_

class Commons {
 public:
  Commons();
  virtual ~Commons();

  // define the States
  enum class LaneChange {
    KeepLane = 0u,
    Move_Left = 1u,
    Move_Right = 2u
  };



};

#endif /* COMMONS_HPP_ */
