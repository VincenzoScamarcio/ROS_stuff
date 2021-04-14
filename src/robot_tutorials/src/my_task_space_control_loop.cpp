#include "state_representation/space/cartesian/CartesianState.hpp"
#include "dynamical_systems/Linear.hpp"
#include <chrono>
#include <thread>

using namespace state_representation;
using namespace dynamical_systems;
using namespace std::chrono_literals;

int main(int, char**) {

  auto dt = 10ms;
  double tol = 1e-3;
  
  //doesn't work using this definition 
  //CartesianState mf("moving_frame"), tf("target_frame");
  //mf.set_pose(Eigen::VectorXd::Random(7));
  //tf.set_pose(Eigen::VectorXd::Random(7));
  
  //it works using this definition
  //CartesianPose mf = CartesianPose::Random("moving_frame"); 
  //CartesianPose tf = CartesianPose::Random("target_frame");
  
  //it works if I manually define pose of the frames
  CartesianState mf("moving_frame");
  mf.set_position(Eigen::Vector3d(0,1,0));  
  mf.set_orientation(Eigen::Quaterniond(1,0,0,0));
  CartesianState tf("target_frame");
  tf.set_position(Eigen::Vector3d(1,0,0));  
  tf.set_orientation(Eigen::Quaterniond(0,1,0,0));
  
  std::cout << mf << std::endl;
  std::cout << tf << std::endl;
  
  double distance_p = dist(mf,tf,CartesianStateVariable::POSITION);
  double distance_o = dist(mf,tf,CartesianStateVariable::ORIENTATION);
  std::cout << "distance in position to attractor: " << std::to_string(distance_p) << std::endl;
  std::cout << "distance in orientation to attractor: " << std::to_string(distance_o) << std::endl;
  
  Linear<CartesianState> ls(tf);
  double distance;
  do {
     CartesianTwist desired_twist = ls.evaluate(mf);
     std::cout << "The desired linear & angular velocity to reach target_frame is: \n"         <<desired_twist << "\n" << std::endl;
     CartesianState new_mf = mf + dt * desired_twist;
     std::cout << new_mf << std::endl;
     distance = dist(new_mf,tf,CartesianStateVariable::POSE);
     std::cout << "distance in position + orientation to attractor: " << std::to_string(distance) << std::endl;
     std::cout << "-----------" << std::endl;
     mf = new_mf;
     std::this_thread::sleep_for(dt);
  } while (distance > tol) ;
  
  std::cout << "##### TARGET #####" << std::endl;
  std::cout << tf << std::endl;
  std::cout << "##### CURRENT POSE #####" << std::endl;
  std::cout << mf << std::endl;
  // std::cout << new_mf << std::endl;
  
  
  return 0;
}
