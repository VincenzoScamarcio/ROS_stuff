#include <ros/ros.h>
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "dynamical_systems/Linear.hpp"
#include <chrono>
#include <thread>
#include <cstdio>

using namespace state_representation;
using namespace dynamical_systems;
using namespace std::chrono_literals;
using namespace std;

void control_loop_idle(const std::chrono::nanoseconds& dt,
                       const double& tol,
                       CartesianState mf,
                       CartesianState tf) {
   Linear<CartesianState> ls(tf);
   double distance;
   do {
     CartesianTwist desired_twist = ls.evaluate(mf);
     cout << "The desired linear & angular velocity to reach target_frame is: \n"         <<desired_twist << "\n" << endl;
     CartesianState new_mf = mf + dt * desired_twist;
     cout << new_mf << endl;
     distance = dist(new_mf,tf,CartesianStateVariable::POSE);
     cout << "distance in position + orientation to attractor: " << to_string(distance) << endl;
     cout << "-----------" << endl;
     mf = new_mf;
     this_thread::sleep_for(dt);
   } while (distance > tol) ;
   
   cout << "##### TARGET #####" << endl;
   cout << tf << endl;
   cout << "##### CURRENT POSE #####" << endl;
   cout << mf << endl;
   }

void control_loop(const std::chrono::nanoseconds& dt,
                  const double& tol,
                  CartesianState mf,
                  CartesianState tf) {
   Linear<CartesianState> ls(tf);
   double distance;
   do {
     CartesianTwist desired_twist = ls.evaluate(mf);
     CartesianTwist tf_twist = tf;
     cout << tf_twist << endl;
     //getchar();
     cout << "The desired linear & angular velocity to reach target_frame is: \n"         <<desired_twist << "\n" << endl;
     CartesianState new_mf = mf + dt * desired_twist;
     CartesianState new_tf = tf + dt * tf_twist;
     cout << new_mf << endl;
     distance = dist(new_mf,tf,CartesianStateVariable::POSE);
     cout << "distance in position + orientation to attractor: " << to_string(distance) << endl;
     cout << "-----------" << endl;
     mf = new_mf;
     tf = new_tf;
     this_thread::sleep_for(dt);
   } while (distance > tol) ;
   
   cout << "##### TARGET #####" << endl;
   cout << tf << endl;
   cout << "##### CURRENT POSE #####" << endl;
   cout << mf << endl;
   }


int main(int, char**) {
   auto dt = 10ms;
   double tol = 1e-3;
   string a;
   cout << "Set moving_frame & target_frame pose? (Yes/No)\n";
   CartesianState mf("moving_frame"), tf("target_frame");
   askquestion_1:
   cin >> a;
   if (a.compare("Yes") == 0) {
      cout << "#####Using the specified poses to set frames#####" << endl;
      //CartesianState mf("moving_frame");
      mf.set_position(Eigen::Vector3d(-1,1,0));  
      mf.set_orientation(Eigen::Quaterniond(1,0,0,0));
  
      //CartesianState tf("target_frame");
      tf.set_position(Eigen::Vector3d(1,0,0));  
      tf.set_orientation(Eigen::Quaterniond(0,1,0,0));
      
   } else if (a.compare("No") == 0) {
      cout << "#####Setting the frames to random poses######" << endl;
      mf.set_pose(Eigen::VectorXd::Random(7));
      tf.set_pose(Eigen::VectorXd::Random(7));
      
   } else {
      cout << "Please, enter Yes/No" << endl;
      goto askquestion_1;
   }
   
   cout << mf << endl;
   cout << tf << endl;
   
   cout << "Is target_frame moving? (Yes/No)\n";
   askquestion_2:
   cin >> a;
   if (a.compare("Yes") == 0) {
      cout << "#####Setting target_frame to random twist#####" << endl;
      //CartesianState mf("moving_frame");
      tf.set_twist(Eigen::VectorXd::Random(6));
      control_loop(dt, tol, mf, tf);
      
   } else if (a.compare("No") == 0) {
      cout << "#####Setting idle target_frame######" << endl;
      cout << mf << endl;
      cout << tf << endl;
      control_loop_idle(dt, tol, mf, tf);  
   } else {
      cout << "Please, enter Yes/No" << endl;
      goto askquestion_2;
   }

   return 0;
}
