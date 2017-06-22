/*
Ce noeud a pour but de récupérer les informations du topic Joint_States pour reconstruire les données odométriques du modèle bicyclette et de les publier dans un topic.
Ces données sont les braquages avant et arrière, les vitesses de braquage avant et arrière, la vitesse du véhicule, son accélération et son jerk.
Ce fichier .cpp permet de décrire les méthodes utilisées dans le noeud.
*/

#include "four_wheel_steering_odometry.hpp"


OdometryJointMessage::OdometryJointMessage(ros::NodeHandle &nb):
  nh_(nb),
  track_(0.0), wheel_base_(0.0), wheel_radius_(0.0), front_steering_angle_(0.0), rear_steering_angle_(0.0), front_steering_angle_velocity_(0.0), rear_steering_angle_velocity_(0.0),
  speed_(0.0), speedprevious_(0.0), acceleration_(0.0), jerk_(0.0), accelerationprevious_(0.0), previous_front_left_steering_angle_(0.0), previous_front_right_steering_angle_(0.0),
  previous_rear_left_steering_angle_(0.0), previous_rear_right_steering_angle_(0.0)

{
}

void OdometryJointMessage::odomJointCallback (const sensor_msgs::JointState::ConstPtr& val)
{
  /// Création des variables qui seront utilisées pour calculer nos données
  double front_left_steering_angle=val->position[0];
  double front_right_steering_angle=val->position[2];
  double rear_left_steering_angle=val->position[4];
  double rear_right_steering_angle=val->position[6];

  double front_left_wheel_velocity=val->velocity[1];
  double front_right_wheel_velocity=val->velocity[3];
  double rear_left_wheel_velocity=val->velocity[5];
  double rear_right_wheel_velocity=val->velocity[7];

//  double front_left_wheel_angle_velocity=val->velocity[0];
//  double front_right_wheel_angle_velocity=val->velocity[2];
//  double rear_left_wheel_angle_velocity=val->velocity[4];
//  double rear_right_wheel_angle_velocity=val->velocity[6];

  /// création de variables temporelles pour les dérivées qui seront calculer
  ros::Duration ros_diff_time = ros::Time::now() - previous;
  double diff_time = ros_diff_time.toSec();




  /// Calcul du front et rear steering angle
  front_steering_angle_=2/((1/tan(front_left_steering_angle))+(1/tan(front_right_steering_angle)));
  rear_steering_angle_=2/((1/tan(rear_left_steering_angle))+(1/tan(rear_right_steering_angle)));

  /// Calcul de la vitesse, de l'accélération et du jerk

  // Formule exacte mais utilisant thetap qui n'est pas implémenté pour l'instant
//  speed_=sqrt(pow((front_left_wheel_velocity_*wheel_radius_),2)-pow((wheel_base_*thetap_/2),2))+thetap_*track_/2;

  speed_=((front_left_wheel_velocity+front_right_wheel_velocity+rear_left_wheel_velocity+rear_right_wheel_velocity)/4)*wheel_radius_;
  acceleration_=(speed_-speedprevious_)/diff_time;
  jerk_=(acceleration_-accelerationprevious_)/diff_time;

  //mise en mémoire des valeurs
  speedprevious_=speed_;
  accelerationprevious_=acceleration_;
  previous = ros::Time::now();


  /// Calcul du front et rear steering velocity

  // Méthode avec les informations du topic joint_states // Ne marche pas pour l'instant
//  front_steering_angle_velocity_=sqrt(pow(thetap_*wheel_base_/2,2)+pow(speed_,2))/wheel_radius_; // 2 formules au cas où 2 rayons de roues différents (mais variables à changer)
//  rear_steering_angle_velocity_=sqrt(pow(thetap_*wheel_base_/2,2)+pow(speed_,2))/wheel_radius_;

  // Méthode avec le taux de variation des angles de braquages
   double front_left_steering_velocity= (front_left_steering_angle-previous_front_left_steering_angle_)/diff_time;
   double front_right_steering_velocity= (front_right_steering_angle-previous_front_right_steering_angle_)/diff_time;
   double rear_left_steering_velocity= (rear_left_steering_angle-previous_rear_left_steering_angle_)/diff_time;
   double rear_right_steering_velocity= (rear_right_steering_angle-previous_rear_right_steering_angle_)/diff_time;

   front_steering_angle_velocity_=(front_left_steering_velocity+front_right_steering_velocity)/2;
   rear_steering_angle_velocity_=(rear_left_steering_velocity+rear_right_steering_velocity)/2;

   previous_front_left_steering_angle_=front_left_steering_angle;
   previous_front_right_steering_angle_=front_right_steering_angle;
   previous_rear_left_steering_angle_=rear_left_steering_angle;
   previous_rear_right_steering_angle_=rear_right_steering_angle;

   // Pour tester les valeurs seuls dans le terminal avec ROS
//  double valeur_test=front_left_steering_velocity;
//  ROS_INFO_STREAM ("valeur test"<<valeur_test); // pour faire des tests sur les valeurs.


}

void OdometryJointMessage::setVehicleParam()
{
  int flag=0; // flag pour compter le nombre de tentative de récupération des valeurs
  while (1)
  {

    bool error = false;

    if (track_ > 0.0 || nh_.getParam("track", track_)) //essai pour récupérer la voie
    {
      ROS_WARN_STREAM("track "<<track_);
    }
    else
      error = true;

    if (wheel_base_ > 0.0 || nh_.getParam("wheel_base", wheel_base_)) // essai pour récupérer l'empattement
    {
      ROS_WARN_STREAM("wheel_base "<<wheel_base_);
    }
    else
      error = true;

    if (wheel_radius_ > 0.0 || nh_.getParam("wheel_radius", wheel_radius_)) // essai pour récupérer le rayon des roue
    {
      ROS_WARN_STREAM("wheel_radius "<<wheel_radius_);
    }
    else
      error = true;

    if(error) // delai pour nouvelle tentative
    {
      flag ++;
      ros::Duration(2).sleep();
    }
    else
      break;

    if(flag==5)
    {
      ROS_WARN_STREAM("unable to get the parameters");
      break;
    }
  }
}

void OdometryJointMessage::odomMessage (four_wheel_steering_msgs::FourWheelSteering &msg)
{
  msg.front_steering_angle=front_steering_angle_;
  msg.rear_steering_angle=rear_steering_angle_;
  msg.front_steering_angle_velocity=front_steering_angle_velocity_;
  msg.rear_steering_angle_velocity=rear_steering_angle_velocity_;
  msg.speed=speed_;
  msg.acceleration=acceleration_;
  msg.jerk=jerk_;
}

