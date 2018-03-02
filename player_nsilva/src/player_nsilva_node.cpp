#include <math.h>
#include <ctime>
#include <iostream>
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// RWS includes
#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>

using namespace std;

float randomizePosition(float *x, float *y)
{
  srand(static_cast<unsigned>(time(0)));
  *x = (((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5) * 10);
  *y = (((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5) * 10);
}

namespace rws_nsilva
{
class Player
{
public:
  Player(string name)
  {
    this->name = name;
  }

  string name;

  void setTeamName(int index)
  {
    if (index == 0)
      setTeamName("red");
    else if (index == 1)
      setTeamName("green");
    else if (index == 2)
      setTeamName("blue");
    else
      setTeamName("none");
  }

  void setTeamName(string team)
  {
    if (team == "red" || team == "green" || team == "blue")
    {
      team_name = team;
    }
    else
      ROS_ERROR("For player %s team name %s is invalid.", this->name.c_str(), team.c_str());
  }

  string getTeamName(void)
  {
    return this->team_name;
  }

private:
  string team_name;
};

class MyPlayer : public Player
{
public:
  boost::shared_ptr<Team> red_team;
  boost::shared_ptr<Team> green_team;
  boost::shared_ptr<Team> blue_team;

  boost::shared_ptr<Team> my_team;
  boost::shared_ptr<Team> my_preys;
  boost::shared_ptr<Team> my_hunters;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  boost::shared_ptr<ros::Subscriber> sub;
  ros::NodeHandle nh;

  float x, y;

  MyPlayer(string argin_name, string argin_team) : Player(argin_name)
  {
    red_team = boost::shared_ptr<Team>(new Team("red"));
    green_team = boost::shared_ptr<Team>(new Team("green"));
    blue_team = boost::shared_ptr<Team>(new Team("blue"));

    if (red_team->playerBelongsToTeam(name))
    {
      my_team = red_team;
      my_preys = green_team;
      my_hunters = blue_team;
    }
    else if (green_team->playerBelongsToTeam(name))
    {
      my_team = green_team;
      my_preys = blue_team;
      my_hunters = red_team;
    }
    else if (blue_team->playerBelongsToTeam(name))
    {
      my_team = blue_team;
      my_preys = red_team;
      my_hunters = green_team;
    }

    struct timeval t1;
    gettimeofday(&t1, NULL);
    srand(t1.tv_usec);
    double start_x = ((double)rand() / (double)RAND_MAX) * 10 - 5;
    double start_y = ((double)rand() / (double)RAND_MAX) * 10 - 5;
    printf("start_x=%f start_y=%f\n", start_x, start_y);
    warp(start_x, start_y, M_PI / 2);

    sub = boost::shared_ptr<ros::Subscriber>(new ros::Subscriber());
    *sub = nh.subscribe("/make_a_play", 100, &MyPlayer::move, this);

    setTeamName(argin_team);

    printReport();
  }

  void warp(double x, double y, double alfa)
  {

    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setOrigin(tf::Vector3(x, y, 0.0));

    tf::Quaternion q;
    q.setRPY(0, 0, alfa);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "nsilva"));
    ROS_INFO("%s: Warping to x=%f, y=%f, alpha=%f", name.c_str(), x, y, alfa);
  }

  void move(const rws2018_msgs::MakeAPlay::ConstPtr &msg)
  {
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double a = 0;

    // AI PART
    double displacement = 6; // computed using AI
    double delta_alpha = M_PI / 2;

    // CONSTRAINSTS PART
    double dist_max = msg->dog;
    double dist_with_constrainsts;

    if (displacement > dist_max)
      displacement = dist_max;

    double delta_alpha_max = M_PI / 3;
    if (fabs(delta_alpha) > fabs(delta_alpha_max))
      delta_alpha = delta_alpha_max * delta_alpha / fabs(delta_alpha);

    tf::Transform my_move_transform;
    my_move_transform.setOrigin(tf::Vector3(displacement, 0.0, 0.0));

    // Update position
    /*int limit = 5;
        if (x > limit || y > limit)
          vf = -1;
        else if (x < -limit || y < -limit)
          vf = 1;
        x += vf;
        y += vf;*/

    transform.setOrigin(tf::Vector3(x, y, 0.0));

    tf::Quaternion q;
    q.setRPY(0, 0, delta_alpha);
    my_move_transform.setRotation(q);

    transform = transform * my_move_transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "nsilva"));
  }

  void printReport()
  {
    ROS_INFO("My name is %s, my team is %s and I am gonna catch you all >:)", name.c_str(), getTeamName().c_str());
  }
};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nsilva");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100000);

  // Creating an instance of class Player
  rws_nsilva::MyPlayer my_player("nsilva", "blue");

  /*while (ros::ok())
  {
    my_player.move();

    ros::spinOnce();
    loop_rate.sleep();
  }*/

  ros::spin();
}