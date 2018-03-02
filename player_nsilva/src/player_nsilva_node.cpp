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

using namespace std;

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

  double x, y;
  double vf = 0.0001;

  MyPlayer(string argin_name, string argin_team) : Player(argin_name)
  {
    red_team = boost::shared_ptr<Team>(new Team("red"));
    green_team = boost::shared_ptr<Team>(new Team("green"));
    blue_team = boost::shared_ptr<Team>(new Team("blue"));

    x = y = 0.0;

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

    setTeamName(argin_team);

    printReport();
  }

  void move()
  {
    // Send tf
    tf::Transform transform;

    // Update position
    if (x > 5)
      vf = -0.0001;
    else if (x < -5)
      vf = 0.0001;
    x += (((double)rand() / (RAND_MAX))) * vf;
    // y += (((double)rand() / (RAND_MAX)) - 0.5) * 0.005;

    transform.setOrigin(tf::Vector3(x, -x, 0.0));

    tf::Quaternion q;
    q.setRPY(0, 0, M_PI);
    transform.setRotation(q);

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

  while (ros::ok())
  {
    my_player.move();

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}