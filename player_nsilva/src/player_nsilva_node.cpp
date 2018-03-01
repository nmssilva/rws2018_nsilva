#include <iostream>
#include <vector>
#include <math.h>
#include <ctime>

// Boost includes
#include <boost/shared_ptr.hpp>

//ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//RWS includes
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
      cout << "[ERROR]: For player " + this->name + " team name " + team + " is invalid." << endl;
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
  tf::TransformBroadcaster br;

  double x, y;

  MyPlayer(string argin_name, string argin_team) : Player(argin_name)
  {
    red_team = boost::shared_ptr<Team>(new Team("red"));
    green_team = boost::shared_ptr<Team>(new Team("green"));
    blue_team = boost::shared_ptr<Team>(new Team("blue"));

    x = y = 0.0;

    setTeamName(argin_team);

    printReport();
  }

  void move()
  {
    // Send tf
    tf::Transform transform;

    //Update position
    x += (((double)rand() / (RAND_MAX)) - 0.5) * 0.1;
    y += (((double)rand() / (RAND_MAX)) - 0.5) * 0.1;

    transform.setOrigin(tf::Vector3(x, y, 0.0));

    tf::Quaternion q;
    q.setRPY(0, 0, M_PI);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "nsilva"));
  }

  void printReport()
  {
    cout << "My name is " << name << ",my team is " << getTeamName() << " and I will catch you all >:)" << endl;
  }
};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nsilva");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

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