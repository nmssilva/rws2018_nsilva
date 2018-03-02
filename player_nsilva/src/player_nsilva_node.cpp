#include <math.h>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// RWS includes
#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>

// defines
#define DEFAULT_TIME 0.05

using namespace std;
using namespace ros;
using namespace tf;

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

  TransformBroadcaster br;
  Transform transform;

  boost::shared_ptr<Subscriber> sub;
  NodeHandle nh;
  Publisher vis_pub;
  TransformListener listener;

  float x, y;

  MyPlayer(string argin_name, string argin_team) : Player(argin_name)
  {
    red_team = boost::shared_ptr<Team>(new Team("red"));
    green_team = boost::shared_ptr<Team>(new Team("green"));
    blue_team = boost::shared_ptr<Team>(new Team("blue"));

    vis_pub = nh.advertise<visualization_msgs::Marker>("/bocas", 0);

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

    sub = boost::shared_ptr<Subscriber>(new Subscriber());
    *sub = nh.subscribe("/make_a_play", 100, &MyPlayer::move, this);

    setTeamName(argin_team);

    printReport();
  }

  void warp(double x, double y, double alfa)
  {
    transform.setOrigin(Vector3(x, y, 0.0));
    transform.setOrigin(Vector3(x, y, 0.0));

    Quaternion q;
    q.setRPY(0, 0, alfa);
    transform.setRotation(q);

    br.sendTransform(StampedTransform(transform, Time::now(), "world", "nsilva"));
    ROS_INFO("%s: Warping to x=%f, y=%f, alpha=%f", name.c_str(), x, y, alfa);
  }

  void move(const rws2018_msgs::MakeAPlay::ConstPtr &msg)
  {
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double a = 0;

    // AI PART
    double displacement = 6;  // computed using AI
    double getaway = 0;
    string closestPlayer = getClosestPlayer(msg);

    ROS_INFO("CLOSEST %s", closestPlayer);
    if (closestPlayer[closestPlayer.length() - 1] == '@')
    {
      closestPlayer = closestPlayer.substr(0, closestPlayer.size() - 1);
      ROS_INFO("Running away from %s", closestPlayer);
      getaway = M_PI;
    }
    else
    {
      getaway = 0;
      ROS_INFO("Chasing %s", closestPlayer);
    }
    double delta_alpha = getAngleToPLayer(closestPlayer) + getaway;

    string marker_string = "Distance to " + closestPlayer + ": ";

    double distanceToPlayer = getDistancetoPlayer(closestPlayer);

    char distance_string[32];
    sprintf(distance_string, "%f", distanceToPlayer);

    marker_string = marker_string + distance_string;

    showMarker(marker_string);
    if (isnan(delta_alpha))
      delta_alpha = 0;

    // CONSTRAINSTS PART
    double dist_max = msg->turtle;
    double dist_with_constrainsts;

    if (displacement > dist_max)
      displacement = dist_max;

    double delta_alpha_max = M_PI / 30;
    if (fabs(delta_alpha) > fabs(delta_alpha_max))
      delta_alpha = delta_alpha_max * delta_alpha / fabs(delta_alpha);

    Transform my_move_transform;
    my_move_transform.setOrigin(Vector3(displacement, 0.0, 0.0));

    // Update position
    transform.setOrigin(Vector3(x, y, 0.0));

    Quaternion q;
    q.setRPY(0, 0, delta_alpha);
    my_move_transform.setRotation(q);

    transform = transform * my_move_transform;

    br.sendTransform(StampedTransform(transform, Time::now(), "world", "nsilva"));
  }

  double getAngleToPLayer(string other_player, double time_to_wait = DEFAULT_TIME)
  {
    StampedTransform t;  // The transform object
    Time now = Time(0);  // get the latest transform received

    try
    {
      listener.waitForTransform("nsilva", other_player, now, Duration(time_to_wait));
      listener.lookupTransform("nsilva", other_player, now, t);
    }
    catch (TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return NAN;
    }

    return atan2(t.getOrigin().y(), t.getOrigin().x());
  }

  double getDistancetoPlayer(string other_player, double time_to_wait = DEFAULT_TIME)
  {
    StampedTransform t;  // The transform object
    Time now = Time(0);  // get the latest transform received

    try
    {
      listener.waitForTransform("nsilva", other_player, now, Duration(time_to_wait));
      listener.lookupTransform("nsilva", other_player, now, t);
    }
    catch (TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return NAN;
    }

    return sqrt(pow(t.getOrigin().x(), 2) + pow(t.getOrigin().y(), 2));
  }

  string getClosestPlayer(const rws2018_msgs::MakeAPlay::ConstPtr &msg)
  {
    double distance = 99;
    string player;

    for (int i = 0; i < msg->red_alive.size(); i++)
    {
      if (distance > getDistancetoPlayer(msg->red_alive[i]))
      {
        distance = getDistancetoPlayer(msg->red_alive[i]);
        player = msg->red_alive[i];
      }
    }

    for (int i = 0; i < msg->green_alive.size(); i++)
    {
      if (distance > getDistancetoPlayer(msg->green_alive[i]))
      {
        distance = getDistancetoPlayer(msg->green_alive[i]);
        player = msg->green_alive[i] + "@";
      }
    }

    return player;
  }

  void showMarker(string text)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "nsilva";
    marker.header.stamp = Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = -0.3;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.text = text;
    marker.scale.z = 0.4;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;

    vis_pub.publish(marker);
  }

  void printReport()
  {
    ROS_INFO("My name is %s, my team is %s and I am gonna catch you all >:)", name.c_str(), getTeamName().c_str());
  }
};
}

int main(int argc, char **argv)
{
  init(argc, argv, "nsilva");
  NodeHandle nh;
  Rate loop_rate(10);

  // Creating an instance of class Player
  rws_nsilva::MyPlayer my_player("nsilva", "blue");

  spin();
}