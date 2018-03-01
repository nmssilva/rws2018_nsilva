#include <stdio.h>
#include <iostream>
using namespace std;

class Player
{
public:
  Player(std::string name)
  {
    this->name = name;
  }

  std::string name;

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

  void setTeamName(std::string team)
  {
    if (team == "red" || team == "green" || team == "blue")
    {
      team_name = team;
    }
    else
      cout << "[ERROR]: For player " + this->name + " team name " + team + " is invalid." << endl;
  }

  std::string getTeamName(void)
  {
    return this->team_name;
  }

private:
  std::string team_name;
};

class MyPlayer : public Player
{
public:
  MyPlayer(std::string argin_name, std::string argin_team) : Player(name)
  {
    setTeamName(argin_team);
  }
};

int main()
{
  // Creating an instance of class Player
  Player player("nsilva");
  player.setTeamName("red");

  std::cout << "Created an instance of class player with public name " << player.name << " in team "
            << player.getTeamName() << std::endl;

  MyPlayer my_player("nsilva", "red");

  std::cout << "my_player team_name = " << my_player.getTeamName() << std::endl;
}