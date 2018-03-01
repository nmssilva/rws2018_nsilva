#include <iostream>                                                                                     

class Player
{
    public:

    Player(std::string name)
    {
        this->name = name;
    }


    std::string name;

    private:

};

int main()
{

    std::string player_name = "nsilva";
    //Creating an instance of class Player
    Player player(player_name);

    std::cout << "Created an instance of class player with public name " << player.name << std::endl;
}