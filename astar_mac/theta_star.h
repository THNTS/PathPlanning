#ifndef THETA_STAR_H
#define THETA_STAR_H

#include <astar.h>

class theta_star: public Astar
{
public:
    ~theta_star(void);
    theta_star(double hweight, bool bt) : Astar(hweight, bt){}

private:
    void update_parent(Node* &cur, Node* &parent, const Map &map, const EnvironmentOptions &options);

    bool line_of_sight(Node* &cur, Node* &parent, const Map &map, const EnvironmentOptions &options);

    void makePrimaryPath(Node curNode);
    void makeSecondaryPath();
};

#endif // THETA_STAR_H
