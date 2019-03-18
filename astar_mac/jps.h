#ifndef JPS_H
#define JPS_H
#include "astar.h"


class JPS:public Astar
{

public:
    JPS(float hweight, bool breakingties):Astar(hweight, breakingties){}
    ~JPS();

private:
    std::list<Node> findSuccessors(Node &curNode, const Map &map, const EnvironmentOptions &options);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath();
};

#endif // JPS_H
