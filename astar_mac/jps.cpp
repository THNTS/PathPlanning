#include "jps.h"

JPS::~JPS()
{

}

bool jump(Node &current, int dx, int dy, const Map &map, const EnvironmentOptions &options) {
    int x = current.i;
    int y = current.j;

    while(map.CellOnGrid(x, y) && map.CellIsTraversable(x, y)) {

            if(x == map.GetMapGoalI() && y == map.GetMapGoalJ())
                return true;

            if(options.cutcorners) {

                if(dy != 0) {
                    if (map.CellOnGrid(x, y + dy)) {
                        if(map.CellOnGrid(x + 1, y))
                            if(map.CellIsTraversable(x + 1, y + dy) && map.CellIsObstacle(x + 1, y))
                                return true;
                        if(map.CellOnGrid(x - 1,y))
                            if(map.CellIsTraversable(x - 1, y + dy) && map.CellIsObstacle(x - 1, y))
                                return true;
                    }
                }
                if(dx != 0 && map.CellOnGrid(x + dx, y)) {
                    if(map.CellOnGrid(x, y + 1))
                        if(map.CellIsTraversable(x + dx, y + 1) && map.CellIsObstacle(x, y + 1))
                            return true;
                    if(map.CellOnGrid(x, y - 1))
                        if(map.CellIsTraversable(x + dx, y - 1) && map.CellIsObstacle(x, y - 1))
                            return true;
                }
            }
            else {

                if(dy != 0) {
                    if (map.CellOnGrid(x, y - dy)) {
                        if(map.CellOnGrid(x + 1, y))
                            if(map.CellIsTraversable(x + 1, y) && map.CellIsObstacle(x + 1, y - dy))
                                return true;
                        if(map.CellOnGrid(x - 1,y))
                            if(map.CellIsTraversable(x - 1, y) && map.CellIsObstacle(x - 1, y - dy))
                                return true;
                    }
                }

                if(dx != 0) {
                    if (map.CellOnGrid(x - dx, y)) {
                        if(map.CellOnGrid(x, y + 1))
                            if(map.CellIsTraversable(x, y + 1) && map.CellIsObstacle(x - dx, y + 1))
                                return true;
                        if(map.CellOnGrid(x, y - 1))
                            if(map.CellIsTraversable(x, y - 1) && map.CellIsObstacle(x - dx, y - 1))
                                return true;
                    }
                }
            }
            x += dx;
            y += dy;
            current.i = dx;
            current.j = dy;
            if (dx * dy == 0) {
                current.g += 1;
            } else {
                current.g += sqrt(2);
            }
        }
    return false;
}

std::list<Node> JPS::findSuccessors(Node &current, const Map &map, const EnvironmentOptions &options) {
    std::list<Node> successors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (jump(current, dx, dy, map, options)) {
                successors.push_back(current);
            }
        }
    }
    return successors;
}

void JPS::makePrimaryPath(Node current) {
    while (true) {
        hppath.push_front(current);
        if (current.parent != nullptr) {
            current = *current.parent;
        } else {
            break;
        }
    }
}

void JPS::makeSecondaryPath() {
    auto PathPntr = hppath.begin();

    while (PathPntr != hppath.end()) {
        auto NextPntr = PathPntr;
        NextPntr++;
        if (NextPntr == hppath.end()) {
            lppath.push_back(*PathPntr);
        } else {
            int dx = NextPntr->i - PathPntr->i;
            int dy = NextPntr->j - PathPntr->j;
            if (dx != 0) {
                dx = dx / abs(dx);
            }
            if (dy != 0) {
                dy = dy / abs(dy);
            }

            int x = PathPntr->i;
            int y = PathPntr->j;

            while (x != NextPntr->i or y != NextPntr->j) {
                Node a;
                a.i = x; a.j = y;
                a.g = 0; a.F = 0; a.H = 0;
                a.parent = nullptr;
                lppath.push_back(a);
                x += dx;
                y += dy;
            }
        }
        PathPntr++;
    }
}
