#include "isearch.h"

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}


SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    bool pathfound = false;
    Node Start;
    Start.parent = nullptr;
    Start.i = map.GetMapStartI(); Start.j = map.GetMapStartJ();
    Start.H = computeHFromCellToCell(Start.i, Start.j, map.GetMapGoalI(), map.GetMapGoalJ(), options);
    Start.F = Start.H * hweight;
    Start.g = 0;
    Open.push(Start);

    while (!Open.empty()) {
        Start = Open.top();
        Open.pop();
        Close.push_back(Start);

        if (Start.i == map.GetMapGoalI() && Start.j == map.GetMapGoalJ()) {
            pathfound = true;
            break;
        }
        std::list<Node> neighbours = findSuccessors(Start, map, options);

        for (Node a : neighbours) {
            a.parent = &Start;
            a.H = computeHFromCellToCell(a.i, a.j, map.GetMapGoalI(), map.GetMapGoalJ(), options);
            Open.push(a);
        }
    }

    if (pathfound) {
            sresult.pathfound = true;
            makePrimaryPath(Start);
            sresult.pathlength = Start.g;
    }

    end = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1000000000;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    std::list<Node> neighbours;
    Node next;
    for (int i = -1; i <= +1; i++)
            for (int j = -1; j <= +1; j++)
                if (map.CellOnGrid(curNode.i + i, curNode.j + j) && (map.CellIsTraversable(curNode.i + i, curNode.j + j))) {
                    if (i != 0 && j != 0) {
                        if (!options.allowdiagonal) {
                            continue;
                        } else if (!options.cutcorners) {
                            if (map.CellIsObstacle(curNode.i, curNode.j + j) || map.CellIsObstacle(curNode.i + i, curNode.j))
                                continue;
                        } else if (!options.allowsqueeze) {
                            if (map.CellIsObstacle(curNode.i, curNode.j + j) && map.CellIsObstacle(curNode.i + i, curNode.j))
                                continue;
                        }
                    }

                    next.i = curNode.i + i;
                    next.j = curNode.j + j;

                    if (std::find(Close.begin(), Close.end(), next) == Close.end()) {
                        if (i * j == 0)
                            next.g = curNode.g + 1;
                        else
                            next.g = curNode.g + sqrt(2);
                        neighbours.push_front(next);
                    }
                }

    return neighbours;
}

void ISearch::makePrimaryPath(Node Start)
{
    Node a = Start;
    while(a.parent) {
        lppath.push_front(a);
        a = *a.parent;
    }
    lppath.push_front(a);
}

void ISearch::makeSecondaryPath()
{
    //need to implement
}
