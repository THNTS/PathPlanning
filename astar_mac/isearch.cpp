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
    Close2.resize(map.getMapHeight());
    bool pathfound = false;
    Node Start;
    Start.parent = nullptr;
    Start.i = map.GetMapStartI(); Start.j = map.GetMapStartJ();
    Start.H = computeHFromCellToCell(Start.i, Start.j, map.GetMapGoalI(), map.GetMapGoalJ(), options);
    Start.F = Start.H * hweight;
    Start.g = 0;
    Open.emplace(Start);
    Close2[Start.i].push_back(Start.j);

    while (!Open.empty()) {
        Start = Open.top();

        Open.pop();
        Close.push_back(Start);
        std::cout << Close.size() << std::endl;


        if (Start.i == map.GetMapGoalI() && Start.j == map.GetMapGoalJ()) {
            pathfound = true;
            break;
        }


        std::list<Node> neighbours = findSuccessors(Start, map, options);
        std::list<Node>::iterator it = neighbours.begin();
        Node * parent = new Node(Close[Close.size() - 1]);
        std::cout << Close[Close.size() - 1].i << " " << Close[Close.size() - 1].j << std::endl;
        while (it != neighbours.end()) {
                it->parent = parent;
                it->H = computeHFromCellToCell(it->i, it->j, map.GetMapGoalI(), map.GetMapGoalJ(), options);
                it->F = it->g + hweight * it->H;
                resetParent(*it, *parent, map, options);
                Open.emplace(*it);
                it++;
        }
    }

    sresult.pathfound = false;

    if (pathfound) {
            sresult.pathfound = true;
            makePrimaryPath(Start);
            sresult.pathlength = Start.g;
    }

    sresult.nodescreated = Close.size() + Open.size();
    sresult.numberofsteps = Close.size();


    end = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1000000000;
    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    return sresult;
}

std::list<Node> ISearch::findSuccessors(Node current, const Map &map, const EnvironmentOptions &options)
{
    std::list<Node> neighbours;
    Node next;
    for (int i = -1; i <= +1; i++)
            for (int j = -1; j <= +1; j++)
                if (map.CellOnGrid(current.i + i, current.j + j) && (map.CellIsTraversable(current.i + i, current.j + j))) {
                    if (i != 0 && j != 0) {
                        if (!options.allowdiagonal) {
                            continue;
                        } else if (!options.cutcorners) {
                            if (map.CellIsObstacle(current.i, current.j + j) || map.CellIsObstacle(current.i + i, current.j))
                                continue;
                        } else if (!options.allowsqueeze) {
                            if (map.CellIsObstacle(current.i, current.j + j) && map.CellIsObstacle(current.i + i, current.j))
                                continue;
                        }
                    }

                    next.i = current.i + i;
                    next.j = current.j + j;
                    if (std::find(Close2[next.i].begin(), Close2[next.i].end(), next.j) == Close2[next.i].end()) {
                        if (i * j == 0)
                            next.g = current.g + 1;
                        else
                            next.g = current.g + sqrt(2);
                        neighbours.push_front(next);
                    }
                }

    return neighbours;
}

void ISearch::makePrimaryPath(Node Start)
{
    Node a = Start;
    while (a.parent) {
        lppath.push_front(a);
        a = *a.parent;
    }
    lppath.push_front(a);
}

void ISearch::makeSecondaryPath()
{
    auto PathPntr = lppath.begin();
    while (PathPntr != hppath.end()) {
        auto NextPntr = PathPntr;
        int x = PathPntr->i;
        int y = PathPntr->j;
        NextPntr++; PathPntr++;
        int dx = NextPntr->i - PathPntr->i;
        int dy = NextPntr->j - PathPntr->j;

        if ((x - NextPntr->i) != dx || (y - NextPntr->j) != dy) {
            hppath.push_back(*(NextPntr));
        } else {
            PathPntr--;
        }
    }
}
