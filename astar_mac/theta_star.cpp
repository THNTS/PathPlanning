#include "theta_star.h"

theta_star::~theta_star(){}

void theta_star::update_parent(Node* &cur, Node* &parent, const Map &map, const EnvironmentOptions &options) {
    if (line_of_sight(cur, parent, map, options)) {
        cur->g = parent->parent->g + sqrt(pow(parent->parent->i - cur->i, 2) + pow(parent->parent->j - cur->j, 2));
        cur->parent = parent->parent;
    }
    return;
}

bool theta_star::line_of_sight(Node *&cur, Node *&parent, const Map &map, const EnvironmentOptions &options)
{
    int x0 = cur->i;
    int y0 = cur->j;
    int x1 = parent->i;
    int y1 = parent->j;
    int f = 0;

    int dy = y1 - y0;
    int dx = x1 - x0;

    int sy;
    if (dy < 0) {
        dy *= - 1;
        sy  = -1;
    } else {
        sy = 1;
    }
    int sx;
    if (dx < 0) {
        dx *= - 1;
        sx  = -1;
    } else {
        sx = 1;
    }

    if (options.cutcorners) {
        if (dx == dy) {
            if (options.allowsqueeze) {
                if (map.CellIsObstacle(x0 + sx, y0 + sy)) {
                    return false;
                } else if (map.CellIsObstacle(x0 + sx, y0 + sy) || (map.CellIsObstacle(x0, y0 + sy) && map.CellIsObstacle(x0 + sx, y0))) {
                    return false;
                }
            }
        } else if (dx >= dy) {
            while (x0 != x1) {
                f += dy;
                if (f >= dx) {
                    if (map.CellIsObstacle(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)))
                        return false;
                    y0 += sy;
                    f -= dx;
                }
                if (f != 0 && map.CellIsObstacle(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)))
                    return false;
                if (dy == 0 && map.CellIsObstacle(x0 + ((sx - 1)/2), y0) && map.CellIsObstacle(x0 + ((sx - 1)/2), y0 - 1))
                    return false;
                x0 += sx;
            }
        } else {
            while (y0 != y1) {
                f += dx;
                if (f >= dy) {
                    if (map.CellIsObstacle(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)))
                        return false;
                    x0 += sx;
                    f -= dy;
                }
                if (f != 0 && map.CellIsObstacle(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)))
                    return false;
                if (dx == 0 && map.CellIsObstacle(x0, y0 + ((sy - 1)/2)) && map.CellIsObstacle(x0 - 1, y0 + ((sy - 1)/2)))
                    return false;
                y0 += sy;
            }
        }
    } else {
        if (dx >= dy) {
            while (x0 != x1) {
                f += dy;
                if (f >= dx) {
                    if (map.CellIsObstacle(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)))
                        return false;
                    y0 += sy;
                    f -= dx;
                }
                if (f == 0 && map.CellIsObstacle(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)))
                    return false;
                if (dy == 0 && map.CellIsObstacle(x0 + ((sx - 1)/2), y0) && map.CellIsObstacle(x0 + ((sx - 1)/2), y0 - 1))
                    return false;
                x0 += sx;
            }
        } else {
            while (y0 != y1) {
                f += dx;
                if (f >= dy) {
                    if (map.CellIsObstacle(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)))
                        return false;
                    x0 += sx;
                    f -= dy;
                }
                if (f == 0 && map.CellIsObstacle(x0 + ((sx - 1)/2), y0 + ((sy - 1)/2)))
                    return false;
                if (dx == 0 && map.CellIsObstacle(x0, y0 + ((sy - 1)/2)) && map.CellIsObstacle(x0 - 1, y0 + ((sy - 1)/2)))
                    return false;
                y0 += sy;
            }
        }
    }

    return true;
}

void theta_star::makePrimaryPath(Node curNode)
{
    Node current = curNode;
    while(current.parent) {
        hppath.push_front(current);
        current = *current.parent;
    }
    hppath.push_front(current);
}
void theta_star::makeSecondaryPath()
{
//imlement
}



