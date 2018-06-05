/* mouse.cpp

 Sean Cork, Ruiqi Li
 May 18, 2018
 Project 8: Motion Planning

 */

#include <map>
#include <set>
#include <queue>
#include "geom.h"
#include <vector>
#include <math.h>
#include <limits>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

using namespace std;

// color schemes
GLfloat red[3] = {1.0, 0.0, 0.0};
GLfloat light_blue[3] = {0.8, 0.9, 1.0};
GLfloat dark_blue[3] = {0, 0.2, 0.4};
GLfloat light_pink[3] = {1, 0.5, 0.6};
GLfloat light_yellow[3] = {1.0, 1.0, .8};
GLfloat blue[3] = {0.0, 0.0, 1.0};
GLfloat black[3] = {0.0, 0.0, 0.0};
GLfloat white[3] = {1.0, 1.0, 1.0};
GLfloat yellow[3] = {1.0, 1.0, 0.0};
GLint fillmode = 0;


// global variables
vector<vector<point2D> > obstacle_set;
vector<point2D> new_obstacle;
int poly_init_mode = 0;
const int WINDOWSIZE = 750;
double mouse_x=-10, mouse_y=-10;
int create_obstable_mode = 0;
int change_start_pt_mode = 0;
int change_finish_pt_mode = 0;
int start_x = 0;
int start_y = 0;
int finish_x = WINDOWSIZE;
int finish_y = WINDOWSIZE;
point2D robot_corner_1;
point2D robot_corner_2;
float eps = 0.0001;
vector<point2D> robot_verticies;

vector<point2D> path_for_robot;
vector<point3D> path3D;

int step_size = 5;
int next_step_index = 0;
int robot_size_x = 40;
int robot_size_y = 10;
float sleep_period = 100000;



// functions
void polygon_create(int, int, int, int);
void initialize_obstacles();
void print_obstacles();
void draw_polygon(vector<point2D> polygon, GLfloat* color);
void draw_obstacles(GLfloat* color);
int signed_area2D(point2D a, point2D b, point2D c);
int left_strictly(point2D a, point2D b, point2D c);
int right_strictly(point2D a, point2D b, point2D c);
bool simple(vector<point2D> polygon);
bool strict_intersect(point2D a, point2D b, point2D c, point2D d);
bool insidePolygon(double x, double y, vector<point2D> poly);
bool non_overlap(vector<point2D> polygon);
bool simpleness_intersect(point2D a, point2D b, point2D c, point2D d);
bool valid_new_obstacle(vector<point2D> obstacle);
bool valid_new_point(int x, int y, vector<point2D>);
double find_distance(point2D p1, point2D p2);
void draw_a_star_path(GLfloat* color);
int heuristic_cost(point2D start, point2D goal);
void a_star(point2D start, point2D goal);
void draw_robot(point2D corner1, point2D corner2, GLfloat *color);
void display();
void pin_point(int x, int y, GLfloat* color);
void keypress(unsigned char key, int x, int y);



// creat the an n-sided polygon centered at (x,y)
void polygon_create(int x, int y, int rad, int n) {

    //clear the vector, in case something was there
    vector<point2D> poly;

    double step = 2 * M_PI / n;
    point2D p;

    for (int i=0; i<n; i++) {
        p.x = x + rad * cos (i * step);
        p.y = y + rad * sin (i * step);
        poly.push_back (p);
    }

    obstacle_set.push_back(poly);
}



// presetting of the scene
void initialize_obstacles(){
    obstacle_set.clear();
    polygon_create(WINDOWSIZE/5, WINDOWSIZE/5, 50, 7);
    polygon_create(WINDOWSIZE/2, WINDOWSIZE/3, 80, 3);
    polygon_create(WINDOWSIZE/2, WINDOWSIZE-100, 40, 4);
    polygon_create(WINDOWSIZE-200, 2*WINDOWSIZE/3, 100, 10);
    polygon_create(200, 2*WINDOWSIZE/3, 100, 5);
}



// print the vertices of each obstacle
void print_obstacles(){
    for (int i = 0; i < obstacle_set.size(); i++){
        cout << "Obstacle " << i << endl;
        for (int j = 0; j < obstacle_set[i].size(); j++){
            cout << obstacle_set[i][j].x << " " << obstacle_set[i][j].y << endl;
        }
        cout << endl;
    }
}



// draw a given polygon
void draw_polygon(vector<point2D> polygon, GLfloat* color){
    // set color
    glColor3fv(color);
    glLineWidth(3);
    for (int i = 0; i < polygon.size(); i++){
        glBegin(GL_LINES);
        glVertex2f(polygon[i].x, polygon[i].y);
        glVertex2f(polygon[(i+1)%polygon.size()].x, polygon[(i+1)%polygon.size()].y);
        glEnd();
    }
    glLineWidth(1);
}



// draw all the polygons in the obstacle set
void draw_obstacles(GLfloat* color){
    for (int i = 0; i < obstacle_set.size(); i++) draw_polygon(obstacle_set[i], color);
}



// draw a point centered at the given x- and y-coordinates
void pin_point(int x, int y, GLfloat* color){
    glColor3fv(color);
    int precision = 100;
    double r = 4;
    double theta = 0;
    glBegin(GL_POLYGON);
    for(int i = 0; i < precision; i++){
        theta = i * 2 * M_PI/precision;
        glVertex2f(x + r*cos(theta), y + r*sin(theta));
    }
    glEnd();
}



/* Helper function for determining which side of ab is c on */
int signed_area2D(point2D a, point2D b, point2D c) {
    // from a to b
    return (b.x-a.x) * (c.y-a.y) - (b.y-a.y) * (c.x-a.x);
}



/* return 1 if c is strictly left of ab; 0 otherwise */
int left_strictly(point2D a, point2D b, point2D c) {
    return (signed_area2D(a, b, c) > eps);
}



/* return 1 if c is strictly right of ab; 0 otherwise */
int right_strictly(point2D a, point2D b, point2D c) {
    return (signed_area2D(a, b, c) < -eps);
}



int collinear(point2D p, point2D q, point2D r) {
    if (p.x * (q.y - r.y) + q.x * (r.y - p.y) + r.x * (p.y - q.y) != 0) return 0;
    return 1;
}



/* Checking if two line segments intersect in the context of checking simpleness of polygon
 * It is different from function strictly_intersect in that it only counts
 * p1p2 makes up segment 1, p3p4 makes up segment 2 */
bool simpleness_intersect(point2D a, point2D b, point2D c, point2D d){
    if (a.x == c.x && a.y == c.y) return false;
    if (a.x == d.x && a.y == d.y) return false;
    if (b.x == c.x && b.y == c.y) return false;
    if (b.x == d.x && b.y == d.y) return false;


    // if c is on the same side of ab as d, return false
    if (left_strictly(a,b,c) && left_strictly(a,b,d)) return false;
    if (right_strictly(a,b,c) && right_strictly(a,b,d)) return false;

    if (left_strictly(c,d,a) && left_strictly(c,d,b)) return false;
    if (right_strictly(c,d,a) && right_strictly(c,d,b)) return false;


    // degenerate cases
    // if a intersects with cd
    if (fabs((c.y + (a.x - c.x) * (d.y - c.y) / (d.x - c.x)) - a.y) < eps){
        if ((c.y + (b.x - c.x) * (d.y - c.y) / (d.x - c.x)) > b.y) return false;
    }
    if (fabs((c.y + (b.x - c.x) * (d.y - c.y) / (d.x - c.x)) - b.y) < eps){
        if ((c.y + (a.x - c.x) * (d.y - c.y) / (d.x - c.x)) > a.y) return false;
    }

    return true;

}



bool Between(point2D a, point2D b, point2D c){
    if(!collinear(a, b, c)) return false;

    if(a.x != b.x){
        return((a.x <= c.x) && (c.x <= b.x)) ||
        ((a.x >= c.x) && (c.x >= b.x));
    }

    return ((a.y <= c.y) && (c.y <= b.y)) || ((a.y >= c.y) && (c.y >= b.y));
}



/* **************************************** */
/* Taken from the book dont really know what it does */
/* **************************************** */
void Assigndi(point2DD *p, point2D a){
    p->x = a.x;
    p->y = a.y;
}



/* **************************************** */
/* This functions returns whether the point of intersection is collinear
 with any ot the segments in the polygon
 */
char ParallelInt(point2D a, point2D b, point2D c, point2D d, point2DD *p){
    if (!collinear(a, b, c)){
        return '0';
    }
    if(Between(a, b, c)){
        Assigndi(p, c);
        return 'e';
    }
    if(Between(a, b, d)){
        Assigndi(p, d);
        return 'e';
    }
    if(Between(c, d, a)){
        Assigndi(p, a);
        return 'e';
    }
    if(Between(c, d, b)){
        Assigndi(p, b);
        return 'e';
    }
    return '0';
}



/* **************************************** */
/*This function checks to see if there are any intersections
 between segments and returns the type of intersection and the points
 of intersection */
/* **************************************** */
char seg_Intersect(point2D a, point2D b, point2D c, point2D d, point2DD *p){
    double s, t;
    double num, denom;
    char code = '?';

    denom = (a.x * (double)(d.y - c.y)) + (b.x * (double)(c.y - d.y)) +
    (d.x * (double)(b.y - a.y)) + (c.x * (double)(a.y - b.y));


    if(denom == 0.0){
        return ParallelInt(a, b, c, d,p);
    }

    num = (a.x * (double)(d.y - c.y)) + (c.x * (double)(a.y - d.y)) + (d.x * (double)(c.y-a.y));
    if((num == 0.0) || (num == denom)){
        code = 'v';
    }
    s = num/denom;

    num = -((a.x * (double)(c.y - b.y)) + (b.x * (double)(a.y - c.y)) + (c.x * (double)(b.y-a.y)));
    if((num == 0.0) || (num == denom)){
        code = 'v';
    }
    t = num/denom;


    if((0.0 < s) && (s <1.0) && (0.0 < t) && (t <1.0)){
        code = 'l';
    }
    else if((0.0 > s) || (s > 1.0) || (0.0 > t) || (t>1.0)){
        code = '0';
    }
    return code;

}



// check if the given point is inside any of the polygon in the given set
bool insidePolygons(double x, double y, vector<vector<point2D> > polygons){
    for(int i = 0; i< polygons.size(); i++){
        if(insidePolygon(x, y, polygons[i])) return true;
    }
    return false;

}



// check if a point is insie a polygon that is known to be convex
bool insideConvexPolygon(point2D pt, vector<point2D> poly){
    int reference = left_strictly(pt, poly[0], poly[1]);
    for (int i = 1; i < poly.size(); i++) if (left_strictly(pt, poly[i], poly[(i+1)%poly.size()]) != reference) return false;
    return true;
}



/* Check if point given by coordinate (x, y) is inside the polygon */
bool insidePolygon(double x, double y, vector<point2D> poly){

    point2D sp1, sp2, origin, rightmost, leftmost;
    origin.x = 0;
    origin.y = 0;
    rightmost.x = WINDOWSIZE;
    rightmost.y = 0;
    leftmost.x = 0;
    leftmost.y = 0;

    int leftIntersectCt = 0;
    int rightIntersectCt = 0;

    // by checking how many times the horizontal line passing through
    // point (x, y) intersects with the poly
    for (int i = 0; i < poly.size(); i++){

        sp1.x = poly[i].x - x;
        sp1.y = poly[i].y - y;
        sp2.x = poly[(i+1)%poly.size()].x - x;
        sp2.y = poly[(i+1)%poly.size()].y - y;

        if (simpleness_intersect(sp1, sp2, origin, leftmost)) leftIntersectCt++;
        if (simpleness_intersect(sp1, sp2, origin, rightmost)) rightIntersectCt++;
    }

    return (leftIntersectCt%2 == 1) || (rightIntersectCt%2 == 1);
}



// check if a polygon is simple
bool simple(vector<point2D> polygon){
    for (int i = 0; i < polygon.size(); i++){
        for (int j = i+2; j < polygon.size(); j++){
            if (simpleness_intersect(polygon[i], polygon[(i+1)%polygon.size()], polygon[j], polygon[(j+1)%polygon.size()])){
                cout << "NOT A SIMPLE POLYGON" << endl;
                cout << "Please press s and draw another one" << endl << endl << endl;
                return false;
            }
        }
    }
    return true;
}



// check if two points touch each other in any way
bool strict_intersect(point2D a, point2D b, point2D c, point2D d){
    if (a.x == c.x && a.y == c.y) return false;
    if (a.x == d.x && a.y == d.y) return false;
    if (b.x == c.x && b.y == c.y) return false;
    if (b.x == d.x && b.y == d.y) return false;

    // if c is on the same side of ab as d, return false
    if (left_strictly(a,b,c) && left_strictly(a,b,d)) return false;
    if (right_strictly(a,b,c) && right_strictly(a,b,d)) return false;

    if (left_strictly(c,d,a) && left_strictly(c,d,b)) return false;
    if (right_strictly(c,d,a) && right_strictly(c,d,b)) return false;
    return true;
}



// if the given polygon intersect with any of the obstacles
bool non_overlap(vector<point2D> polygon){
    point2DD *p = NULL;
    char type;

    // check if a polygon that is already in the set overlaps with the newly created polygon
    // 1. no edge intersection
    // 2. no inside
    for (int i = 0; i < obstacle_set.size(); i++){
        for (int j = 0; j < polygon.size(); j++){

            // check if the point is in the exisiting obstacle
            for (int k = 0; k < obstacle_set[i].size(); k++){
                type = seg_Intersect(polygon[j], polygon[(j+1)%polygon.size()],
                                     obstacle_set[i][k], obstacle_set[i][(k+1)%obstacle_set[i].size()], p);
                if(type == 'e' || type == 'v' || type == 'l') return false;
            }
        }
    }
    return true;
}



// check if the new clicked point is not inside any obstacle
bool valid_new_point(int x, int y){
    for (int i = 0; i < obstacle_set.size(); i++){
        vector<point2D> obstacle = obstacle_set[i];
        if (insidePolygon(x, y, obstacle)) return false;
    }
    return true;
}



// check if the newly created polygon intersects
// with any obstacle or intersects itselfs
bool valid_new_obstacle(vector<point2D> obstacle){
    if (!simple(obstacle)) return false;
    if (!non_overlap(obstacle)) return false;

    return true;
}



// rotating a around b
point2D point_rotation(point2D a, point2D b, float theta){
    point2D newpoint;
    newpoint.x = cos(theta) * (a.x - b.x) - sin(theta) * (a.y - b.y) + b.x;
    newpoint.y = sin(theta) * (a.x - b.x) + cos(theta) * (a.y - b.y) + b.y;
    return newpoint;
}



// rotating the robot rectangle around the given midpoint by theta (in radian)
vector<point2D> robot_rotation(vector<point2D> robot, float theta, point2D midpoint){
    point2D new_point;
    vector<point2D> new_robot;
    for(int i = 0; i < robot.size(); i++){
        new_point = point_rotation(robot[i], midpoint, theta);
        new_robot.push_back(new_point);
    }
    return new_robot;
}



// find the Euclidean distance between two points
double find_distance(point2D p1, point2D p2){
    return pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2);
}



// draw the path generated using A* algorithm
void draw_a_star_path(GLfloat* color){
    glColor3fv(color);

    // if (shortest_path.size() == 0) return;
    if (path3D.size() == 0) return;

    glLineWidth(3);

    // draw the path of the center point
    for (int i = 0; i < path3D.size()-1; i++){
        glBegin(GL_LINES);
        glVertex2f(path3D[i].pt.x, path3D[i].pt.y);
        glVertex2f(path3D[i+1].pt.x, path3D[i+1].pt.y);
        glEnd();
    }

    // draw the robot rectangle at the next step
    point3D curr = path3D[path3D.size() - next_step_index - 1];
    next_step_index = (next_step_index + 1) % path3D.size();
    vector<point2D> curr_robot;
    point2D newp;
    newp.x = curr.pt.x + robot_size_x;
    newp.y = curr.pt.y + robot_size_y;
    curr_robot.push_back(newp);
    newp.x = curr.pt.x + robot_size_x;
    newp.y = curr.pt.y - robot_size_y;
    curr_robot.push_back(newp);
    newp.x = curr.pt.x - robot_size_x;
    newp.y = curr.pt.y - robot_size_y;
    curr_robot.push_back(newp);
    newp.x = curr.pt.x - robot_size_x;
    newp.y = curr.pt.y + robot_size_y;
    curr_robot.push_back(newp);
    curr_robot = robot_rotation(curr_robot, curr.theta, curr.pt);

    glBegin(GL_POLYGON);
    for(int j = 0; j < 4; j++) glVertex2f(curr_robot[j].x, curr_robot[j].y);
    glEnd();

    usleep(sleep_period);

    glLineWidth(1);
}



// estimated distance between the two points
int heuristic_cost(point3D start, point3D goal){
    int D = step_size;
    int D2 = sqrt(2) * step_size;
    int dx = fabs(start.pt.x - goal.pt.x);
    int dy = fabs(start.pt.y - goal.pt.y);
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
}



// making sure the robot rectangle is inside the rendered window
bool outofwindow(int x, int y){
    return (x > WINDOWSIZE - robot_size_x || x < robot_size_x || y > WINDOWSIZE - robot_size_y || y < robot_size_y);
}



// find the neighbors in all eight directions
// plus the two resulted from rotating by .5 rad, left and right
vector<point3D> construct_neighbors(point3D curr){
    int curr_x = curr.pt.x;
    int curr_y = curr.pt.y;
    float curr_theta = curr.theta;

    vector<point3D> neighbors;

    // position the current robot without rotation
    vector<point2D> curr_robot;
    
    point2D newp;
    newp.x = curr_x + robot_size_x;
    newp.y = curr_y + robot_size_y;
    curr_robot.push_back(newp);
    newp.x = curr_x + robot_size_x;
    newp.y = curr_y - robot_size_y;
    curr_robot.push_back(newp);
    newp.x = curr_x - robot_size_x;
    newp.y = curr_y - robot_size_y;
    curr_robot.push_back(newp);
    newp.x = curr_x - robot_size_x;
    newp.y = curr_y + robot_size_y;
    curr_robot.push_back(newp);


    // moving the robot up, down, left, right
    for (int x_disp = 1; x_disp > -2; x_disp--){
        for (int y_disp = -1; y_disp < 2; y_disp++){
            vector<point2D> new_robot;
            point2D newp;
            point3D new3p;

            newp.x= curr_x + x_disp * step_size + robot_size_x;
            newp.y = curr_y + y_disp * step_size + robot_size_y;
            new_robot.push_back(newp);

            newp.x= curr_x + x_disp * step_size + robot_size_x;
            newp.y = curr_y + y_disp * step_size - robot_size_y;
            new_robot.push_back(newp);

            newp.x= curr_x + x_disp * step_size - robot_size_x;
            newp.y = curr_y + y_disp * step_size - robot_size_y;
            new_robot.push_back(newp);

            newp.x= curr_x + x_disp * step_size - robot_size_x;
            newp.y = curr_y + y_disp * step_size + robot_size_y;
            new_robot.push_back(newp);

            newp.x = curr_x + x_disp * step_size;
            newp.y = curr_y + y_disp * step_size;
            new_robot = robot_rotation(new_robot, curr.theta, newp);


            if (non_overlap(new_robot)){
                if(!outofwindow(newp.x, newp.y)){
                    new3p.pt = newp;
                    new3p.theta = curr.theta;
                    neighbors.push_back(new3p);
                }
            }
        }
    }

    // rotation left and right by .5 rad (cannot rotate over 90 degrees in each direction)
    if (curr_theta < 3.1415926/2 - 0.5){
        vector<point2D> right_rotated_robot = robot_rotation(curr_robot, curr.theta + .5, curr.pt);
        if (non_overlap(right_rotated_robot)){
            point3D new3p;
            new3p.pt = curr.pt;
            new3p.theta = curr.theta + .5;
            neighbors.push_back(new3p);
        }
    }
    if (curr_theta > -3.1415926/2 + .5){
        vector<point2D> left_rotated_robot = robot_rotation(curr_robot, curr.theta - .5, curr.pt);
        if (non_overlap(left_rotated_robot)){
            point3D new3p;
            new3p.pt = curr.pt;
            new3p.theta = curr.theta - .5;
            neighbors.push_back(new3p);
        }
    }

    return neighbors;
}



// 3D A*
void a_star(point2D s, point2D g){
    next_step_index = 0;
    cout << "running a-star" << endl;

    path3D.clear();

    set<point3D> visited;
    set<point3D> to_examine;
    point3D start = {s, 0};
    point3D goal = {g, 0};
    to_examine.insert(start);

    map<point3D, point3D> prev;
    // distance from start to point3D
    map<point3D, int> gscore;
    // total cost of path passing through point3D
    map<point3D, int> fscore;

    gscore[start] = 0;
    fscore[start] = heuristic_cost(start, goal);

    while (!to_examine.empty()){

        point3D curr;
        int min = INT_MAX;

        for (set<point3D>::iterator pt = to_examine.begin(); pt != to_examine.end(); pt++){
            if (fscore[*pt] < min){
                min = fscore[*pt];
                curr = *pt;
            }
        }

        // current location of the robot
        vector<point2D> curr_robot;
        point2D newp;
        newp.x = curr.pt.x + robot_size_x;
        newp.y = curr.pt.y + robot_size_y;
        curr_robot.push_back(newp);
        newp.x = curr.pt.x + robot_size_x;
        newp.y = curr.pt.y - robot_size_y;
        curr_robot.push_back(newp);
        newp.x = curr.pt.x - robot_size_x;
        newp.y = curr.pt.y - robot_size_y;
        curr_robot.push_back(newp);
        newp.x = curr.pt.x - robot_size_x;
        newp.y = curr.pt.y + robot_size_y;
        curr_robot.push_back(newp);
        curr_robot = robot_rotation(curr_robot, curr.theta, curr.pt);

        // see if we have reached the goal
        if (insideConvexPolygon(goal.pt, curr_robot)){
            cout << "found it!!!!" << endl;
            prev[goal] = curr;
            break;
        }

        to_examine.erase(curr);
        visited.insert(curr);

        pin_point(curr.pt.x, curr.pt.y, red);

        // find the eligible neighbors and see if they should be examined next
        vector<point3D> neighbors = construct_neighbors(curr);
        for (vector<point3D>::iterator neighbor = neighbors.begin();
             neighbor != neighbors.end(); neighbor++){

            if (gscore.find(*neighbor) == gscore.end()) gscore[*neighbor] = INT_MAX;

            if (visited.find(*neighbor) != visited.end()) continue;
            if (to_examine.find(*neighbor) == to_examine.end()){
                to_examine.insert(*neighbor);
            }

            int temp_gscore;
            if (neighbor->theta != curr.theta){
                temp_gscore = gscore[curr] + 1;
            }

            temp_gscore = gscore[curr] + heuristic_cost(curr, *neighbor);

            if (temp_gscore >= gscore[*neighbor]) continue;

            prev[*neighbor] = curr;
            gscore[*neighbor] = temp_gscore;
            fscore[*neighbor] = gscore[*neighbor] + heuristic_cost(*neighbor, goal);
        }
    }

    // give a mocking message if the goal cannot be reached
    if (prev.find(goal) == prev.end()){
        cout << "No Path HAHAHAHAHHA" << endl;
        return;
    }

    // construct the path from goal to start
    while (prev.find(prev[goal]) != prev.end()){
        path3D.push_back(prev[goal]);
        goal = prev[goal];
    }
}



// draw the outline of the robot
void draw_robot(vector<point2D> robot, GLfloat *color){
    // set color
    int size = robot.size();
    glColor3fv(color);
    glLineWidth(2);
    for(int i= 0; i< robot.size(); i++){
        glBegin(GL_LINES);
        glVertex2f(robot[i].x, robot[i].y);
        glVertex2f(robot[(i+1)%size].x, robot[(i+1)%size].y);
        glEnd();
    }
    glLineWidth(1);
}



void display() {

    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity(); //clear the matrix


    /* The default GL window is [-1,1]x[-1,1] with the origin in the
     center.

     Our system of coordinates (in which we generate our points) is
     (0,0) to (WINSIZE,WINSIZE), with the origin in the lower left
     corner.

     We need to map the points to [-1,1] x [-1,1]

     Assume we are the local coordinate system.

     First we scale down to [0,2] x [0,2] */
    glScalef(2.0/WINDOWSIZE, 2.0/WINDOWSIZE, 1.0);
    /* Then we translate so the local origin goes in the middle of teh
     window to (-WINDOWSIZE/2, -WINDOWSIZE/2) */
    glTranslatef(-WINDOWSIZE/2, -WINDOWSIZE/2, 0);

    //now we draw in our local coordinate system (0,0) to
    //(WINSIZE,WINSIZE), with the origin in the lower left corner.
    pin_point(mouse_x, mouse_y, white);
    draw_obstacles(yellow);
    draw_polygon(new_obstacle, light_yellow);

    if (create_obstable_mode == 0 && change_start_pt_mode == 0 && change_finish_pt_mode == 0){
        draw_a_star_path(light_pink);
    }

    //draw a circle where the mouse was last clicked. Note that this
    //point is stored as a global variable and is modified by the mouse
    //handler function
    pin_point(start_x, start_y, light_blue);
    pin_point(finish_x, finish_y, light_blue);


    draw_robot(robot_verticies, light_blue);

    glFlush();
}



void keypress(unsigned char key, int x, int y) {
    switch(key) {
        case 'q':
            exit(0);
            break;

        // draw a new obstacle
        case 'n':
            if (change_finish_pt_mode == 1 || change_start_pt_mode == 1){
                change_start_pt_mode = 0;
                change_finish_pt_mode = 0;
            }
            cerr << "Creating a new obstacle polygon" << endl;
            create_obstable_mode = 1;

            glutPostRedisplay();
            break;

        // finish drawing the new obstacle
        case 'd':
            create_obstable_mode = 0;
            if (new_obstacle.size() > 2){
                if(valid_new_obstacle(new_obstacle)){
                    if (insidePolygon(start_x, start_y, new_obstacle)){
                        start_x = robot_size_x;
                        start_y = robot_size_y;
                    }
                    if (insidePolygon(finish_x, finish_y, new_obstacle)){
                        finish_x = WINDOWSIZE-robot_size_x;
                        finish_y = WINDOWSIZE-robot_size_y;
                    }

                    obstacle_set.push_back(new_obstacle);
                }
            }
            new_obstacle.clear();
            path3D.clear();
            glutPostRedisplay();
            break;

        // reselect a start point
        case 's':
            if (create_obstable_mode == 1){
                cerr << "Still trying to create a new polygon" << endl;
                glutPostRedisplay();
                break;
            }
            if (change_finish_pt_mode == 1) change_finish_pt_mode = 0;
            cout << "Recreating robot" << endl;
            change_start_pt_mode = 1;
            glutPostRedisplay();
            break;

        // reselect a finish point
        case 'f':
            if (create_obstable_mode == 1){
                cerr << "Still trying to create a new polygon" << endl;
                glutPostRedisplay();
                break;
            }
            if (change_start_pt_mode == 1) change_start_pt_mode = 0;
            cout << "Reselecting end point" << endl;
            change_finish_pt_mode = 1;
            glutPostRedisplay();
            break;

        // clear all obstacles
        case 'c':
            path3D.clear();
            obstacle_set.clear();
            new_obstacle.clear();
            glutPostRedisplay();
            break;

        // run the A* algorithm with the given start and finish points
        case 'a':
            if (create_obstable_mode == 1){
                cerr << "Still trying to create a new polygon" << endl;
                glutPostRedisplay();
                break;
            }
            change_start_pt_mode = 0;
            change_finish_pt_mode = 0;
            point2D start = {start_x, start_y};
            point2D finish = {finish_x, finish_y};
            a_star(start, finish);
            glutPostRedisplay();

    }
}



void mousepress(int button, int state, int x, int y) {


    if(state == GLUT_DOWN) {

        mouse_x = x;
        mouse_y = y;
        //(x,y) are in wndow coordinates, where the origin is in the upper
        //left corner; our reference system has the origin in lower left
        //corner, this means we have to reflect y
        mouse_y = WINDOWSIZE - mouse_y;

        printf("mouse click at (x=%d, y=%d)\n", (int)mouse_x, (int)mouse_y);

        if (change_start_pt_mode == 1 && valid_new_point(mouse_x, mouse_y)){
            start_x = mouse_x;
            start_y = mouse_y;

            robot_verticies.clear();
            point2D p1 = {start_x-robot_size_x, start_y-robot_size_y};
            point2D p2 = {start_x+robot_size_x, start_y-robot_size_y};
            point2D p3 = {start_x+robot_size_x, start_y+robot_size_y};
            point2D p4 = {start_x-robot_size_x, start_y+robot_size_y};

            robot_verticies.push_back(p1);
            robot_verticies.push_back(p2);
            robot_verticies.push_back(p3);
            robot_verticies.push_back(p4);
        } else if (change_finish_pt_mode == 1 && valid_new_point(mouse_x, mouse_y)){
            finish_x = mouse_x;
            finish_y = mouse_y;
        } else if (create_obstable_mode == 1){
            point2D p = {mouse_x, mouse_y};
            new_obstacle.push_back(p);
        }
    }

    glutPostRedisplay();
}



void timerfunc(){
    glutPostRedisplay();
}




/* ****************************** */
int main(int argc, char** argv) {

    initialize_obstacles();
    print_obstacles();

    start_x += robot_size_x;
    start_y += robot_size_y;
    finish_x -= robot_size_x;
    finish_y -= robot_size_y;

    point2D p1 = {start_x-robot_size_x, start_y-robot_size_y};
    point2D p2 = {start_x+robot_size_x, start_y-robot_size_y};
    point2D p3 = {start_x+robot_size_x, start_y+robot_size_y};
    point2D p4 = {start_x-robot_size_x, start_y+robot_size_y};
    
    robot_verticies.push_back(p1);
    robot_verticies.push_back(p2);
    robot_verticies.push_back(p3);
    robot_verticies.push_back(p4);

    /* initialize GLUT  */
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(WINDOWSIZE, WINDOWSIZE);
    glutInitWindowPosition(100,100);
    glutCreateWindow(argv[0]);
    
    /* register callback functions */
    glutDisplayFunc(display);
    glutKeyboardFunc(keypress);
    glutMouseFunc(mousepress);
    glutIdleFunc(timerfunc);
    
    /* init GL */
    /* set background color black*/
    glClearColor(0, 0, 0, 0);
    /* here we can enable depth testing and double buffering and so
     on */
    
    
    /* give control to event handler */
    glutMainLoop();
    return 0;
}

