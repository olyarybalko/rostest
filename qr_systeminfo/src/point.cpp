#include "point.h"
#include <cmath>
#include <iostream>
using namespace std;
void Point::setX(double x)
{
 this->x = x;
}
void Point::setY(double y)
{
 this->y = y;
}
double Point::getX()
{
 return x;
}
double Point::getY()
{
 return y;
}
double Point::distance(const Point &P)
{
 double dx, dy;
 dx = x - P.x;
 dy = y - P.y;
 return sqrt(dx*dx + dy*dy);
}
Point Point::milieu(const Point &P)
{
 Point M;
 M.x = 0.5*(P.x + x);
 M.y = 0.5*(P.y + y);
 return M;
}

void Point::afficher()
{
 cout << "L'abscisse vaut " << x << endl;
 cout << "L'abscisse vaut " << y << endl;
} 