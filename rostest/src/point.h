#ifndef POINT_H
#define POINT_H
class Point
{

    private:
        double x,y;

    public:
        
        Point(){ x = 0.0; y = 0.0;}

        

        Point(double x, double y){ this->x = x;  this->y = y;}

        
        Point(const Point &p) {this->x = p.x; this->y = p.y; } 

        void setX(double x);
        void setY(double y);
        double getX();
        double getY();
        double distance(const Point &P);
        Point milieu(const Point &P);
        void saisir();
        void afficher();

};
#endif