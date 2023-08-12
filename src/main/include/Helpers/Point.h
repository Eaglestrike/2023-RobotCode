#pragma once

#include <cmath>
#include <string>
#include <sstream>
#include <iostream>

class Point{
    public:
        Point():x_(0.0), y_(0.0){};
        Point(double x, double y): x_(x), y_(y) {}
        Point(const Point& other) : x_(other.x_), y_(other.y_) {}

        inline double originDist() {
            return sqrt(x_*x_ + y_*y_);
        }

        inline double getMagnitude(){
            return originDist();
        }

        inline double dist(Point p) {
            double dx = p.x_ - x_;
            double dy = p.y_ - y_;
            return sqrt(dx*dx + dy*dy);
        }

        inline double getAng() {
            return atan2(y_, x_);
        }

        void move(double dx, double dy){
            x_ += dx;
            y_ += dy;
        }

        void move(Point p){
            x_ += p.x_;
            y_ += p.y_;
        }


        /// @brief Rotates counterclockwise
        /// @param ang Radians
        /// @return rotated point around origin
        Point rotate(double ang){
            //[cos(a)  -sin(a)]
            //[sin(a)  cos(-a)]
            double nx = cos(ang) * x_ - sin(ang) * y_;
            double ny = sin(ang) * x_ + cos(ang) * y_;
            return Point(nx, ny);
        }

        /***
         * Rotates counterclockwise
        */
        void rotateThis(double ang){//Radians
            //[cos(a)  -sin(a)]
            //[sin(a)  cos(-a)]
            double nx = cos(ang) * x_ - sin(ang) * y_;
            double ny = sin(ang) * x_ + cos(ang) * y_;
            x_ = nx;
            y_ = ny;
        }

        inline Point rotateClockwise90(){
            return {y_, -x_};
        }

        inline Point rotateCounterclockwise90(){
            return {-y_, x_};
        }

        void rotateClockwise90This(){
            double xtemp = x_;
            x_ = y_;
            y_ = -xtemp;
        }

        void rotateCounterclockwise90This(){
            double xtemp = x_;
            x_ = -y_;
            y_ = xtemp;
        }
        
        /// @brief gets the point with the magnitude and angle from the origin
        /// @param mag magnitude
        /// @param ang radians
        /// @return the point
        static Point extend(double mag, double ang){
            return {mag*cos(ang), mag*sin(ang)};
        }

        /// @brief gets the point with the magnitude and angle from the origin
        /// @param mag magnitude
        /// @param ang degrees
        /// @return the point
        static Point extendDeg(double mag, double ang){
            ang *= M_PI/180.0;
            return {mag*cos(ang), mag*sin(ang)};
        }

        double getX(){return x_;}
        double getY(){return y_;}
        void setX(double x){x_ = x;}
        void setY(double y){y_ = y;}

        std::string toString(){
            std::stringstream ss;
            ss<<"("<<x_<<","<<y_<<")";
            return ss.str();
        }

        void print(){
            std::cout<<"("<<x_<<","<<y_<<")"<<std::endl;
        }

        void print(std::string name){
            std::cout<<name<<": ("<<x_<<","<<y_<<")"<<std::endl;
        }

        Point& operator= (const Point& p){
            x_ = p.x_;
            y_ = p.y_;
            return *this;
        }

        Point& operator+= (const Point& p){
            x_ += p.x_;
            y_ += p.y_;
            return *this;
        }

        Point& operator-= (const Point& p){
            x_ -= p.x_;
            y_ -= p.y_;
            return *this;
        }

        Point& operator*= (double k){
            x_ *= k;
            y_ *= k;
            return *this;
        }

        Point& operator/= (double k){
            x_ /= k;
            y_ /= k;
            return *this;
        }

        Point& operator+ (const Point& p){
            return *(new Point(x_ + p.x_, y_ + p.y_));
        }

        Point& operator- (const Point& p){
            return *(new Point(x_ - p.x_, y_ - p.y_));
        }

        Point& operator* (double k){
            return *(new Point(k * x_, k * y_));
        }

        Point& operator/ (double k){
            return *(new Point(x_ / k, y_ / k));
        }

    private:
        double x_;
        double y_;
};
typedef Point Vector;