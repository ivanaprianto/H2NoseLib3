#include <math.h>
#include <iostream>
#include <stdio.h>
using namespace std;

int main ()
{
    double x, x0, x1, y, y0, y1;
    cout << "x value (max readable ppm)\n";
    cin >> x;
    cout << "x0 value (min readable ppm)\n";
    cin >> x0;
    cout << "y value (max readable ppm y axis)\n";
    cin >> y;
    cout << "y0 value (min readable ppm y axis)\n";
    cin >> y0;
    cout << "x1 value (intercept ppm)\n";
    cin >> x1;
    cout << "y1 value (intercept y axis)\n";
    cin >> y1;
    double m = log10(y/y0) / log10(x/x0);
    double b = log10(y1) - (m * log10(x1));
    cout << "M : " << m << "\n";
    cout << "B : " << b << "\n";
}