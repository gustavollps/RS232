class DigitalFilter{
  public:
    //First order constructor
    /*
     * y(z) = a0 + a1*z(-1) + a2*z(-2)
     * u(z)   b0 + b1*z(-1) + b2*z(-2)
    */
    DigitalFilter(double Ky0, double Ky1, double Ku0, double Ku1);

    //Second crdem constructor
    DigitalFilter(double Ky0, double Ky1, double Ky2, double Ku0, double Ku1, double Ku2);

    double Filter(double read);

  private:
    double y_0_,y_1_,y_2_;
    double x_0_,x_1_,x_2_;
    double a0_,a1_,a2_;
    double b0_,b1_,b2_;
};
