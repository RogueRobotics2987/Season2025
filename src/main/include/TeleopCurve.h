// Program will give robot has more control at lower speeds
// We will do this with the equation y=x^3
// 1. Make a class called TeleopCurve
// 2. In this class you will have one function with a return type of double, one parameter of type double, and the function should be called apply.
// 3. The apply function should calculate the equation y=x^3 and return the value y. x is the parameter of the function.

class TeleopCurve{
    public:
    TeleopCurve(){};

    static double apply3Fine(double input){
        double output = input*input*input;
        output = output/4;
        if ((output < 0.001) &&  (output > -0.001)){
            output = 0;
        }
        return output;
    }

    static double apply3Fast(double input){
        double output = input*input*input;
        if ((output < 0.001) &&  (output > -0.001)){
            output = 0;
        }
        return output;
    }

    static double applyCube2024(double input){
        double output = input * input * input;  // exponetial curve, slow acceleration at begining

        if ((output < 0.001) &&  (output > -0.001)){
            output = 0;
        } else if (output >= 0.001){
            output = output;
        } else if (output <= -0.001){
            output = output ;
        }
        return output;
    }
};

// TeleopCurve::apply(joystick.GetLeftX());