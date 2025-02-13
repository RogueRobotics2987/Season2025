// Program will give robot has more control at lower speeds
// We will do this with the equation y=x^3
// 1. Make a class called TeleopCurve
// 2. In this class you will have one function with a return type of double, one parameter of type double, and the function should be called apply.
// 3. The apply function should calculate the equation y=x^3 and return the value y. x is the parameter of the function.

class TeleopCurve{
    public:
    TeleopCurve(){};

    static double applyFine(double input){
        double output = input;//*input*input*input*input;
        output = input / 4;
        if ((input < 0.05) &&  (input > -0.05)){
            output = 0;
        }
        return output;
    }

    static double applyFast(double input){
        double output = input;//*input*input*input*input;
        if ((input < 0.05) &&  (input > -0.05)){
            output = 0;
        }
        return output;
    }

    static double applyFineR(double input){ // for rotation
        double output = input/5;//*input*input;//*input*input;
        output = output/5;
        if ((input < 0.075) &&  (input > -0.075)){
            output = 0;
        }
        return output;
    }

    static double applyFastR(double input){ // for rotation
        double output = input;//*input*input*input*input;
        if ((input < 0.075) &&  (input > -0.075)){
            output = 0;
        }
        return output;
    }

    static double apply2023(double input, bool rotation){ // Mantis control curve
        double output = input;
        if((input < 0.01) && (input > -0.01)) {
            output = 0;
        }
        if(rotation == true){
            if(output >= 0.1) {
                output = output - 0.1;
            }
            else if(output <= -0.1) {
                output = output + 0.1;
            }
        }
        return output;
    }
};

// TeleopCurve::apply(joystick.GetLeftX());