// Program will give robot has more control at lower speeds
// We will do this with the equation y=x^3
// 1. Make a class called TeleopCurve
// 2. In this class you will have one function with a return type of double, one parameter of type double, and the function should be called apply.
// 3. The apply function should calculate the equation y=x^3 and return the value y. x is the parameter of the function.

class TeleopCurve{
    public:
    TeleopCurve(){};

    static double applyFast(double input){
        double output = input;//*input*input;//*input*input;
        if ((input < 0.1) &&  (input > -0.1)){
            output = 0;
        }
        return output;
    }
    
        static double applyFine(double input){
        double output = input / 6;
        if ((input < 0.5) &&  (input > -0.5)){ // deadzone
            output = 0;
        }
        return output;
    }};
// TeleopCurve::apply(joystick.GetLeftX());