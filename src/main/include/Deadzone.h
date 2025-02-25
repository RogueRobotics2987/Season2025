class Deadzone{
    public:
    
    Deadzone(){};

    static double applyDeadzone(double input){
        double output = input;
        if ((input < 0.1) &&  (input > -0.1)){ // deadzone
            output = 0;
        }
        return output;
    }
    
    };