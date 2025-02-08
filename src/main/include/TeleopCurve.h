class TeleopCurve{
    public:
    TeleopCurve(){};

    static double apply(double input){
        double output = input*input*input;
        return output;
    }
};