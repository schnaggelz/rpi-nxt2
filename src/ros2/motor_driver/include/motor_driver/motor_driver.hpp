class MotorDriver {
public:
    MotorDriver();
    void initialize();
    void control(int speed, bool direction);
};