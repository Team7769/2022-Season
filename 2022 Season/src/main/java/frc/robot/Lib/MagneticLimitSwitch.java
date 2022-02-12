package frc.robot.Lib;

import edu.wpi.first.wpilibj.DigitalInput;

public class MagneticLimitSwitch extends DigitalInput {
    
    public MagneticLimitSwitch(int port)
    {
        super(port);
    }

    public boolean isBlocked()
    {
        return !this.get();
    }
}
