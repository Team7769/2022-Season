package frc.robot.Lib;

import edu.wpi.first.wpilibj.DigitalInput;

public class Photoeye extends DigitalInput {
    /**
     * Constructor for the Photoeye
     * @param port - Port to read the photoeye digital input from
     */
    public Photoeye(int port)
    {
        super(port);
    }

    /**
     * Get the state of the photoeye
     * @return true | false
     */
    public boolean isBlocked()
    {
        return !this.get();
    }
}
