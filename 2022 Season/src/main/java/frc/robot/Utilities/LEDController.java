package frc.robot.Utilities;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDController {
    private static Spark _lowerBlinkin;
    private static Spark _upperBlinkin;
    public static LEDController _instance;

    public final double kTrackingTarget = -0.31;
    public final double kOnTarget = -0.23;
    public final double kWaitingForConfirmation = .77;
    public final double kFireLarge = -.57;
    public final double kBlueHeartBeat = -.23;
    public final double kRedHeartBeat = -.25;
    public final double color1HeartBeat = .07;
    public final double color2HeartBeat = .27;
    public final double bpmParty = -.67;
    public final double kYellow = .69;
    
    public LEDController()
    {
        _lowerBlinkin = new Spark(1);
        _upperBlinkin = new Spark(0);
    }
    public static LEDController GetInstance()
    {
        if (_instance == null)
        {
            _instance = new LEDController();
        }
        return _instance;
    }

    public void setLowerLED(double value)
    {
        _lowerBlinkin.set(value);
    }

    public void setUpperLED(double value)
    {
        _upperBlinkin.set(value);
    }

    public void setTrackingTargetState()
    {
        _upperBlinkin.set(kTrackingTarget);
    }
    public void setOnTargetState()
    {
        _upperBlinkin.set(kOnTarget);
    }

    public void setWaitingForConfirmation()
    {
        _upperBlinkin.set(kWaitingForConfirmation);
    }

    public void setFire()
    {
        _upperBlinkin.set(kFireLarge);
    }
}