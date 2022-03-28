package frc.robot.Utilities;

public class VisionTargetState {
    private double _offset;
    private double _distance;

    public VisionTargetState(double offset, double distance)
    {
        _offset = offset;
        _distance = distance;
    }

    public double getOffset()
    {
        return _offset;
    }

    public double getDistance()
    {
        return _distance;
    }
}
