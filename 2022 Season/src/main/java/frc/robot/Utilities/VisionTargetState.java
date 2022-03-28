package frc.robot.Utilities;

public class VisionTargetState {
    private double _offset;
    private double _distance;

    public VisionTargetState(double offset, double distance)
    {
        _offset = offset;
        _distance = distance;
    }

    /**
     * The offset of the VisionTargetState
     * @return offset
     */
    public double getOffset()
    {
        return _offset;
    }

    /**
     * The distance of the VisionTargetState
     * @return distance to target
     */
    public double getDistance()
    {
        return _distance;
    }
}
