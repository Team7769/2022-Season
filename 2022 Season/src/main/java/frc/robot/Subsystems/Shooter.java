package frc.robot.Subsystems;

public class Shooter implements ISubsystem {

    public static Shooter _instance;

    /** 
     * Gets the shooter instance
    */
    public static Shooter GetInstance() {
        if (_instance == null) {
            _instance = new Shooter();
        }
        return _instance;
    }

    /** 
     * Constructor for implementing Shooter
    */
    public Shooter() {

    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
