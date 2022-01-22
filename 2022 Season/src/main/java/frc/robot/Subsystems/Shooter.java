package frc.robot.Subsystems;

public class Shooter implements ISubsystem {

    private static Shooter _instance;

    /** 
     * Returns instance of Shooter 
    */
    public static Shooter GetInstance() {
        if (_instance == null) {
            _instance = new Shooter();
        }

        return _instance;
    }

    /** 
     * Initializes Shooter
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
