package frc.robot.Subsystems;

public class Collector implements ISubsystem {

private static Collector _instance;   
    
    public static Collector GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Collector();
        }
        return _instance;
    }

    public Collector() 
    { 
        
    }

    @Override
    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }

}
 
