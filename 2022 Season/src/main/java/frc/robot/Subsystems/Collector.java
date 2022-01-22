package frc.robot.Subsystems;

public class Collector {

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

}
 
