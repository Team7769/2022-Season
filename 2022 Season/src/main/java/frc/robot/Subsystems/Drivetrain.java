package frc.robot.Subsystems;

public class Drivetrain {
    
    private static Drivetrain _instance;

    public static Drivetrain GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Drivetrain();
        }
        return _instance;
    }

    public Drivetrain()
    {
        
    }
}
