package frc.robot.Subsystems;

public class Climber {

    private static Climber _instance;

    /** 
     * Used to get Instance of the Climber
    */
    public static Climber GetInstance() {

        if (_instance == null){
            _instance =  new Climber();
        }

        return _instance;
    }

    /** 
     * Constructor for climber
    */
    public Climber(){}
}
