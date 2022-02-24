package frc.robot.Subsystems;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Configuration.Constants;

public class Climber implements ISubsystem {

    private DoubleSolenoid _climber;
    private DoubleSolenoid _ratchet;
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
    public Climber(){
        _climber = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kClimberSolenoidForwardChannel, Constants.kClimberSolenoidReverseChannel);
        _ratchet = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kRatchetSolenoidForwardChannel, Constants.kClimberSolenoidReverseChannel);
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
