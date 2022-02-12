package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Collector implements ISubsystem {

private static Collector _instance;   
    
    private CANSparkMax _CollectorMotor;

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
        _CollectorMotor = new CANSparkMax(13, MotorType.kBrushless);
        _CollectorMotor.setInverted(true);
        _CollectorMotor.setIdleMode(IdleMode.kBrake);
    }

    /** 
     * Moves collect motor to a certain position 
     * @param speed - Speed you want to set motor to
    */
    public void moveCollect(double speed) {
        _CollectorMotor.set(speed);
    }

    /**
     * Stops the collect motor from moving
     */
    public void stopCollect() {
        _CollectorMotor.set(0.0);
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }

}
 
