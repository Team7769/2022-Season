package frc.robot.Utilities;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class PathFollower {
    private RamseteController _controller;
    private Trajectory _currentPath;
    private Timer _timer;

    public PathFollower()
    {
        _controller = new RamseteController();
        _timer = new Timer();
    }
    public void startPath()
    {
        _timer.reset();
        _timer.start();
    }
    public boolean isPathFinished()
    {
        return _timer.get() > _currentPath.getTotalTimeSeconds();
    }
    // public void setLineToTrenchPath(TrajectoryConfig config)
    // {
    //     _currentPath = getLineToTrenchTrajectory(config);
    // }
    // public void setTrenchToLinePath(TrajectoryConfig config)
    // {
    //     _currentPath = getTrenchToLineTrajectory(config);
    // }
    public void setTestPath(TrajectoryConfig config)
    {
        _currentPath = generateTestTrajectory(config);
    }

    public Trajectory getCurrentTrajectory()
    {
        return _currentPath;
    }
    public Pose2d getStartingPose()
    {
        return _currentPath.getInitialPose();
    }
    public DifferentialDriveWheelSpeeds getPathTarget(Pose2d currentPose)
    {
        var time = _timer.get();
        var goal = _currentPath.sample(time);
        var targetSpeeds = _controller.calculate(currentPose, goal);
        SmartDashboard.putNumber("goalDegrees", goal.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("totalPathTime", _currentPath.getTotalTimeSeconds());
        SmartDashboard.putNumber("goal", goal.poseMeters.getTranslation().getX());

        return Constants.kDriveKinematics.toWheelSpeeds(targetSpeeds);
    }
    /*private Trajectory generateTrajectory(TrajectoryConfig config) {

        var start = new Pose2d(3.129, 2.402, Rotation2d.fromDegrees(0));
        var end = new Pose2d(8.106, .75, Rotation2d.fromDegrees(0));
    
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(5.708, .75));
    
        var trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
        return trajectory;
      }*/
      private Trajectory generateTestTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
      }
    //   private Trajectory getLineToTrenchTrajectory(TrajectoryConfig config) {

    //     return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kCenterGoalLineX, Constants.kCenterGoalLineY, new Rotation2d(Constants.kCenterGoalLineStartAngle)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         new Translation2d(Constants.kTrenchPathMidpointX,Constants.kTrenchPathMidpointY)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(Constants.kTrenchPathEndX, Constants.kTrenchPathEndY, new Rotation2d(Constants.kTrenchPathEndAngle)),
    //     // Pass config
    //     config);
    //   }
    //   private Trajectory getTrenchToLineTrajectory(TrajectoryConfig config)
    //   {
    //     return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kTrenchPathEndX,Constants.kTrenchPathEndY, new Rotation2d(Constants.kTrenchPathEndAngle)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         new Translation2d(Constants.kTrenchPathMidpointX, Constants.kTrenchPathMidpointY)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(Constants.kCenterGoalLineX, Constants.kCenterGoalLineY, new Rotation2d(Constants.kCenterGoalLineStartAngle)),
    //     // Pass config
    //     config);
    //   }
    }