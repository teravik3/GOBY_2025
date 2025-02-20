package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil;
import frc.robot.utilities.FieldPoseUtil.CoralStationSubPose;

public class GetCoral extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final HandlerSubsystem m_handler;
  private final FieldPoseUtil m_fieldPoseUtil = new FieldPoseUtil();

  public GetCoral(DriveSubsystem drive, HandlerSubsystem handler, CoralStationSubPose subPose) {
    m_drive = drive;
    m_handler = handler;
    addRequirements(m_drive, m_handler);

    Command driveToPose = new DriveToPose(m_fieldPoseUtil.getTargetPoseAtStation(m_fieldPoseUtil.closestStation(m_drive.getPose()), subPose), drive);

    addCommands(
      driveToPose,
      //TODO: pivot and elevator (from crane subsystem)
      Commands.runOnce(() -> m_handler.intakeCoral()),
      Commands.waitUntil(() -> m_handler.isLoadedCoral())
    );
  }


}
