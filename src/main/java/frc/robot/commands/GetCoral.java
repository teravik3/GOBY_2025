package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.HandlerSubsystem;
import frc.robot.utilities.FieldPoseUtil.CoralStationSubPose;

public class GetCoral extends SequentialCommandGroup {
  private final HandlerSubsystem m_handler;
  private final Crane m_crane;

  public GetCoral(HandlerSubsystem handler, Crane crane, CoralStationSubPose subPose) {
    m_handler = handler;
    m_crane = crane;
    addRequirements(m_handler, m_crane);

    addCommands(
      Commands.runOnce(() -> m_crane.moveTo(CraneConstants.kPositionIntake)),
      Commands.runOnce(() -> m_handler.intakeCoral()),
      Commands.waitUntil(() -> m_handler.isLoadedCoral()),
      Commands.runOnce(() -> m_crane.moveTo(CraneConstants.kPositionHome)),
      Commands.waitUntil(() -> m_crane.atSetpoint().isPresent())
    );
  }
}
