package com.team2813.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{
    private final ClimbIO io; 

    public Climb(ClimbIO io) {
    this.io = io;
  }
  public void climbOuterHooks() {
    
  }
  public Command l3_Climb_Sequnce(){
      
    
  }
   public Command deploy_Climb(){

  }
   public Command l1_Climb_Sequnce(){

  }
   public Command l2_Climb_Sequnce(){

  }
   public Command post_Auto_Climb_Sequence(){

  }
  public Command retract_Passive_Hooks(){

  }
  public Command extend_Passive_Hooks(){

  }
}
