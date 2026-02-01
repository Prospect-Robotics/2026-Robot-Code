![Image](../../../../../../../documentation/images/intake_and_vectoring_motor_CAN_ID_diagram.png)

## CanIDs
Directions of motor are when robot is intaking.
- **23**: Left Feeder/Vectoring Motor
  - Rotates *Counter-Clockwise*
- **24**: Right Feeder/Vectoring Motor
  - Rotates *Clockwise*
- **22**: Follower Roller/Magazine Motor
  - Rotates *Clockwise*
  - Follows **25**
- **25**: Main Roller/Magazine Motor
  - Rotates *Clockwise*

Motor 22 and 25 spin the rollers of the system, moving fuel elements toward the vectoring motors and shooter.

Motors 23 and 24 spin the vectoring "triangles" condensing a bunch of balls into a single stream for the shooter.
