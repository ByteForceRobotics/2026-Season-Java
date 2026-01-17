TODO:
* Autonomous
  * port over auto code from previous python code for following april tags
  * pathplanner (https://pathplanner.dev/home.html)
    * move the robot config to the constants file

* reef managment subsystem
  * Coral intake 
    * simple motor control: done
    * optical sensors
      * once they detect the piece of coral, have the coral intake prime it for deployment, detect it has reached the end via a second set of opticals sensors (see if possible without a second set) DONE
  * algae removal
    * simple motor control DONE
    * bumper detection + roll up (when the jaw runs into the reef)

* photon vision
  * April Tag detection
  * pose estimation
  * reef alignment

* elevator
  * Program actual set points
  * DONE replace braking with active position alignment 
  * create a variable with setgoalPosition, either use a bang bang controller or PID controller, or maybe just use the goto method we have   TOO MCUH EFFORT NOT ENOUGH TIME
  * add a button to lower elevator to zero DONE
  * limit the elevator to 4th coral level so no overshooting should bearound-133-135 DONE
  * if no coral in either beam breaks, then automatically go to position 0 NOT A GOOD IDEA

* drive
  * quality of life: fix the bug that has the wheels snap to position when going to deadband
    * look into advantage scope
    * if robot is moving and eleavtor is up then lower elevator DONE
      * as a failsafe make it only active when the elevator is almost at the top DONE
    * if elevator is lifted then lower movement speed DONE

* Controller Button Bindings
  * button mappings as of 10/25 11:13 AM are located at:
    * https://www.padcrafter.com/?templates=Controller+Scheme+1&leftTrigger=elevator+down&rightTrigger=elevator+up&rightBumper=coral+dispense&leftBumper=coral+back&leftStickClick=change+speed&leftStick=movement&rightStick=rotation&xButton=set+wheels+to+x&aButton=go+to+height+%2235%22&yButton=reset+encoders%2C+only+do+when+elevator+is+fully+down&startButton=deploy+coral+fully&bButton=zero+heading%28sets+rotation+to+zero+i+think%29d
  * possible optimizations of the keybinds via the CommandXboxController.
  * add a secondary controller, or a way to implement it DONE
  * add strafing to the robot via d-pad or jusst a button to toggle field oriented driving

* set up simulator
  *



ADd exponential drive


and also implemnt trigger for driving slowdown

