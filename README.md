TODO:

* General
  * figure out balancing via the Navx
  * figure out how to tie command groups to buttons

* Autonomous
  * figure out detecting mutlipe april tags
  * make functions like drive to april tags n stuff

* Climber
  * figure out units limit for climber
  * create methods to auto climb to top/level 1 DONE
  * add managing the hooks(i kinda forgot the mechanism we were planning on using so i didnt add it)

* launcher
  * create a command group to start launching it, with like a delway so it can spin up

* intake
  * create class for intake
    * add methods, should be rudimentary


* photon vision
  * April Tag detection
  * pose estimation


* drive
  * quality of life: fix the bug that has the wheels snap to position when going to deadband
    * look into advantage scope
  * look into exponential drive
  * look into adding a trigger for driving slowdown

* Controller Button Bindings
  * button mappings as of 10/25 11:13 AM are located at:
    * https://www.padcrafter.com/?templates=Controller+Scheme+1&leftTrigger=elevator+down&rightTrigger=elevator+up&rightBumper=coral+dispense&leftBumper=coral+back&leftStickClick=change+speed&leftStick=movement&rightStick=rotation&xButton=set+wheels+to+x&aButton=go+to+height+%2235%22&yButton=reset+encoders%2C+only+do+when+elevator+is+fully+down&startButton=deploy+coral+fully&bButton=zero+heading%28sets+rotation+to+zero+i+think%29d
  * possible optimizations of the keybinds via the CommandXboxController.
  * add a secondary controller, or a way to implement it DONE
  * add strafing to the robot via d-pad or jusst a button to toggle field oriented driving

* set up simulator
  *



