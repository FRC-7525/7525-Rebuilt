1. Better Aimlock (Yagsl Steal)
2. Setup for tuning
3. Sim testing
4. Rewrite
5. Otto things
6. Merge conflicts

### Tuning order:
* Get correct ids from old laptop
* Find id for 2nd kicker
* Get Hood PID
* Get Hood Min and Max
* Tune wheels with WHEEL_TUNING_DASH
* Test full spin up + down plus micro adjust. 
* Tape over sharp edges
* Test all spinning. attempt to get consistent shot with all.
* Test drive
* Delete dis
* Undo Testing Mode + Uncomment stuffs
* Push and PR

### Tuning Controls:
* addTrigger(ManagerStates.IDLE, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);
* addTrigger(ManagerStates.WINDING_UP_FIXED_SHOT, ManagerStates.SHOOTING_FIXED, DRIVER_CONTROLLER::getBButtonPressed);
* addTrigger(ManagerStates.SHOOTING_FIXED, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getBButtonPressed);

