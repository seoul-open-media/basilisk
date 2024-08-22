## Todo
### Basilisk
- Try stabilizing backlashing gear by switching output source encoders
- Encoder validity check, WARN if not
- WARN if electromagnet is set high for a certain amount of time
- Xbee communication between host computer and Teensy's
- Use Serial Plotter
- "d exact" does not preserve Reply position at power off
- Let CommandReceiver parse external signal to Basilisk::Command, not Executer::Parse.
- Use fields instead of global functions, in order to configure details(IDs, etc.) on main(.ino) file.