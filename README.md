# 1. This project is control component.
Based on the planning trajectory and the car's current status,
the Control module uses different control algorithms to generate a comfortable driving experience.

### Input
  * Planning trajectory
  * Car status(chassis)
  * Localization

### Output
  * Control commands (steering, accel, gear, throttle, brake) to the chassis.

# 2. How to compile?
https://wiki.jiduauto.com/pages/viewpage.action?pageId=275313286
https://jidudev.com/acu/integration

In general, the control code are inside integration/src/control
cd ~/integration
## 2.1 build orin
### build in one pack in order to run onboard ,if unit test is unnessary,without -u
```
acu_build control
```
### clean the build output
```
acu_build CLEAN
```
## 2.2 build x86
### build in one pack in order to run onboard ,if unit test is unnessary,without -u
```
acu_build -d x86 control
```
# 3.How to run?
## 3.1 On x86
```
source ./path/to/install/deploy_jidu_x86.bash /path/to/apolloos/folder
```

### If you received crash error ,please open another terminal and source the deploy.sh again.

you can just `./control_test`(in install/bin/) run ut or `pavaro -c dag/control.dag` run pavaro 
If you test on x86 with logsim/worldsim, you can use 'pavaro -c control.dag'

## 3.2 onboard
please read  /intergration/scripts/packer/README.sh

```
./scripts/packer/packer.sh <packagename>
```
then use scp to transport <packagename>.tar.gz to orin.

*to be acomplish.....*

If you have questions, please contact chi.zhang@jiduauto.com.
