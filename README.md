## MuJoCo-Tutorials
Continuousy updated based on my progress on the 2018 edition of underactuated robotics course. You need to place your licence file ```mjkey.txt``` in the root of the repository and copy ```*.so.*``` files from bin of MuJoCo folder to ```libraries```.

Then, assuming you are in the root of the repository and ```$MJPRO``` is your path to the downloaded MuJoCo folder:

```sh
$ mkdir libraries && cp $MJPRO/bin/*.so* libraries/ 
$ mkdir build && cd &_
$ cmake .. && make
$ ./src/0_preliminaries/pd
```
If you see this, we are good :wink:

[![N|Solid](https://cdn-images-1.medium.com/max/800/1*A73_QqmDm3puXsBv4FldGg.gif)](https://medium.com/coinmonks/mujoco-tutorial-on-mits-underactuated-robotics-in-c-part-0-2cbd259f6adc)
