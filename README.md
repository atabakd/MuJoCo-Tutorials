## MuJoCo-Tutorials
Continuousy updated in [my blog](https://atabakd.github.io/blog/) based on my progress on the 2018/9 edition of underactuated robotics course. You need to place your licence file ```mjkey.txt``` in the root of the repository and copy ```*.so.*``` files and ```libglfw3.a```from bin of MuJoCo 2.0 folder to ```libraries```.

Then, assuming you are in the root of the repository and ```$MJPRO``` is your path to the downloaded MuJoCo folder:

```sh
$ mkdir libraries && cp $MJPRO/bin/*.so* libraries/ && cp $MJPRO/bin/libglfw3.a libraries/
$ mkdir build && cd $_
$ cmake .. && make
$ cd src/0_preliminaries
$ ./pd
```
If you see this, we are good :wink:

[![N|Solid](https://cdn-images-1.medium.com/max/800/1*A73_QqmDm3puXsBv4FldGg.gif)](https://medium.com/coinmonks/mujoco-tutorial-on-mits-underactuated-robotics-in-c-part-0-2cbd259f6adc)
