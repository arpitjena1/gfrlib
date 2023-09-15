# Gael Force Robotics - V 

## Eigen Port for the VEX V5
This is a port of the Eigen library for the VEX V5. It is a header-only library, so you can just include the headers in your project.

## Credit
All credit for Eigen goes to the Eigen team. See their [project page](https://eigen.tuxfamily.org/index.php?title=Main_Page) for more information

## Installation

> Note that only PROS is supported due to VEXCode using an ancient compiler

1. Download `Eigen@version.zip` from the [latest release](https://github.com/LemLib/Eigen/releases/latest)
2. Add the zip file to your project
3. Run `pros c fetch Eigen@version.zip; pros c apply Eigen@version` in the PROS integrated terminal

## Additional Versions

Want a different version of Eigen? Let us know by opening an issue [here](https://github.com/LemLib/Eigen/issues/new)

## Perks
 > Extended pneumatics class for solenoids &
 > State Machines

## Movement Functions
### boomerang
 ```

  using namespace gfr::chassis;
  move({{24,24,90}}, 127, gfr::ASYNC|gfr::THRU);

```
> Movement flags such as ASYNC to support asynchronous actions

### pure pursuit
```
  
  using namespace gfr::chassis;
  ASSET(path_txt)
  follow(path_txt,3000,10,true);

```
> Build paths on LemLib's path gen!

### Ramsete
```

  std::vector<gfr::Pose> *poses;
	
	Pose pose1 = {1,2,3};
	poses->push_back(pose1);

	FollowPath(poses,3000,200,10,10,false);

```
> Plug-in points and push back for the robot to follow the trajectory!

---
#### _Contributions welcome. Anything missing? Send in a pull request. Thanks._
Follow  ([@gaelforcev](https://instagram.com/gaelforcev)) on Instagram for updates.

---

