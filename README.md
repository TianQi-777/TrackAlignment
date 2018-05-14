# TrackAlignment
This is a small demo to align and plot SLAM estimated trajectories and ground-truth trajectories.

## Mathematical theory
In order to better understand this demo, you need to understand some knowledge about [ICP](https://en.wikipedia.org/wiki/Iterative_closest_point) and [SVD](https://en.wikipedia.org/wiki/Singular-value_decomposition).  
Papers on ICP Matching:[A Review of Point Cloud Registration Algorithms for Mobile Robotics](http://xueshu.baidu.com/s?wd=paperuri%3A%281f56b0259dec2c0938045f62ba42d450%29&filter=sc_long_sign&tn=SE_xueshusource_2kduw22v&sc_vurl=http%3A%2F%2Fciteseerx.ist.psu.edu%2Fviewdoc%2Fdownload%3Fdoi%3D10.1.1.709.2212%26rep%3Drep1%26type%3Dpdf&ie=utf-8&sc_us=10337083750033872330)

## Data description  
**compare.txt**:Record ground-truth and estimated trajectories.  

**Data storage form**   
Time-g  Translation-x-g  Translation-y-g  Translation-z-g  Quaternion-x-g  Quaternion-y-g  Quaternion-z-g  Quaternion-w-g  
Time-e  Translation-x-e  Translation-y-e  Translation-z-e  Quaternion-x-e  Quaternion-y-e  Quaternion-z-e  Quaternion-w-e  
g means ground-truth trajectories  
e means estimated trajectories  

## Additional Prerequisites for this demo  
**Pangolin**  
Use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and interface. 
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

**Sophus**  
Use [Sophus](https://github.com/strasdat/Sophus) for Lie groups commonly used for 2d and 3d geometric problems. 
Dowload and install instructions can be found at: https://github.com/strasdat/Sophus.  

## Build and Run
```
cd XX/XX(include compare.cpp ,compare.txt and CMakeLists.txt)  
mkdir build  
cd build  
cmake ..  
make -j2  
./compare
```  

## Result
**Pangolin GUI:**    
<div align=center>  
  
![](https://github.com/TianQi-777/TrackAlignmentWith_ICP/blob/master/Images/compare_.jpg)
</div>



