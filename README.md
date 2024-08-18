# Test Calculating Angular Velocity using Quaternion Differentiation

## Result Comparison

Red line (ang_vel_compare) is calculated using the method from <sup id="ref2"><a href="#ref2">[2]</a></sup> and <sup id="ref3"><a href="#ref3">[3]</a></sup>.

Blue line (my_ang_vel) is calculated myself, almost completely overlapped with red.

Grey line (imu_gryoscope_array) is the "ground truth" from <sup id="ref4"><a href="#ref4">[4]</a></sup>.

<p align="center">
  <img src="data/Pendulum_Test08_Trial1_Segment_1.csv_ang_vel.png" alt="Image 1" width="45%">
  <img src="data/TStick_Test02_Trial1.csv_ang_vel.png" alt="Image 2" width="45%">
</p>

More results in /data proved the equivalence.

## References

<a id="ref1"></a>[1] https://quaternions.online/

<a id="ref2"></a>[2] https://github.com/Mayitzin/ahrs/blob/c82197605ad719dedfa057ee8808b1df397e02ad/ahrs/common/quaternion.py#L3011

<a id="ref3"></a>[3] https://mariogc.com/post/angular-velocity-quaternions/

<a id="ref4"></a>[4] https://github.com/agnieszkaszczesna/RepoIMU
