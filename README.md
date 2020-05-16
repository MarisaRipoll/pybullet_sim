# Pybullet_sim


## Trying to pull the string:

- For URDF version:
```bash
python3 human_pull_string_moveit.py
```

## Problems:

1. How do I fix the debugline's starting point in the link frame origin and the debugline's ending point in the world frame?
2. To see the implementation of the setJointMotorControlArray uncomment the line before the last loop. (The non-fixed part of the figure moves uncontrollably, even when only the left arm link indices are specified).
3. The same reaction is obtained when I create a costraint.
4. The biggest problem is that applyExternalForce doesnt make the figure move.


