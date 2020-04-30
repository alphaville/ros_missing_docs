# Messages

## The Message Header

In ROS, a message [header] is a special message type defined [here][header].
A header is defined as follows:

```
uint32  seq
time    stamp
string  frame_id
```

It has three attributes: (i) `seq` which is a consecutively increasing identifier,
(ii) a time stamp, of type `time` and (iii) a frame id, which is a string that
identifies which frame of coordinates the current message is associated with.

This information is often updated automatically. For example, we do not need to
- and must not - update `seq` (it is handled internally by ROS).


[header]: http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html



## Standard Messages

Work in progress


## Composite messages

Work in progress


## An Empty Message?!

Work in progress
