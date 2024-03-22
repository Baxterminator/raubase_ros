# Why this project ?

During 2024, 9 intensive weeks of this course, I observed several things:

- A lot of people never touch a robot before and thus can be a bit lost while digging in the original code,
- Moreover, a lot do not know about C++ either, which make the discovery and understanding process even more complicated,
- The original code is working, but I felt the structure and data sharing a bit messy,
- If we wanted to add anything, we had to it from scratch and in C++,
- The Raspberry Pi, even if we have RPi 4 with 8GB, they are still limiting in computation power, autonomy (the robot on battery), lacks of GPU, ..., and it is not that easy to integrate a cloud computing solution of the already existing code without having to rewrite a whole communication part.

For this reason, we decided to switch to ROS with a few goals in minds:

1. **Improved Backend**
    1. **Robust**: Make a more robust system by splitting the program in Node communicating between them with DDS.
    2. **External Server Computing** the DDS communication allow an easy Cloud Computing implementation,
    3. **Python support** using ROS put a new programming language in the balance - Python - that is a bit more known than C++.
2. **Opening the door to every ROS2 available tools**
    1. **Do not reinvent the wheel**: a lot of things have already been done, especially about the navigation stack or even on sensor fusion, ... With a ROS2 integration, we open the door to all of these tools.
3. **High-level User Friendly Interface** 
    1. **Easy to program**: as everyone is not that equal in skills about programming, this interface should provide an easy task description and running.
    2. **As simple as possible**: even when adding Python in the balance, a lot of people doesn't know both languages that much. The interface should be as simple as possible so that everyone could write at least simple tasks.
    3. **ROS agnostic**: since ROS is not that well-known either, the interface should abstract all ROS works, at least for the simple tasks.