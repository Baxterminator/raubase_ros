# Multicast server for ROS2 in DTU

As stated in the [goals of the project](../../introduction/why), one of the goal was to enable Cloud Computing with the robot. This means that you need to communicate between the robot and your computer. However, DTU routers configurations prevent the usage of the default communication protocols for ROS2. On this page, you will the informations you need on what problem arise will using ROS2 in DTU network, and how to fix it.

## The problem: ROS2 discovery protocol

If you try to make two computers running ROS2 communicate over the DTU network you will quickly see that they do not see each others. On the other side, ROS1 projects are running very well without any specific configurations. Why? The problem comes from how nodes are discovered in both versions.

In ROS1, you have a master (i.e. a broker) that will centralized the dynamic sharing of information on every DDS participant. In this case, you configure the IP of the master on each device that will participate, so the communication only work on *unicast* (i.e. sending information from one device to one another). As this communication method does not rely on anything from the router point of view (except from redirecting the message to the IP), it works fine on the DTU network.

ROS2 on the other hand choose to use a decentralized discovery protocol, thus there is no fixed "master" to redistribute the data. In fact, ROS2 rely on *multicast* (i.e. one device sending a message to all devices on a network using a gateway). However, this method is blocked inside of the DTU Network. 

## How to solve this problem ?

The solution is in fact really trivial, we just have to go back to the idea of ROS1: having a broker for discovery. I will here describe the method for the FastDDS RMW, but others RMW may have similar options.

We will split between the server and the clients, but it's the same thing, except for the IP that you'll have to enter. You find references about the setup on the FastDDS wiki [^1] and on this Medium article[^2]. 


### On the server host (typically the robot)

On the host, create a file `fastdds_conf.xml`[^3] somewhere, and put the contents of [this file](https://fast-dds.docs.eprosima.com/en/latest/_downloads/9f9e92b14612364b742c8ecde24b3d24/super_client_configuration_file.xml) inside.
```shell
curl https://fast-dds.docs.eprosima.com/en/latest/_downloads/9f9e92b14612364b742c8ecde24b3d24/super_client_configuration_file.xml > fast_dds.conf
```

Now, you can add the following lines in your .bashrc (or .zshrc if you use zsh):

```shell
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path_to_your_file/fastdds_conf.xml
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
```

Finally to apply the changes, you just have to restart the ros2 daemon to apply to changes:

```shell
ros2 daemon stop
ros2 daemon start
```

### On the clients side

For the clients, the processus is a bit the same as for the server. However, for the server we used the loopback IP (127.0.0.1), and thus won't work for the client. The first thing you have to do is to get the server IP:

```shell title="Run this on the robot"
ifconfig
```

You will then have a result like this:

```shell hl_lines="10 11"
lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 432746  bytes 178993688 (178.9 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 432746  bytes 178993688 (178.9 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlp0s20f3: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 10.15.222.62  netmask 255.255.240.0  broadcast 10.15.223.255
        inet6 fe80::289a:b1c0:5f05:dcf  prefixlen 64  scopeid 0x20<link>
        ether ec:63:d7:fd:8a:fe  txqueuelen 1000  (Ethernet)
        RX packets 8618102  bytes 10333207988 (10.3 GB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 1315555  bytes 343510087 (343.5 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

The highlighted lines correspond to the wifi interface[^4]. On the second line you can find the configuration if this interface (i.e. the IPV4 (`inet`), the network mask (`netmask`) and the broadcast address (`broadcat`)). Here, only the IPV4 will be of use in the next.

Now, you will do pretty much the same things as for the server. Begin by making the `fastdds_conf.xml` file.

```shell title="Run this on the client"
curl https://fast-dds.docs.eprosima.com/en/latest/_downloads/9f9e92b14612364b742c8ecde24b3d24/super_client_configuration_file.xml > fast_dds.conf
```

But now you have to change the IP in this file. So instead of `127.0.0.1`, replace by the IP you got with the `ifconfig` previously.

```xml hl_lines="14 14"  title="Change this on the client"
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="super_client_profile" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                        <discoveryServersList>
                            <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                                <metatrafficUnicastLocatorList>
                                    <locator>
                                        <udpv4>
                                            <address>put the robot IP here</address>
                                            <port>11811</port>
                                        </udpv4>
                                    </locator>
                                </metatrafficUnicastLocatorList>
                            </RemoteServer>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

Then, you can add this lines to your .bashrc (or .zshrc if you use zsh) and change the IP once again with the robot IP here:

```shell title="Change this on the client"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path_to_your_file/fastdds_conf.xml
export ROS_DISCOVERY_SERVER=<put the robot ip here>:11811
```

Finally, to apply the changes, run the following commands:

```shell title="Run this on the client"
ros2 daemon stop
ros2 daemon start
```

[^1]: [EProsima FastDDS - Use ROS 2 with Fast-DDS Discovery Server](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html)
[^2]: [RoboFoundy on Medium - How to setup ROS2 Fast-DDS Discovery Server](https://robofoundry.medium.com/how-to-setup-ros2-fast-dds-discovery-server-3843c3a4adec)
[^3]: The name of the file can be anything, one was just decided for coherence through the page.
[^4]: Here the wifi interface's name is `wlp0s20f3`, but it can be anything else. However they will likely start with a "w", for "wireless".