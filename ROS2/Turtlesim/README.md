# Turtlesim

The Turtlesim is a simulator with learning purpose on ROS2. It uses a simple representation to teach the uses and interactions of nodes, topics,serices, parameters and actions on ROS. 

## Installing
To install Turtlesim on humble: 

    sudo apt update

    sudo apt install ros-humble-turtlesim

To check if it is installed:

    ros2 pkg executables turtlesim

## Starting Turtlesim
To start Turtlesim:

    ros2 run turtlesim turtlesim_node

If everything is installed correctly, a window will show up:

window

## Using Turtlesim
Open a new terminal and source ROS 2

Now you will run a new node to control the turtle in the first node:

    ros2 run turtlesim turtle_teleop_key

At this moment you must have two terminals, one running the turtlesim and the other running the node that controls the turtle on the turtlesim. With both the node terminal and the turtlesim window on the screen, you can use the arrow keys on the teleop terminal to move and rotate the turtle around and see its path.

You can see the list of nodes and their associated topics, services, and actions, using the list subcommands of the respective commands:

    ros2 node list
    ros2 topic list
    ros2 service list
    ros2 action list

We will use each one of these to learn about each aspect of theses concepts.


## ros2 run

The command ros2 run launches an executable from a package.

    ros2 run <package_name> <executable_name>

To run turtlesim, open a new terminal, and enter the following command:

    ros2 run turtlesim turtlesim_node

The turtlesim window will open, as you saw in the previous tutorial.

Here, the package name is turtlesim and the executable name is turtlesim_node.


# Nodes
Each node is a structure responsible for a single, modular action. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

To find all the names of all running nodes, there is the command

    ros2 node list

If you open a new terminal while turtlesim is still running in the other one and enter ros2 node list, the terminal will return the node name:

    /turtlesim

Open another new terminal and start the teleop node with the command:

    ros2 run turtlesim turtle_teleop_key

Here, we are referring to the turtlesim package again, but this time we target the executable named turtle_teleop_key.

Return to the terminal where you ran ros2 node list and run it again. You will now see the names of two active nodes:

    /turtlesim
    /teleop_turtle

## Node Remapping

Remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values. In the last tutorial, you used remapping on turtle_teleop_key to change the cmd_vel topic and target turtle2.

Now, let’s reassign the name of our /turtlesim node. In a new terminal, run the following command:

    ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

Since you’re calling ros2 run on turtlesim again, another turtlesim window will open. However, now if you return to the terminal where you ran ros2 node list, and run it again, you will see three node names:

    /my_turtle
    /turtlesim
    /teleop_turtle

# Topic

ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.

![Example of two nodes communicating with topics](/imgs/topic_intro1.gif "topic communication example")

Topics don't need to only be one-to-one communication, it can be one-to-many, many-to-one or many-to-many communication. 

A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.

![Example of three nodes communicating with topics](/imgs/topic_intro2.gif "topic multiple communication example")


Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system.

## Topic List

Now, with one terminal running the turtlesim and other running the turtle_teleop_key,you can run the ros2 topic list command in a new terminal to show all the currently active topics in the system:

    /parameter_events
    /rosout
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose

ros2 topic list -t will return the same list of topics, this time with the topic type appended in brackets:

    /parameter_events [rcl_interfaces/msg/ParameterEvent]
    /rosout [rcl_interfaces/msg/Log]
    /turtle1/cmd_vel [geometry_msgs/msg/Twist]
    /turtle1/color_sensor [turtlesim/msg/Color]
    /turtle1/pose [turtlesim/msg/Pose]

## Topic Echo

To see the data being published on a topic, use:

    ros2 topic echo <topic_name>

To change the velocity of the turtle on the turtlesim, the topic used is /turtle1/cmd_vel. We can see the information of this topic with the command:

    ros2 topic echo /turtle1/cmd_vel 

At first, this command won’t return any data. That’s because it’s waiting for /teleop_turtle to publish something.

Return to the terminal where turtle_teleop_key is running and use the arrows to move the turtle around. Watch the terminal where your echo is running at the same time, and you’ll see position data being published for every movement you make:

    linear:
    x: 2.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0
    ---

## Topic info

Topics don’t have to only be one-to-one communication; they can be one-to-many, many-to-one, or many-to-many.

Another way to look at this is running:

    ros2 topic info /turtle1/cmd_vel

Which will return:

    Type: geometry_msgs/msg/Twist
    Publisher count: 1
    Subscription count: 2

As you can see, this topic have one publisher and two subscriptions, wich means it receives a message from one publisher and sends it to two subscribers.

## Topic Insterface Show

Nodes send data over topics using messages. Publishers and subscribers must send and receive the same type of message to communicate.

The topic types we saw earlier after running ros2 topic list -t let us know what message type is used on each topic. Recall that the cmd_vel topic has the type:

    geometry_msgs/msg/Twist

This means that in the package geometry_msgs there is a msg called Twist.

Now we can run ros2 interface show <msg type> on this type to learn its details. Specifically, what structure of data the message expects.

    ros2 interface show geometry_msgs/msg/Twist

For the message type from above it yields:

This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
    Vector3  angular

This tells you that the /turtlesim node is expecting a message with two vectors, linear and angular, of three elements each. If you recall the data we saw /teleop_turtle passing to /turtlesim with the echo command, it’s in the same structure:

    linear:
    x: 2.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0
    ---

## Topic Pub

Now that you have the message structure, you can publish data onto a topic directly from the command line using:

    ros2 topic pub <topic_name> <msg_type> '<args>'

The '<args>' argument is the actual data you’ll pass to the topic, in the structure you just discovered in the previous section.

It’s important to note that this argument needs to be input in YAML syntax. Input the full command like so:

    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

--once is an optional argument meaning “publish one message then exit”.

You will see the following output in the terminal:

    publisher: beginning loop
    publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))

The turtle (and commonly the real robots which it is meant to emulate) require a steady stream of commands to operate continuously. So, to get the turtle to keep moving, you can run:

    ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

The difference here is the removal of the --once option and the addition of the --rate 1 option, which tells ros2 topic pub to publish the command in a steady stream at 1 Hz.

And you will see your turtle move in a circular pattern:

![Turtlesim simulator with a circular motion](/imgs/turtlesim_circle.png "simulator doing a circular motion")