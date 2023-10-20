# Learn Luxonis RAE from Zero

## Description
This repository aims to provide resources, tutorials, and code snippets for those who are interested in starting their journey with Luxonis RAE from the very basics. Whether you're a student, a developer, or an enthusiast, you'll find something useful here!

### Tutorial 1: Introduction to Luxonis RAE

#### Overview

The rae is a product developed by Luxonis, known for their OAK series of stereo depth cameras. The OAK devices are recognized in the field for their capability to aid in the development and deployment of AI-powered vision applications. They are noted for their affordability and ease of use, and they have garnered attention from an active open-source community. rae utilizes the RVC3 (Robotics Vision Core 3), the same technical specifications of the OAK 3 series. rae is designed to be cost-effective, easy to use, and offers customization options. The tagline "robotic access for everyone" encapsulates the product's intent to make robotic vision technology more accessible to a broader audience. For more information, please check their website and github link:

https://github.com/luxonis/rae-ros
https://docs.luxonis.com/projects/hardware/en/latest/pages/rae/

#### RobotHub

To start using rae, we can use RobotHub, the cloud platform prepared by Luxonis. By using RobotHub, we can connect our computer (or smartphone) with rae. Simply follow the instruction that available from their documentation. Basically, sign up to use the RobotHub service. Then put the WiFi information and scan the QR code shown on the page to connect with rae. In addition, we can also monitor and control the rae by using this platform.

![RobotHub Dashboard show 1 connected robot](https://i.imgur.com/x58RX4l.png)

### Tutorial 2: Controlling RAE

Based on the developer documentation, we can control rae by using 2 method. The first method is use the RobotHub, first we will use this method. After connecting the rae with RobotHub, now we can see the device connected. From the "Robot" tab from the left side dashboard, we can see the information about the connected robot like in the picture below.

![RobotHub Dashboard show 1 connected robot](https://i.imgur.com/jFqp8Fy.png)

From this page, select the "RAE - Default app" in the Percepation Apps box. Now, we can see the activity and the logs of the robot. By selecting the Local Frontend, we can control the robot and get the livefeed from its camera. 

![Display clicking local frontend button](https://i.imgur.com/my1jMZu.png)

Click the button to take control RAE.

![Controlling RAE's display](https://i.imgur.com/9QRS2TX.png)

We can use the joystick available on the screen to control your RAE. In addition, there are also 3 buttons on the top of the scree. Left button will change the control from joystick style to arrow style, and vice versa. The middle button allow you to change the display of lcd on your RAE. Finally, the right button you can modify the led color of your RAE.

### Tutorial 3: ROS2 in RAE

Based on the rae-ros github, we can implement ROS on RAE. In this step we are going to use docker, make sure to have docker installed in your machine. Download the image provided by the Luxonis by 

```bash
docker pull luxonis/rae-ros-robot:humble
```

Then, we are going to upload the image to our RAE. We can either using a USB cable or by connecting to its WiFi. We are going to use the first option as its more simple and faster. Connect the USB from computer to RAE and then we remotely accessing the command line of the robot's onboard computer using the SSH (Secure SHell) protocol. (Make sure you have 7-8 GB of free space in the /data directory on the robot.)

```bash
docker save luxonis/rae-ros-robot:humble | ssh -C root@192.168.197.55 docker load
```

Then run Docker Image on the Robot. SSH into the robot and execute the Docker image.

```bash
docker run -it --restart=unless-stopped -v /dev/:/dev/ -v /sys/:/sys/ --privileged  --net=host luxonis/rae-ros-robot:humble
```

At this step you run the image of the docker inside the RAE, and you have ROS2 pre-installed inside the images. try run ros2 command inside the images ex: ros2 node list

One of the example by running the provided script.

Testing the Encoder:
Run the following command to initiate the encoder testing process:

```bash
ros2 run rae_hw test_encoders
```

The output should be like this:

![Expected output](https://i.imgur.com/Gexg42M.png)

If you rotate the wheel, the value will change, its to make sure the motors are working.

### Some possible errors

![echo: write error: Device or resource busy](https://i.imgur.com/52ec3gs.png)

when running

```bash
docker run -it --restart=unless-stopped -v /dev/:/dev/ -v /sys/:/sys/ --privileged  --net=host luxonis/rae-ros-robot:humble
```

This is due to line 5 in entrypoint.sh

echo 2 > /sys/class/pwm/pwmchip0/export && echo 1 > /sys/class/pwm/pwmchip0/export && chmod -R a+rw /sys/class/pwm/pwmchip0/pwm1/ && chmod -R a+rw /sys/class/pwm/pwmchip0/pwm2/ && chmod -R a+rw /dev/gpiochip0

This mean pwmchip0 already exported, simply run:

```bash
echo 2 > /sys/class/pwm/pwmchip0/unexport && echo 1 > /sys/class/pwm/pwmchip0/unexport
```

and run the docker run command again.




![error requesting GPIO lines: Device or resource busy](https://i.imgur.com/gSZLL8v.png)

It possible when you run the "ros2 run" command, the script are using the GPIO pin, and conflicted to Device or resource busy. This problem due to the RAE in RobotHub running perception apps and make the GPIO conflicted. Shut down the perception apps in RobotHub and run the script again.

