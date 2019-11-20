# AWS RoboMaker Builder Session - Smart Edge


## Preparation

### Instructor Preparation

1. Create 10 environments and 10 IAM users, and share the environments with users.
2. Deploy RoboMaker-Common and CertGenerator
3. Setup WiFi connection for all robots.

 For each robot, print robot name, IP address on paper.
 1. laminate iam user name , robot name, IP, and leave the password field empty, so we can wipe and write on it before each session.


## Instructions

### Get Ready

1. Clone the repository from: https://github.com/mark-gu/aws-robomaker-smartedge.git

2. Navigate to "infrastructure\robomaker\\**cert_generator**", compress all files into a ZIP archive named **cert_generator.zip**

3. Physically connect to the Jetbot (via HDMI and USB cables), and connect it to the provided WiFi network. Attendees laptops must also join the same network.

4. Login to the AWS Management Console using the provided IAM username and password.

### Create AWS Resources

1. Navigate to **CloudFormation** and deploy [01-robomaker-common.yml](infrastructure/robomaker/templates/01-robomaker-common.yml).
    - Set *Stack name* to *RoboMaker-Common*
    - Save the output values for later use

    **Note**: This step may have already been completed by your instructor.

2. Navigate to **S3**, and click into the bucket deployed in the previous step.
    - Create a new directory named *utils*
    - Upload *cert_generator.zip* into the *utils* directory

    **Note**: This step may have already been completed by your instructor.

3. Navigate to **CloudFormation** and deploy [02-robomaker-cert_generator.yml](infrastructure/robomaker/templates/02-robomaker-cert_generator.yml).
    - Set *Stack name* to *RoboMaker-CertGenerator*
    - Set other parameters appropriately

    **Note**: This step may have already been completed by your instructor.

4. Navigate to **CloudFormation** and deploy [03-robomaker-fleet.yml](infrastructure/robomaker/templates/03-robomaker-fleet.yml).
    - Set *Stack name* to *RoboMaker-Fleet-Jetbots*
    - Set *FleetName* to a value of your choice

5. Navigate to **CloudFormation** and deploy [04-robomaker-robot.yml](infrastructure/robomaker/templates/04-robomaker-robot.yml).
    - Set *Stack name* to *RoboMaker-Robot-Jetbot01*
    - Set *Name* to a value of your choice
    - Set *Architecture* to *ARM64*
    - Set others parameters appropriately
    - After deployment, download all the files from the *Outputs* tab to a local directory, e.g. `/path/to/robot_files/`.

### Configure the Robot

1. Power up the robot and wait for it to boot up.

2. **[SparkFun Jetbot Only]** Copy [resources/90-i2c.rules](resources/90-i2c.rules) to `/path/to/robot_files/` also.

3. Copy all the required files onto the robot.

    ```bash
    # Copy all the downloaded files including certs and config files
    scp /path/to/robot_files/* jetbot@<ip-address>:/home/jetbot/
    ```

4. Configure AWS IoT Greengrass on the robot.

    ```bash
    # SSH to the robot
    ssh jetbot@<ip-address>

    # Switch to the root user
    sudo su -s /bin/bash

    # Copy Amazon Root CA1 and the robot's own certificate to Greengrass
    cp *.pem /greengrass/certs/

    # Copy the robot's own certificate private key to Greengrass
    cp *.private.key /greengrass/certs/

    # Copy the configuration file to Greengrass
    cp config.json /greengrass/config/

    # Keep the SSH connection alive and continue to the next step...
    ```

5. **[SparkFun Jetbot Only]** Update robot devices

    ```bash
    # Change the owner of the file
    sudo chown root:root 90-i2c.rules

    # Move the file to udev
    sudo cp 90-i2c.rules /etc/udev/rules.d

    # Reload device permissions
    sudo udevadm control --reload && sudo udevadm trigger

    # Verify the updated permissions are in effect
    ls -la /dev/i2c-1

    # You should see permissions like the following:
    # Ensure UID = ggc_user and GID = i2c
    #
    #              UID      GID
    #              ^^^      ^^^
    > crw-rw---- 1 ggc_user i2c 89, 1 Nov  8 16:16 /dev/i2c-1

    # Keep the SSH connection alive and continue to the next step...
    ```

6. Start Greengrass

    ```bash
    # Change the owner of the file
    sudo /greengrass/ggc/core/greengrassd start

    # Verify Greengrass is running
    ps aux | grep greengrass

    # Terminate the SSH connection
    exit # or Ctrl-d
    ```

### Build & Bundle the ROS Application

1. In the AWS Management Console, navigate to **RoboMaker**.

2. In the left pane, choose **Development environments**, and then choose **Create environment**.
   - Set *Name* to a value of your choice
   - Set *Pre-installed software suite* to *Melodic*
   - Set other fields appropriately

    **Note**: This step may have already been completed by your instructor.

3. Log into the **Cloud9 development environment** once it's ready.

4. Clone this repository in Cloud9.

    ```bash
    # Change to the environment directory
    cd ~/environment

    # Clone the project repository
    git clone https://github.com/mark-gu/aws-robomaker-smartedge.git SmartEdge
    ```

    **Note**: This step may have already been completed by your instructor.

5. Install application dependencies and build a cross-compilation container.

    ```bash
    # Change to the scripts directory
    cd ~/environment/SmartEdge/app/scripts

    # Add ROS dependencies
    sudo cp -a deps/* /etc/ros/rosdep/sources.list.d/
    echo "yaml file:///$(pwd)/jetbot.yaml" | sudo tee -a /etc/ros/rosdep/sources.list.d/21-customdepenencies.list > /dev/null

    # Log in to the an AWS ECR to enable your machine to pull a base Docker image
    $(aws ecr get-login --no-include-email --registry-ids 593875212637 --region us-east-1)

    # Install Ubuntu dependencies for cross compilation
    sudo apt update && sudo apt install -y qemu-user-static

    # Build a Docker container
    docker build -t jetbot-ros -f Dockerfile .

    # Fix ROS permissions
    rosdep fix-permissions
    sudo -u ubuntu rosdep update
    ```

    **Note**: This step may have already been completed by your instructor.

6. Cross-compile the application.

    ```bash
    # Change to the application directory
    cd ~/environment/SmartEdge/app

    # Run the Docker container
    docker run --rm -ti -v $(pwd):/environment/jetbot jetbot-ros

    # You will be dropped into the shell of the Docker container
    (docker)$ apt update
    (docker)$ rosdep fix-permissions && rosdep update

    (docker)$ cd robot_ws
    (docker)$ rosdep install --from-paths src --ignore-src -r -y
    (docker)$ colcon build --build-base arm64_build --install-base arm64_install
    (docker)$ colcon bundle --build-base arm64_build --install-base arm64_install --bundle-base arm64_bundle --apt-sources-list /opt/cross/apt-sources.yaml

    # Wait until shell script is completed...

    # Exit the container
    (docker)$ exit # or Ctrl-d
    ```

    **Note**: This step may have already been completed by your instructor.

7. Copy the bundled application to S3.

    ```bash
    # Make sure you exited out of the container in previous step

    # List all the S3 bucket names
    aws s3 ls

    # Copy the application to S3
    aws s3 cp ./robot_ws/arm64_bundle/output.tar s3://<bucket-name>/apps/detector_bot_<seq-no>.arm64.tar
    ```

    **Note**: This step may have already been completed by your instructor.

### Deploy the ROS Application

### Clean up

1. Remove certs, config
2. Delete all Robot and Fleet Stacks.
3. Delete files under S3 bucket/robots/ directory.
4. Deploy empty ROS app to bot???

## User Info Card

AWS Management Console: **https://aws-robots.signin.aws.amazon.com/console**

IAM user name:              **User1**  
IAM user password:          

Jetbot name:                **Jetbot1**  
Jetbot IP address:          