# AWS RoboMaker Builder Session - Smart Edge

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

2. Navigate to **S3**, and click into the bucket deployed in the previous step.
    - Create a new directory named *utils*
    - Upload *cert_generator.zip* into the *utils* directory

3. Navigate to **CloudFormation** and deploy [02-robomaker-cert_generator.yml](infrastructure/robomaker/templates/02-robomaker-cert_generator.yml).
    - Set *Stack name* to *RoboMaker-CertGenerator*
    - Set other parameters appropriately

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

2. Copy [resources/90-i2c.rules](resources/90-i2c.rules) to `/path/to/robot_files/` also.

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

5. Update robot devices

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

3. Log into the **Cloud9 development environment** once it's ready.

4. Clone this repository in Cloud9.

    ```bash
    # Change to the environment directory
    cd ~/environment

    # Clone the project repository
    git clone https://github.com/mark-gu/aws-robomaker-smartedge.git SmartEdge
    ```

5. Install application dependencies and build a cross-compilation container.

    ```bash
    # Change to the scripts directory
    cd ~/environment/SmartEdge/app/assets/scripts

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

7. Copy the bundled application to S3.

    ```bash
    # Make sure you exited out of the container in previous step

    # List all the S3 bucket names
    aws s3 ls

    # Copy the application to S3
    aws s3 cp ./robot_ws/arm64_bundle/output.tar s3://<bucket-name>/apps/detector_bot_<seq-no>.arm64.tar
    ```

### Deploy the Machine Learning Model

1. Zip the model and upload it to S3.

    ```bash
    cd ~/environment/models
    zip ../models.zip *

    aws s3 cp ~environment/models.zip s3://<bucket-name>/models.zip
    ```

2. Navigate to **Lambda**, and click on **Create function**.
    - Set *Function name* to a value of your choice
    - Set *Runtime* to *Python 2.7*
    - Create a new role, or using an existing one
    - After the new function is created, copy the content of the file [infrastructure/robomaker/model_sync/index.py](infrastructure/robomaker/model_sync/index.py) into the Function code area.
    - Save and publish a new version

3. Navigate to **IoT Greengarss**.

4. In the left pane, click on **Greengrass**, then **Groups**, then the group for your robot.

5. In the left pane of the Greengrass Group view, click on **Lambdas**, then **Add your first Lambda**, then **Use existing Lambda**.

6. Once the Lambda function is added, select **Edit configuration** in the upper right corner of the Lambda function.
    - Under *Lambda lifecycle*, select *Make this function long-lived and keep it running indefinitely*

7. In the Greengrass > Groups, click on **Resources**

8. Click on **Machine Learning**, then **Add a machine learning resource**
    - Set Resource name to a value of your choice
    - Set Model Source to *Upload a model in S3 (including models optimized through Deep Learning Compiler)*
    - Locate the models.zip in the S3 bucket
    - Set *Local path* to */trained_models*
    - Set *Identify resource owner and set access permissions* to *No OS group*
    - Link the Lambda function and the resource, and set permission to *Read and write access*

9. In the Greengrass > Groups, click on **Resources**.

10. Click on **Local**, then **Add local resource**.
    - Set Resource name to a value of your choice
    - Set Resource type to *Volume*
    - Set Source path to */tmp*
    - Set Destination path to */tmp*
    - Set Group owner file access permission to *Automatically add OS - group permissions of the Linux group that owns the resource*
    - Link the Lambda function and the resource, and set permission to *Read and write access*

11. Navigate to **IAM** and allow Greengrass_ServiceRole to access S3.

12. Navigate to **IoT Greengrass**, then **Groups**. Click on **Actions** > **Deploy**. After the deployment completes successfully, you should be able to see /tmp/trained_models direction on the robot.

### Deploy the ROS Application

1. In the AWS Management Console, navigate to **RoboMaker**.

2. In the left pane, choose **Robot Applications**, and then choose **Create application**.

3. In the left page, choose **Deployments**, and then choose **Create deployment**.

4. If the deployment is successful, the robot will spin for 10 seconds and stop.

### Clean up

1. Remove certs, config
2. Delete all Robot and Fleet Stacks.
3. Delete files under S3 bucket/robots/ directory.

## User Info Card

AWS Management Console: **https://aws-robots.signin.aws.amazon.com/console**

Jetbot name:                **Jetbot1**  
Jetbot IP address:          