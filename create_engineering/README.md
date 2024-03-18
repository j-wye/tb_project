# During Creative Engineering
## USAR Contest with using Turtlebot
- Mission : In an ***Unknown Environment***, Turtlebot has to drive autonomously until it recognizes a set number of *QR codes*.

- Concepts I used to solve the mission :
    - 1. Detect QR Code
        - Using pyzbar package

    - 2. ROS2 Communication
        - 1. CvBridge
            - Converting between ROS images and OpenCV images
            - imgmsg_to_cv2 : ros이미지를 cv2이미지로 변환
        
        - 2. rclpy.qos
            - Quality of Service change follows :

                ```
                History : 데이터를 몇 개나 보관할 것인지
                Reliability : 데이터 전송에서 속도를 우선하는지 신뢰성을 우선하는지
                Durability : 데이터를 수신하는 Subscribe가 생성되기 전의 데이터를 사용할 것인지
                Deadline : 정해진 주기 안에 데이터가 발신, 수신되지 않을 경우
                Lifespan : 정해진 주기 안에서 수신되는 데이터만 유효 판정하고, 그렇지 않은 데이터는 삭제
                Liveliness : 정해진 주기 안에서 노드 혹은 토픽의 생사를 확인
                ```
        
        - 3. create_publisher / create_subscription :
            - Use various ROS2 msgs (e.g. Twist, Marker, Laserscan, PoseWithCovarianceStamped)
            - subscribe and publish necessary informations
        
        - 4. Calculate position :
            - Firstly, just calculate distance of detected qr code
                - Use 2d Lidar for obtain depth value (Because, camera is monocular)
            - Secondly, calculate exact position of detected qr code
                - Use Quaternion from current position and orientation and calculated distance to qr code
        
        - 5. Display marker point at exact position of detected qr code on RVIZ

        - 6. If completes all given missions, returns to its starting point and stops working.

- Disappointing at Control Logic part...