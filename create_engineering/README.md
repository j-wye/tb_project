# During Creative Engineering
## USAR Contest with using Turtlebot
- Mission : Turtlebot이 미지의 Map을 자율주행하며 qr code를 인식하고 주어진 갯수만큼 인식했다면 다시 원래 자리로 되돌아와서 정지

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
        
        - 3. Twist, Marker, Laserscan, PoseWithCovarianceStamped : ROS2 msg type