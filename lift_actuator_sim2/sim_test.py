import pybullet as p
import pybullet_data
import time

# PyBullet 연결 및 환경 설정
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 사용자 정의 바닥(plane) 로드 코드가 들어갈 자리
# (예: planeId = p.loadURDF("plane.urdf"))

planeId = p.loadURDF("plane.urdf")

# 1. 휠체어 URDF 불러오기
startPos = [0, 0, 0.1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
wheelchairId = p.loadURDF("lift_actuator_sim2/URDF/wheelchair.urdf", startPos, startOrientation)

# 2. URDF에 등록된 바퀴 조인트 인덱스 확인 및 지정 (URDF 순서에 따름)
# 보통 조인트 이름을 기반으로 인덱스를 찾습니다.
num_joints = p.getNumJoints(wheelchairId)
left_wheel_joint = 0   # 본인 URDF에 맞는 인덱스로 수정
right_wheel_joint = 1  # 본인 URDF에 맞는 인덱스로 수정

# 3. 주행 속도 설정 (라디안/초 단위)
target_velocity = 5.0  # 앞으로 전진할 속도

# 시뮬레이션 루프
for i in range(10000):
    # 왼쪽 바퀴 모터 제어
    p.setJointMotorControl2(bodyUniqueId=wheelchairId,
                            jointIndex=left_wheel_joint,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=target_velocity,
                            force=10) # 모터 힘(토크)
    
    # 오른쪽 바퀴 모터 제어
    p.setJointMotorControl2(bodyUniqueId=wheelchairId,
                            jointIndex=right_wheel_joint,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=target_velocity,
                            force=10)
    
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
