"""FR3 GenericSystem Fake Hardware를 위한 공통 Python 제어 API.

Robot은 호출할 때 ROS 자원을 만들며, 모듈 import는 노드나 로봇 연결을 시작하지 않는다.
"""

from .robot import Robot

__all__ = ['Robot']
