"""FR3 Fake Hardware와 명시적으로 선택한 실물 backend의 공통 Python 제어 API.

Robot은 호출할 때 ROS 자원을 만들며, 모듈 import는 노드나 로봇 연결을 시작하지 않는다.
기본 backend는 fake다. real은 대상 robot_ip를 지정해야 하며 실제 동작을 요청한다.
"""

from .robot import Robot

__all__ = ['Robot']
