#!/usr/bin/env python3
"""설치된 데모 진입점이 ros2 run으로 검색되는지 검사한다.

ROS 노드나 로봇 연결을 만들지 않는다. Python 모듈 import만으로 발견하지
못하는 실행 권한 오류도 ros2 run과 동일한 검색 API로 검출한다.
"""

import sys

from ros2run.api import get_executable_path


def main():
    """빌드 후 실행한다. 모든 진입점 검색 성공 시 0, 누락 시 1을 반환한다."""
    expected = {
        'fr3_moveit_demo': (
            'moveit_demo', 'moveit_demo.py', 'basic_sequence.py', 'read_state.py',
        ),
        'fr3_offline_preview': ('joint_state_preview',),
    }
    missing = []
    for package, executables in expected.items():
        for executable in executables:
            if get_executable_path(package_name=package, executable_name=executable) is None:
                missing.append(f'{package}/{executable}')
    if missing:
        print('실행 파일 검색 실패: ' + ', '.join(missing), file=sys.stderr)
        print('Git 실행 권한과 설치 경로를 확인하고 scripts/build.sh를 다시 실행하세요.',
              file=sys.stderr)
        return 1
    print('INSTALLED EXECUTABLES PASS: C++/Python 데모 진입점을 ros2 run으로 검색했습니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
