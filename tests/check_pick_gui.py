#!/usr/bin/env python3
"""실제 CV 창/작업 스레드로 가상 물체를 선택한다. 클릭 이벤트만 코드로 주입한다.

FR3_PICK_GUI=main bash scripts/test_pick.sh로 실행한다. Fake 서버와 GUI 세션이 필요하다.
운영체제 마우스 조작이나 실제 카메라/물체 집기를 검증하는 테스트는 아니다.
"""

import json
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

import cv2

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'ros2_ws/src/fr3_moveit_demo/examples'))
import pick_and_return as example


def main():
    """가상 CV 실행 진입점에 중앙 클릭을 주입하고 작업 보고서/시작 자세 복귀를 확인한다."""
    callback = []
    original_callback, original_wait = cv2.setMouseCallback, cv2.waitKey

    def register(window, handler):
        callback[:] = [handler]
        original_callback(window, handler)

    def pump(delay):
        result = original_wait(delay)
        if callback:
            callback[0](cv2.EVENT_LBUTTONDOWN, 320, 240, 0, None)
        return result

    with tempfile.TemporaryDirectory(prefix='fr3-gui-pick-') as directory:
        report = Path(directory) / 'report.json'
        argv = ['pick_and_return.py', '--demo', '--execute', '--duration', '1',
                '--hold', '0.1', '--report', str(report)]
        with patch.object(sys, 'argv', argv), patch.object(cv2, 'setMouseCallback', register), \
                patch.object(cv2, 'waitKey', pump):
            assert example.main() == 0
        data = json.loads(report.read_text(encoding='utf-8'))
        assert data['backend'] == 'fake' and data['stages'][-1]['stage'] == 'returned'
        assert len(data['stages']) == 8 and not data['physical_grasp_verified']
    print('PICK GUI PASS: actual CV window, injected callback, single worker cycle and report. No FCI.')


if __name__ == '__main__':
    main()
