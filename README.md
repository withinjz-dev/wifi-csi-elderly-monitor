# WiFi CSI 기반 비접촉 독거노인 이상감지 시스템

제24회 임베디드 소프트웨어 경진대회 (자유공모 부문) 출품작.

카메라나 웨어러블 없이, WiFi CSI(Channel State Information)만으로 재실 여부와 호흡 유무를 판별해 수면 중 이상 상황을 감지하는 ESP32 standalone 시스템입니다.

전체 개발 배경·설계·검증 결과는 [개발완료보고서.txt](./개발완료보고서.txt)에 정리되어 있습니다.

## 구조

```
ESP32-CSI-Tool/          펌웨어 (오픈소스 ESP32-CSI-Tool 기반 + 자체 개발분)
  _components/
    edge_dsp.h            링버퍼, biquad, Goertzel, 자기상관 (자체 개발)
    edge_csi.h            CSI 취득, 서브캐리어 선택, 시간 빈 (자체 개발)
    edge_baseline.h        정지 baseline 적응형 학습 + NVS (자체 개발)
    edge_state.h            상태기계, 히스테리시스, FAULT/EMERGENCY 분리 (자체 개발)
    edge_monitor.h          판정 태스크 (자체 개발)
    alert_component.h       GPIO/LEDC 알림 제어 (자체 개발)
    input_component.h       시리얼 명령 파싱 (수정)
  active_ap/                RX(엣지 판정) 펌웨어. EDGE_MODE 스위치로 standalone/레거시 전환
  active_sta/                TX(송신) 펌웨어

data/                    호스트 개발/분석용 Python (자체 개발)
  live_monitor.py          호스트 레퍼런스 판정 구현
  validate_embedded.py      호스트-임베디드 교차검증
  validate_normalization.py  임계값 정규화 설계 검증
  calibrate.py              Otsu 기반 공간별 임계값 산출
  train_classifier.py        로지스틱 회귀 분류기 학습
  ...

개발완료보고서.txt        개발완료보고서 전문
```

## 라이선스 및 출처

CSI 수집 계층은 [ESP32-CSI-Tool](https://github.com/StevenMHernandez/ESP32-CSI-Tool) (Steven M. Hernandez, MIT License)을 기반으로 하며, 그 위에 엣지 신호처리·상태판정·적응형 보정·알림 시스템을 자체 개발했습니다.
