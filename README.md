# LiDAR Tape Correction Package

고반사 테이프로 인한 라이다 거리 측정 오차를 보정하는 ROS2 패키지입니다.

## 빌드 방법

```bash
cd ~/catkin_tapeDetector
colcon build --packages-select lidar_tape_correction
source install/setup.bash
```

## 사용 방법

### 1. 캘리브레이션 단계

캘리브레이션 노드를 실행하여 반사도-거리 오차 관계를 학습합니다:

```bash
ros2 launch lidar_tape_correction calibration.launch.py
```

#### 캘리브레이션 절차:

1. 고반사 테이프를 정확히 측정된 거리에 배치
2. 실제 거리 입력:
   ```bash
   ros2 topic pub /actual_distance std_msgs/msg/Float32 "data: 1.5"  # 1.5m 예시
   ```
3. 캘리브레이션 포인트 수집:
   ```bash
   ros2 service call /collect_calibration_point std_srvs/srv/Trigger
   ```
4. 여러 거리(예: 0.5m, 1.0m, 1.5m, 2.0m, 2.5m)에서 2-3단계 반복
5. 모델 학습:
   ```bash
   ros2 service call /fit_calibration_model std_srvs/srv/Trigger
   ```
6. 캘리브레이션 저장:
   ```bash
   ros2 service call /save_calibration std_srvs/srv/Trigger
   ```

### 2. 보정 적용 단계

캘리브레이션이 완료되면 보정 노드를 실행합니다:

```bash
ros2 launch lidar_tape_correction correction.launch.py
```

보정된 스캔 데이터는 `/corrected_laser_scan` 토픽으로 퍼블리시됩니다.

## 파라미터

### calibration_node
- `scan_topic`: 입력 LaserScan 토픽 (기본: `/bot_sensor/lidar_front/laser_scan`)
- `calibration_file`: 캘리브레이션 파일 경로 (기본: `config/lidar_calibration.txt`)
- `intensity_threshold`: 고반사로 판단할 최소 반사도 값 (기본: 200.0)
- `collection_window_size`: 평균 계산에 사용할 샘플 수 (기본: 10)
- `high_intensity_angle_range`: 고반사 포인트 검색 각도 범위 [도] (기본: 5.0)

### correction_node
- `scan_topic`: 입력 LaserScan 토픽
- `corrected_scan_topic`: 출력 LaserScan 토픽 (기본: `/corrected_laser_scan`)
- `calibration_file`: 캘리브레이션 파일 경로
- `publish_markers`: 시각화 마커 퍼블리시 여부 (기본: true)
- `intensity_threshold`: 보정을 적용할 최소 반사도 값 (기본: 200.0)

## 시각화

RViz에서 다음 토픽들을 추가하여 보정 효과를 확인할 수 있습니다:
- `/correction_markers`: 원본(빨강)과 보정된(초록) 포인트 표시
- `/high_intensity_point`: 캘리브레이션 중 감지된 고반사 포인트

## 알고리즘 설명

1. **문제**: 고반사 테이프는 실제 거리보다 가까운 값을 반환
2. **해결**: 반사도와 거리 오차 간의 관계를 2차 다항식으로 모델링
3. **보정**: `corrected_distance = measured_distance + f(intensity)`
   - f(intensity) = a₀ + a₁×intensity + a₂×intensity²

## 주의사항

- 캘리브레이션 시 테이프까지의 실제 거리를 정확히 측정해야 합니다
- 다양한 거리에서 충분한 캘리브레이션 데이터를 수집하세요 (최소 3개 이상)
- 환경 조건(조명, 온도 등)이 크게 변하면 재캘리브레이션이 필요할 수 있습니다