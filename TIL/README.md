# TIL (Today I Learned)

이 저장소는 우주선 자세 제어(Attitude Control) 역학과 파이썬 비행 소프트웨어 아키텍처 학습 기록을 담고 있습니다.

---

### 📅 2026년 7월 (July 2026)

<details>
<summary><b>2026-07-01: CMG 구조의 이해와 EOM 유도</b></summary>
<br>

- **핵심 개념:** Control Moment Gyroscope의 구동 원리 및 김벌(Gimbal) 역학
- **세부 내용:** - 각운동량 보존 법칙을 활용한 우주선 회전 운동 방정식(EOM) 유도
  - 평행축 정리(Parallel Axis Theorem) 수학적 모델링
- 🔗 [상세 문서 보러가기](./2026-07-01-CMG-EOM.md)

</details>

<details>
<summary><b>2026-07-03: 방향 코사인 행렬(DCM)과 오일러 각 변환</b></summary>
<br>

- **핵심 개념:** 파이썬 객체 지향 프로그래밍을 활용한 우주선 `Attitude` 클래스 설계
- **세부 내용:**
  - 부동소수점 오차 방지를 위한 `max()` 및 `np.sqrt()` 클리핑(Clipping) 기법
  - 팩토리 메서드(`@classmethod`)를 활용한 아키텍처 설계
  - `tilde_matrix` 등 수학 유틸리티 함수의 모듈화(Packaging) 기법
- 🔗 [상세 문서 보러가기](./2026-07-03-Attitude-Class.md)

</details>
