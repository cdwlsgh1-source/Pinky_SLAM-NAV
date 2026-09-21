## 📋 **zone_gate_client.py** 코드 해석
<img width="1472" height="840" alt="image" src="https://github.com/user-attachments/assets/6f6e8f69-9f85-441e-98b2-c8c5677100fb" />

## 📋 개요

| 항목 | 내용 |
| --- | --- |
| **제목** | `ZoneGateClient` — zone(구역) 진입 허가를 요청/보고하는 로봇용 통신 헬퍼 |
| **기능** | 로봇이 특정 구역에 들어가기 전에 "들어가도 돼?"라고 물어보고(`request_entry`), 허가(`grant_entry`)가 올 때까지 기다렸다가, 
구역을 나오면 "나왔어"라고 알려주는(`notify_exit`) 통신 담당 클래스 |

**비유**: 놀이공원의 좁은 구름다리 같은 곳. 한 번에 한 팀만 건널 수 있어서, 입구 관리자(`zone_manager_node`)에게 "저 지나가도 돼요?" 물어보고(`request_entry`), "네 가세요"(`grant_entry`) 하면 건너고, 다 건너면 "저 다 건넜어요"(`notify_exit`)라고 알려주는 구조입니다.

---

### 1단계: `import` 부분

```python
import time
import uuid
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
```

| 모듈 | 역할 |
| --- | --- |
| `time` | 시간 재기(타임아웃 체크, 대기) |
| `uuid` | 겹치지 않는 고유한 식별값(토큰) 생성 |
| `rclpy` | ROS2 파이썬 라이브러리 (노드, spin 등) |
| `QoSProfile` 등 | 통신 품질(QoS) 설정 |
| `String` | ROS2 기본 메시지 타입(문자열 하나 담는 그릇) |

#### 🔍 헷갈릴 만한 문법: `uuid.uuid4().hex[:8]`

기초 문법부터 순서대로 보면:

1. `uuid.uuid4()` → 무작위 고유 ID 하나 생성 (충돌 확률이 사실상 0)
2. `.hex` → 그걸 16진수 문자열로 바꿈 (예: `'3f9a1b2c...'`, 32자)
3. `[:8]` → 파이썬 **슬라이싱**. 앞에서 8글자만 자름

```python
>>> uuid.uuid4().hex
'3f9a1b2c7d4e4a9b8f1a2c3d4e5f6789'
>>> uuid.uuid4().hex[:8]
'3f9a1b2c'
```

**왜 이렇게?** → 매 요청마다 "이건 내가 방금 보낸 요청이야"를 구분할 고유 표(token)가 필요하기 때문. 이게 없으면 예전에 받았던 허가 응답과 지금 요청에 대한 응답을 구분 못 함 (반례는 아래 `_on_grant`에서 설명).

---

### 2단계: `__init__` — 생성자

```python
def __init__(self, node, robot_id, retry_period_sec=1.0):
    self.node = node
    self.robot_id = robot_id
    self.retry_period = retry_period_sec
    self._granted_token = None
```

| 코드 | 역할 |
| --- | --- |
| **`node`** | 이미 존재하는 ROS2 노드를 **빌려 쓴다** (새로 만들지 않음) |
| `robot_id` | "나는 누구인가"를 나타내는 문자열 (예: `'pinky1'`) |
| `retry_period_sec=1.0` | 기본값 1초 — 몇 초마다 재요청할지 |
| `self._granted_token = None` | 아직 허가 안 받은 상태로 초기화 |

#### 🔍 헷갈릴 만한 문법: `_granted_token`처럼 앞에 언더스코어(`_`)

파이썬에서 `_변수명`은 "이건 클래스 내부에서만 쓰는 변수야, 외부에서 건드리지 마"라는 **암묵적 약속**입니다 (강제는 아님). Java의 `private`과 비슷한 역할이지만 문법적으로 강제되진 않아요.

```python
qos = **QoSProfile**(depth=10,
                  reliability=ReliabilityPolicy.RELIABLE,
                  history=HistoryPolicy.KEEP_LAST)
```

| 파라미터 | 의미 |
| --- | --- |
| `depth=10` | 메시지를 최대 10개까지 버퍼에 쌓아둠 |
| `RELIABLE` | 메시지가 확실히 도착하도록 보장 (놓치면 재전송) |
| `KEEP_LAST` | 오래된 것부터 버리고 최신 10개만 유지 |

**왜 `RELIABLE`?** → "들어가도 돼?"라는 요청은 **놓치면 안 되는** 중요한 메시지이기 때문. 만약 `BEST_EFFORT`(빠르지만 유실 가능)를 썼다면, 요청이 씹혀서 로봇이 영원히 기다리는 문제가 생길 수 있어요.

```python
self.request_pub = node.**create_publisher**(String, '/zone_manager/**request_entry**', qos)
self.exit_pub = node.**create_publisher**(String, '/zone_manager/**notify_exit**', qos)
self.grant_sub = node.**create_subscription**(String, '/zone_manager/**grant_entry**', self.**_on_grant**, qos)
```

| 코드 | 역할 |
| --- | --- |
| **`request_pub`** | "들어갈래요" **발행자**(publisher) |
| **`exit_pub`** | "나왔어요" **발행자** |
| **`grant_sub`** | "허가함" 메시지를 **구독**(subscription) → 오면 **`_on_grant`** 함수 자동 호출 |

<aside>
💡

### '/zone_manager/이름' 경로가 뜻하는 의미는?

좋은 질문이에요! 저 문자열은 **ROS2 토픽(topic) 이름**입니다. 파일 경로가 아니라 "메시지가 흘러다니는 채널 이름"이에요.

---

### 📋 개요

| 항목 | 내용 |
| --- | --- |
| **제목** | ROS2 토픽 이름 — 메시지가 오가는 "방송 채널" 주소 |
| **기능** | 발행자(publisher)와 구독자(subscriber)가 같은 이름을 써야 서로 메시지를 주고받을 수 있음 |

---

#### 1. 비유로 먼저 이해하기

**토픽 이름 = 라디오 주파수(채널)**

```
request_pub.publish(msg)   →  '/zone_manager/**request_entry**' 채널로 방송
grant_sub                  →  '/zone_manager/**grant_entry**' 채널을 청취
```

- **`create_publisher**(타입, 토픽이름, qos)` → 이 채널로 **송신기**를 켬
- **`create_subscription**(타입, 토픽이름, 콜백, qos)` → 이 채널을 **수신기**로 듣고 있다가, 뭔가 오면 콜백 함수 실행

**핵심**: **같은 이름**(`'/zone_manager/grant_entry'`)을 쓰는 **publisher와 subscriber끼리만 서로 통신**됩니다. 이름이 한 글자라도 다르면 서로 못 알아듣습니다 (연결 자체가 안 됨, 에러도 안 남 — 그냥 조용히 통신 안 됨).

---

#### 2. 왜 앞에 `/`가 붙고 `/`로 계층이 나뉘어 있나?

```
/zone_manager/request_entry
 └──┬──────┘ └────┬──────┘
  네임스페이스      토픽 이름
  (namespace)
```

| 부분 | 역할 |
| --- | --- |
| 맨 앞 `/` | 절대 경로임을 표시 (파일 시스템의 루트 `/`와 비슷한 느낌) |
| `zone_manager` | **네임스페이스**(namespace) — 이 토픽이 어느 그룹/시스템 소속인지 |
| `request_entry` | 실제 토픽 이름 |

#### 🔍 헷갈릴 만한 개념: 네임스페이스가 왜 필요한가

만약 이렇게 짧게만 지었다면?

```python
node.create_publisher(String, 'request_entry', qos)
```

이 프로젝트에 나중에 `door_manager`, `elevator_manager` 같은 다른 관리자 노드가 생겼는데 걔도 우연히 `request_entry`라는 토픽을 쓴다면? → **이름 충돌**! zone 요청인지 door 요청인지 구분이 안 되고, 심지어 두 시스템이 서로의 메시지를 받아버리는 사고가 날 수 있어요.

그래서 `/zone_manager/` 라는 네임스페이스를 앞에 붙여서 "이건 zone_manager 전용 채널이야"라고 **소속을 명확히** 하는 거죠. 마치 파일 시스템에서 `/home/user1/note.txt`와 `/home/user2/note.txt`가 이름은 같아도(`note.txt`) 폴더가 달라서 안 섞이는 것과 같은 원리입니다.

---

#### 3. 이 코드에서 쓰인 3개 토픽 정리

| 토픽 이름 | 타입 | 발행 주체 | 구독 주체 | 의미 |
| --- | --- | --- | --- | --- |
| `/zone_manager/**request_entry**` | `String` | 로봇 (`ZoneGateClient`) | **`zone_manager_node`** | "들어가도 돼요?" |
| `/zone_manager/**grant_entry**` | `String` | `zone_manager_node` | 로봇 (**`ZoneGateClient`**) | "허가함" |
| `/zone_manager/**notify_exit**` | `String` | 로봇 (`ZoneGateClient`) | **`zone_manager_node`** | "나왔어요" |

**흐름도**로 보면:

```
[**ZoneGateClient**]                    [**zone_manager_node**]
      │                                     │
      │──/zone_manager/**request_entry**───────▶│   (요청)
      │                                     │
      │◀──/zone_manager/**grant_entry**─────────│   (허가)
      │                                     │
      │──/zone_manager/**notify_exit**─────────▶│   (이탈 통보)
```

주목할 점: 화살표 방향이 다 다르죠? `request_entry`와 `notify_exit`은 로봇→매니저 방향, `grant_entry`는 매니저→로봇 방향입니다. **한 토픽은 한 방향으로만 흐른다**는 게 토픽 통신의 특징이에요 (양방향이 필요하면 이렇게 토픽 2개를 따로 만들어야 함 — 이게 바로 지난번에 궁금해하셨던 "왜 Service가 아니라 Topic 2개를 썼을까"의 힌트이기도 해요).

---

#### 4. 파일 경로랑 헷갈리지 않는 팁

|  | 파일 경로 | ROS2 토픽 이름 |
| --- | --- | --- |
| 생김새 | `/home/user/file.txt` | `/zone_manager/request_entry` |
| 실체 | 디스크에 저장된 파일 | 그냥 **문자열 식별자** (아무것도 저장 안 됨) |
| 확인 방법 | `ls /home/user/` | 터미널에서 `ros2 topic list` |

실제로 코드가 돌아가는 중에 터미널에서 아래 명령어를 치면 지금 이 토픽들이 실시간으로 보입니다:

```bash
ros2 topic list
ros2 topic echo /zone_manager/grant_entry   # 실시간으로 메시지 내용 훔쳐보기
```

</aside>

**흐름도**:

```
**grant_entry** 토픽에 메시지 도착
        ↓
ROS2가 자동으로 self.**_on_grant**(msg) 호출
        ↓
msg 안의 내용을 보고 self.**_granted_token** 갱신
```

---

### 3단계: `_on_grant` — 콜백 함수

```python
def _on_grant(self, msg: String):
    try:
        robot_id, token = msg.data.split(':', 1)
    except ValueError:
        return
    if robot_id == self.robot_id:
        self._granted_token = token
```

| 줄 | 설명 |
| --- | --- |
| `msg.data.split(':', 1)` | 문자열을 `:` 기준으로 **최대 1번만** 잘라서 2개로 나눔 |
| `robot_id, token = ...` | 파이썬 **언패킹**: 나뉜 결과 2개를 변수 2개에 한 번에 할당 |
| `try/except ValueError` | 혹시 `:`가 없어서 나뉜 개수가 안 맞으면 조용히 무시 |
| `if robot_id == self.robot_id` | **내 것이 아닌** 허가 메시지는 무시 (다른 로봇 거일 수 있음) |

#### 🔍 헷갈릴 만한 문법: `split(':', 1)`의 `1`

```python
>>> 'pinky1:3f9a1b2c'.split(':')      # 최대 개수 제한 없음
['pinky1', '3f9a1b2c']
>>> 'pinky1:3f9a1b2c'.split(':', 1)   # 최대 1번만 자름
['pinky1', '3f9a1b2c']
```

지금 예시는 결과가 같지만, 만약 `robot_id`나 토큰 자체에 `:`가 섞여 있으면 결과가 달라집니다. `1`을 안 넣으면 예상보다 더 잘게 쪼개져서 언패킹(`robot_id, token = ...`)에서 에러가 날 수 있어요. **안전장치**인 셈이죠.

**여기서 `token`이 왜 필요한지** (앞서 `uuid`에서 예고했던 부분):

> 만약 토큰 없이 `robot_id`만 비교했다면? → 로봇이 요청을 여러 번 재전송했는데, 그중 "옛날" 허가 메시지가 뒤늦게 도착해도 무조건 `True`로 착각할 수 있음. 토큰으로 "이건 방금 그 요청에 대한 응답이 맞다"를 확인하는 것.
> 

---

### 4단계: `wait_for_entry` — 핵심 로직

```python
def wait_for_entry(self, timeout_sec=None, stop_check=None):
    token = uuid.uuid4().hex[:8]
    self._granted_token = None
    start = time.time()

    while rclpy.ok():
        if stop_check is not None and stop_check():
            return False

        msg = String()
        msg.data = f'{self.robot_id}:{token}'
        self.request_pub.publish(msg)

        end_wait = time.time() + self.retry_period
        while time.time() < end_wait:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._granted_token == token:
                self.node.get_logger().info(f'[{self.robot_id}] zone 진입 허가 받음')
                return True
            if stop_check is not None and stop_check():
                return False
            if timeout_sec is not None and (time.time() - start) > timeout_sec:
                return False

    return False
```

#### 전체 흐름도

```
① token 생성 (uuid)
② while rclpy.ok():             ← ROS2가 살아있는 동안 반복
③   stop_check() 확인            → True면 즉시 False 리턴
④   "robot_id:token" 메시지 발행  → request_entry 토픽으로 전송
⑤   최대 retry_period(1초)초 동안:
⑥     spin_once() 실행            → 이 짧은 순간에 콜백(_on_grant)이 호출될 기회를 줌
⑦     허가 토큰 일치? → True 리턴 (끝!)
⑧     중단 요청? → False 리턴
⑨     타임아웃? → False 리턴
⑩   (1초 지나면) ②로 돌아가서 다시 요청 발행 (재시도)
```

#### 🔍 헷갈릴 만한 문법 하나씩

**(1) `stop_check` — 함수를 인자로 넘기기 (콜백 패턴)**

기초 예시부터:

```python
def greet():
    return "hi"

def caller(func):   # 함수 자체를 매개변수로 받음
    print(func())    # 나중에 호출

caller(greet)   # "hi" 출력
```

지금 코드에서 `stop_check`는 "지금 멈춰야 하나요?"를 물어보는 함수 그 자체입니다. 클래스 문서에 있던 예시처럼:

```python
self.gate.wait_for_entry(stop_check=lambda: self._stop_requested)
```

**(2) `lambda`**

```python
lambda: self._stop_requested
```

= `def temp(): return self._stop_requested` 를 한 줄로 줄인 것. "인자 없이 호출하면 `self._stop_requested` 값을 리턴하는 익명 함수"입니다.

**(3) `rclpy.spin_once(self.node, timeout_sec=0.1)`**

이게 이 코드의 **핵심 트릭**입니다.

> ROS2에서 구독(subscription) 콜백은 보통 `rclpy.spin()`을 돌려야 실행됩니다. 그런데 여기서는 `wait_for_entry` 함수 자체가 `while` 루프를 돌며 **블로킹**(멈춰서 기다림)하고 있어요. 이 안에서 직접 `spin_once()`를 짧게(0.1초) 호출해줘서 "잠깐만 밀린 콜백들 처리해줘"라고 수동으로 시켜주는 것.
> 

**왜 이렇게?** → 만약 `spin_once()`를 안 부르면, `_on_grant` 콜백이 절대 실행되지 않아서 `self._granted_token`이 영원히 갱신 안 됨 → 영원히 대기. **반례**로 설명하면: `spin_once()` 없이 그냥 `while: time.sleep(0.1)`만 돌렸다면 zone_manager가 아무리 허가를 보내줘도 이 로봇은 절대 눈치채지 못합니다.

**(4) 이중 `while` 구조를 쓴 이유**

| 바깥 `while` | 안쪽 `while` |
| --- | --- |
| "요청을 재전송할지" 결정 | "1초 동안 응답 오는지 감시" |
| 1초마다 한 번 실행 | 0.1초 간격으로 여러 번 실행 |

**대안**: 요청을 딱 한 번만 보내고 무한정 기다릴 수도 있었겠죠. 하지만 그러면 **메시지가 유실된 경우**(예: 네트워크 순간 문제) 영원히 응답을 못 받게 됩니다. 그래서 1초마다 "혹시 놓쳤을까봐 다시 보낼게요"하고 **재요청**하는 구조를 택한 것.

---

### 5단계: `notify_exit`

```python
def notify_exit(self):
    msg = String()
    msg.data = self.robot_id
    self.exit_pub.publish(msg)
    self.node.get_logger().info(f'[{self.robot_id}] zone 이탈 통보 전송')
```

| 코드 | 역할 |
| --- | --- |
| `msg.data = self.robot_id` | 토큰 없이 그냥 `robot_id`만 보냄 |
| `self.exit_pub.publish(msg)` | 발행 → 응답을 기다리지 않고 바로 끝남 (Fire-and-forget) |

**`wait_for_entry`와 비교**:

|  | `wait_for_entry` | `notify_exit` |
| --- | --- | --- |
| 응답 대기 여부 | O (허가 올 때까지 블로킹) | X (보내고 바로 끝) |
| 메시지 형식 | `robot_id:token` | `robot_id`만 |
| 재시도 | O (1초마다) | X |

**왜 `notify_exit`은 응답을 안 기다릴까?** → "나갔다"는 사실은 로봇 입장에서 뒤집힐 일이 없는 확정된 사실이라, 상대의 허락이 필요 없기 때문. 반면 "들어가도 되나요"는 **허락이 전제조건**이라 반드시 응답을 기다려야 하죠.

---

### 📌 요약 정리

| 메서드 | 언제 부름 | 하는 일 |
| --- | --- | --- |
| `wait_for_entry()` | zone 진입 직전 | 허가받을 때까지 요청 반복 + 대기 (블로킹) |
| `notify_exit()` | zone 탈출 직후 | 이탈 통보만 하고 끝 (논블로킹) |
| `_on_grant()` | (자동 호출) | 허가 메시지 오면 토큰 저장 |
