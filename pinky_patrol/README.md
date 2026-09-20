<aside>
<img src="/icons/clipping_gray.svg" alt="/icons/clipping_gray.svg" width="40px" />

## A. 코드해석

## 코드 해석: `patrol_client.py` (초보자용 완전 쉬운 버전)

#### 1. 제목 / 기능

| 항목 | 내용 |
| --- | --- |
| **제목** | 터미널에서 로봇에게 "가라(start)/멈춰라(stop)" 명령을 보내거나, 로봇 상태를 계속 지켜보는(monitor) 프로그램 |
| **기능** | 실행할 때 로봇 이름과 명령어 2개를 같이 적어주면, 그에 맞는 동작을 함 |

---

### 2. 전체를 관통하는 비유

이 프로그램은 **"TV 리모컨"**입니다.

- `send_command('start')` → 리모컨 전원 버튼 누르기 (누르고 끝)
- `monitor` → TV 화면을 계속 쳐다보고 있기 (Ctrl+C 눌러야 그만 봄)
- `status_callback` → TV에서 뭔가 화면에 바뀔 때마다 자동으로 알아채는 것

---

### 3. 조각별로 아주 천천히

#### 1️⃣ 맨 위 4줄 — 도구 가져오기

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time
```

| 줄 | 쉬운 말로 |
| --- | --- |
| `import rclpy` | "ROS2 기능 쓸 거야" 선언 |
| `from rclpy.node import Node` | "Node라는 틀(설계도)을 가져올게" |
| `from std_msgs.msg import String` | "문자열 메시지 상자(String)를 가져올게" |
| `import sys` | "터미널에서 입력한 값들 읽을 도구" |
| `import time` | "몇 초 기다리기 기능" |

> 💡 **이 부분은 그냥 "장바구니에 필요한 물건 담기"라고 생각하면 됩니다.** 뭘 담았는지만 기억해두고, 실제로 어떻게 쓰이는지는 아래에서 하나씩 나옵니다.
> 

---

#### 2️⃣ 클래스 시작 — "설계도" 선언

```python
class PatrolClient(Node):
```

이 한 줄만 따로 봅시다.

| 조각 | 뜻 |
| --- | --- |
| `class PatrolClient` | "PatrolClient라는 이름의 틀(설계도)을 만든다" |
| `(Node)` | "그런데 이 틀은 `Node`라는 기존 틀을 **베이스로 삼는다**" |

> 🔍 **비유**: `Node`는 "자동차의 기본 골격(바퀴, 엔진)"이고, `PatrolClient`는 거기에 "우리 회사 로고랑 옵션을 추가한 자동차"입니다. 기본 골격은 그대로 물려받고, 우리가 필요한 기능만 추가하는 것.
> 

#### 3️⃣ `__init__` — 이 클래스가 "태어날 때" 하는 일

```python
def __init__(self, namespace):
    super().__init__('patrol_client_' + namespace)
```

**아주 천천히 뜯어봅시다:**

| 조각 | 뜻 |
| --- | --- |
| `def __init__(self, namespace):` | "이 클래스로 뭔가 만들어질 때(예: `PatrolClient('pinky1')`) 자동으로 실행되는 함수" |
| `namespace` | 밖에서 넘겨준 값, 예를 들면 `'pinky1'` |
| `super()` | "내 부모(`Node`)를 가리켜라" |
| `super().__init__(...)` | "부모(`Node`)가 하는 초기 설정을 먼저 실행해줘" |
| `'patrol_client_' + namespace` | 문자열 두 개를 더하기: `'patrol_client_' + 'pinky1'` → `'patrol_client_pinky1'` |

> 🔍 **`'patrol_client_' + namespace` 계산 예시**
> 
> 
> ```
> namespace = 'pinky1'
> 'patrol_client_' + namespace
> = 'patrol_client_' + 'pinky1'
> = 'patrol_client_pinky1'
> ```
> 
> 문자열끼리 `+`를 하면 그냥 이어붙습니다 (숫자 더하기와 다름!).
> 

> 🔍 **`super().__init__(...)`를 왜 꼭 써야 하나요?**
> 
> 
> 이걸 안 쓰면, `Node`가 해줘야 하는 기본 설정(ROS2 시스템과 연결하는 것)이 하나도 안 됩니다. 그러면 바로 다음 줄에서 쓰는 `self.create_publisher(...)` 같은 기능이 아예 고장 납니다. **무조건 클래스 맨 처음에 한 번 호출**해줘야 하는 필수 절차라고 외워두면 됩니다.
> 

<aside>
💡

#### [참고] Class 문법에 대한 디테일한 설명

### `super()`, `Node`, "부모"가 뭔지 아주 기초부터

#### 1. 먼저 "부모 클래스"라는 개념부터 (파이썬 순수 예시로)

ROS2 다 빼고, 완전히 순수한 파이썬 예제로 먼저 보겠습니다.

```python
class Animal:
    def __init__(self, name):
        self.name = name
        print(f'{name}이(가) 태어났습니다')

class Dog(Animal):
    def __init__(self, name):
        super().__init__(name)
        print('멍멍!')
```

| 용어 | 이 예시에서 누구? |
| --- | --- |
| **부모 클래스** (= 상위 클래스) | `Animal` |
| **자식 클래스** (= 하위 클래스) | `Dog` |
| `class Dog(Animal):`의 `(Animal)` | "`Dog`는 `Animal`을 부모로 삼는다"는 뜻 |
| `super()` | "내 부모(`Animal`)를 가리켜라" |
| `super().__init__(name)` | "부모(`Animal`)의 `__init__` 함수를 실행해줘" |

**실행해보면:**

```python
d = Dog('바둑이')
```

```
바둑이이(가) 태어났습니다     ← super().__init__(name) 가 실행한 것
멍멍!                     ← Dog 자신의 코드가 실행한 것
```

---

#### 2. 왜 부모 클래스가 필요한가? (비유)

**부모 = "이미 만들어진 기본 틀"**, **자식 = "그 틀에 내 것만 추가"**

| 비유 | 코드 |
| --- | --- |
| `Animal`(부모)이 이미 "이름 짓기" 기능을 가지고 있음 | `Animal.__init__`이 `self.name = name`을 함 |
| `Dog`(자식)는 그 기능을 **또 안 만들고 재사용**함 | `super().__init__(name)` 한 줄로 끝 |
| `Dog`만의 새 기능(짖기)만 추가로 씀 | `print('멍멍!')` |

만약 `super().__init__(name)`을 안 쓰면?

```python
class Dog(Animal):
    def __init__(self, name):
        print('멍멍!')   # super() 호출 안 함

d = Dog('바둑이')
print(d.name)   # ❌ 에러! self.name이 아예 생성된 적이 없음
```

→ 부모가 해주는 기본 설정(`self.name = name`)이 통째로 빠지게 됩니다.

---

#### 3. 이제 여러분 코드에 그대로 적용

```python
class PatrolClient(Node):
    def __init__(self, namespace):
        super().__init__('patrol_client_' + namespace)
```

**1:1 대응표:**

| 아까 예시 | 여러분 코드 |
| --- | --- |
| `Animal` (부모) | `Node` (부모) — **rclpy가 미리 만들어놓은 "ROS2 노드의 기본 틀"** |
| `Dog` (자식) | `PatrolClient` (자식) |
| `class Dog(Animal):` | `class PatrolClient(Node):` |
| `super().__init__(name)` | `super().__init__('patrol_client_' + namespace)` |

즉, **"부모는 누구냐"의 답은 `Node`입니다.**

```python
class PatrolClient(Node):
        # 여기 괄호 안에 있는 게 바로 부모!
```

---

#### 4. `Node`(부모)가 미리 해주는 일이 뭔데?

`Node`는 Anthropic이 아니라 **rclpy 라이브러리**가 이미 만들어놓은 클래스입니다. 대충 이런 식으로 생겼다고 상상하면 됩니다 (실제 내부 코드는 훨씬 복잡하지만 개념만):

```python
# rclpy가 이미 만들어놓은 것 (여러분이 직접 안 씀)
class Node:
    def __init__(self, node_name):
        self.name = node_name
        # ROS2 시스템에 "나 이런 노드야" 하고 등록하는 복잡한 작업들...
        # create_publisher, create_subscription 같은 기능도
        # 다 이 Node 클래스 안에 이미 구현되어 있음
```

**그래서:**

| 우리가 직접 안 만들어도 되는 것 | 왜? |
| --- | --- |
| `self.create_publisher(...)` | `Node`(부모)가 이미 만들어놓은 기능이라 `PatrolClient`(자식)도 그냥 갖다 씀 |
| `self.create_subscription(...)` | 마찬가지 |
| `self.get_name()` | 마찬가지 |

→ 이게 바로 **상속의 핵심 이유**: 매번 "노드 만드는 법"을 처음부터 다시 안 짜도, `Node`를 부모로 삼기만 하면 그 기능들을 **공짜로 물려받습니다.**

---

#### 5. `super().__init__(...)`의 정확한 실행 순서

```python
def __init__(self, namespace):
    super().__init__('patrol_client_' + namespace)
```

```
1. PatrolClient('pinky1') 호출됨
        ↓
2. namespace = 'pinky1' 로 받음
        ↓
3. 'patrol_client_' + namespace  →  'patrol_client_pinky1' 계산
        ↓
4. super().__init__('patrol_client_pinky1') 실행
        ↓
   [여기서 부모 Node의 __init__으로 점프!]
   Node.__init__(self, 'patrol_client_pinky1')
        - 노드 이름을 'patrol_client_pinky1'로 저장
        - ROS2 시스템에 이 노드를 등록
        - create_publisher, create_subscription 같은 기능을
          쓸 수 있는 상태로 준비시킴
        ↓
   [다시 PatrolClient.__init__으로 복귀]
5. (만약 이 아래에 self.cmd_pub = ... 같은 코드가 있다면 이제 실행 가능)
```

> 🔑 **핵심**: `super().__init__(...)`을 실행하기 **전까지는**, `self.create_publisher(...)` 같은 것들이 아직 준비 안 된 상태입니다. 그래서 항상 `__init__` 안에서 **제일 먼저** 호출해야 합니다.
> 

---

#### 6. 한눈에 정리

| 질문 | 답 |
| --- | --- |
| 부모가 누구인가? | `Node` (rclpy 라이브러리가 미리 만들어둔 클래스) |
| 자식이 누구인가? | `PatrolClient` (여러분이 만든 클래스) |
| `super()`는 뭘 가리키나? | "내 부모 클래스" (`Node`)를 가리킴 |
| `super().__init__(...)`는 왜 필요한가? | 부모(`Node`)가 해주는 기본 설정을 먼저 실행해야, 자식이 그 기능들(publisher 만들기 등)을 쓸 수 있게 됨 |
| 이걸 안 하면? | `self.create_publisher(...)` 같은 코드에서 에러 발생 (기본 설정이 안 됐으므로) |
</aside>

계속해서:

```python
    topic_cmd = '/' + namespace + '/patrol_cmd'
    self.cmd_pub = self.create_publisher(String, topic_cmd, 10)
```

| 줄 | 뜻 |
| --- | --- |
| `topic_cmd = '/' + namespace + '/patrol_cmd'` | 문자열 조립: `'/'` + `'pinky1'` + `'/patrol_cmd'` = `'/pinky1/patrol_cmd'` |
| `self.create_publisher(String, topic_cmd, 10)` | "이 주소(`topic_cmd`)로 문자열(`String`) 메시지를 **보낼 수 있는** 우편함을 하나 만들어줘" |
| `self.cmd_pub = ...` | 그 우편함을 `self.cmd_pub`이라는 이름으로 저장 (나중에 또 쓰려고) |

> 🔍 **비유**: `create_publisher`는 "우체국에 가서 발송 전용 사서함을 하나 만드는 것"입니다. `topic_cmd`(`/pinky1/patrol_cmd`)는 그 사서함의 **주소**입니다.
> 

```python
    topic_status = '/' + namespace + '/patrol_status'
    self.create_subscription(String, topic_status, self.status_callback, 10)
```

| 줄 | 뜻 |
| --- | --- |
| `topic_status = ...` | 마찬가지로 주소 조립: `/pinky1/patrol_status` |
| `self.create_subscription(String, topic_status, self.status_callback, 10)` | "이 주소로 오는 메시지를 **받을 수 있는** 우편함을 만들고, 뭔가 오면 `status_callback` 함수를 자동으로 실행해줘" |

> 🔍 **`create_publisher` vs `create_subscription` 비교**
> 

|  | **`create_publisher`** | **`create_subscription`** |
| --- | --- | --- |
| **우편함 종류** | 보내는 용도 | 받는 용도 |
| **필요한 것** | 어디로(주소) | 어디서(주소) + 오면 뭘 할지(함수 이름) |
| **이번 코드 예시** | `patrol_cmd` 주소로 명령 보냄 | `patrol_status` 주소에서 상태 받음 |

> 🔍 **`self.status_callback`처럼 괄호 없이 쓰는 이유**
> 
> 
> ```python
> self.status_callback       # "이 함수 자체"를 가리키는 것 (아직 실행 안 함)
> self.status_callback()     # 지금 당장 실행하는 것 (여기선 쓰면 안 됨!)
> ```
> 
> 여기선 "메시지가 도착하면 나중에 실행해줘"라고 **함수를 등록만** 하는 것이라 괄호 없이 이름만 넘깁니다.
> 

---

#### 4️⃣`status_callback` — 메시지 오면 자동 실행되는 함수

```python
def status_callback(self, msg):
    print('상태 수신:', msg.data)
```

| 줄 | 뜻 |
| --- | --- |
| `def status_callback(self, msg):` | "메시지(`msg`)가 도착했을 때 실행될 함수" |
| `msg.data` | 그 메시지 안에 들어있는 실제 문자열 값 (예: `'주행 중'`) |
| `print('상태 수신:', msg.data)` | 화면에 `상태 수신: 주행 중` 같은 식으로 출력 |

> 🔍 **이 함수는 내가 직접 부르지 않습니다!** `/pinky1/patrol_status` 주소로 뭔가 도착하는 **그 순간**, ROS2가 알아서 이 함수를 실행시켜 줍니다. 
마치 초인종이 울리면 자동으로 문 앞 카메라가 켜지는 것과 비슷합니다.
> 

---

#### 5️⃣ `send_command` — 명령 보내는 함수

```python
def send_command(self, cmd):
    time.sleep(1.0)

    msg = String()
    msg.data = cmd
    self.cmd_pub.publish(msg)
```

<aside>
💡

#### `cmd`와 `msg.data`가 정확히 뭔지 추적해보기

#### 1. 먼저 결론부터: `cmd`는 그냥 "전달받은 값이 담기는 이름표"

```python
def send_command(self, cmd):
```

이 줄에서 `cmd`는 **아직 아무 값도 아닙니다.** 그냥 "나중에 여기로 뭔가 들어올 자리"를 미리 이름 지어둔 것뿐입니다.

> 🔍 **비유**: `cmd`는 택배 받는 사람 이름이 아직 안 적힌 **빈 이름표**입니다. 실제로 누가 택배를 보내야(=함수를 호출해야) 그 이름표에 실제 이름이 적힙니다.
> 

---

#### 2. 실제로 언제 값이 채워지나? — 호출하는 곳을 봐야 함

이 함수 정의만 보면 `cmd`가 뭔지 절대 알 수 없습니다. **어디서 이 함수를 불렀는지**를 봐야 합니다. 이전 코드에 이런 부분이 있었죠:

```python
if action == 'start' or action == 'stop':
    node.send_command(action)
```

여기서 `action`은 터미널에서 입력받은 값입니다. 예를 들어 터미널에 이렇게 입력했다면:

```
python3 patrol_client.py pinky1 start
```

→ `action = 'start'` 가 됩니다.

**이제 `node.send_command(action)`을 호출하면:**

```
node.send_command(action)
              │
              ▼
   action의 실제 값인 'start' 가
   send_command 함수의 cmd 자리에 복사되어 들어감
              │
              ▼
def send_command(self, cmd):   ← 이 순간 cmd = 'start' 가 됨
```

---

#### 3. 순수 파이썬으로 이 원리만 떼어서 연습

ROS2 다 빼고, 아주 단순한 예시로 봅시다.

```python
def greet(name):
    print('안녕, ' + name)

greet('철수')
```

| 줄 | 무슨 일이 일어나나 |
| --- | --- |
| `def greet(name):` | `name`이라는 **빈 이름표**를 가진 함수를 정의 |
| `greet('철수')` | 함수를 호출하면서 `'철수'`라는 값을 넘김 |
| 함수 안에서 | 이 순간 `name = '철수'`가 됨 |
| 출력 | `안녕, 철수` |

**같은 함수를 다른 값으로 또 불러보면:**

```python
greet('영희')   # 이번엔 name = '영희'
```

```
안녕, 영희
```

> 🔑 **핵심**: `name`(또는 `cmd`)이라는 이름은 **고정**되어 있지만, 그 안에 담기는 **실제 값은 호출할 때마다 달라질 수 있습니다.**
> 

이걸 여러분 코드에 그대로 대입하면:

```python
def send_command(self, cmd):   # cmd는 빈 이름표
    ...

node.send_command('start')     # 이번엔 cmd = 'start'
node.send_command('stop')      # 다음번엔 cmd = 'stop'  (같은 함수, 다른 값)
```

---

#### 4. 이제 함수 안에서 `cmd`가 어떻게 쓰이는지 한 줄씩

```python
def send_command(self, cmd):
    time.sleep(1.0)

    msg = String()
    msg.data = cmd             # data는 msg의 값
    self.cmd_pub.publish(msg)
```

`cmd = 'start'`라고 가정하고 한 줄씩 실제 값을 대입해서 따라가 봅시다:

| 줄 | 실행 후 상태 |
| --- | --- |
| `time.sleep(1.0)` | 1초 대기 (cmd랑 관계없음) |
| `msg = String()` | 빈 메시지 상자 생성. 이 시점엔 `msg.data`가 아직 없음 (또는 빈 값) |
| `msg.data = cmd` | `cmd`(='start')를 `msg.data`에 **복사해서 넣음** → 이제 `msg.data == 'start'` |
| `self.cmd_pub.publish(msg)` | 이 `msg`(안에 `data='start'`가 들어있는 상태)를 실제로 전송 |

---

#### 5. `cmd`와 `msg.data`의 관계 — 그림으로

```
cmd 라는 변수                    msg 라는 객체
┌─────────┐                    ┌──────────────┐
│ 'start' │  ──── 복사 ────>    │ msg.data =   │
└─────────┘   (msg.data=cmd)   │   'start'    │
                               └──────────────┘
```

- `cmd`는 그냥 **함수 안에서 잠깐 쓰는 변수**
- `msg.data`는 **메시지 상자 안의 특정 칸**(ROS2가 정해둔 규칙: `String` 타입은 반드시 `.data`라는 칸에 문자열을 넣어야 함)
- `msg.data = cmd`는 "`cmd`에 든 값을 `msg.data`라는 칸으로 옮겨 담는 것"

> 🔑 **`=`(등호)의 의미**: 수학의 "같다"가 아니라, "오른쪽 값을 왼쪽에 **저장(대입)**해라"는 명령입니다.
> 
> 
> ```python
> msg.data = cmd
> #    ↑        ↑
> #  여기에   이 값을
> #  저장해   가져와서
> ```
> 

---

#### 6. 왜 굳이 `cmd`라는 중간 변수를 거치나? (반례)

만약 `msg.data = cmd` 없이 아예 다르게 짰다면 어떻게 될까 비교해봅시다.

**지금 방식 (변수로 값 전달):**

```python
def send_command(self, cmd):
    msg = String()
    msg.data = cmd
    self.cmd_pub.publish(msg)

node.send_command('start')   # 이렇게 부르면 'start'가 전달됨
node.send_command('stop')    # 이렇게 부르면 'stop'이 전달됨
```

→ **하나의 함수**로 `start`도, `stop`도, 나중에 다른 명령어도 다 처리 가능.

**만약 `cmd`를 안 받고 하드코딩했다면 (나쁜 예):**

```python
def send_start(self):
    msg = String()
    msg.data = 'start'    # 'start'라고 고정박음
    self.cmd_pub.publish(msg)

def send_stop(self):
    msg = String()
    msg.data = 'stop'     # 똑같은 코드를 또 씀
    self.cmd_pub.publish(msg)
```

→ 명령어 종류마다 함수를 **따로따로 만들어야** 해서 코드가 중복되고 지저분해짐.

> 🔑 **`cmd`라는 변수를 쓰는 이유**: 똑같은 로직(1초 기다리고, 상자에 담고, 보내는 것)을 **재사용**하면서, 안에 들어갈 값만 그때그때 바꿔 쓰기 위함입니다.
> 

---

#### 정리

| 질문 | 답 |
| --- | --- |
| `cmd`가 어디서 오나? | 이 함수를 호출하는 쪽(`node.send_command('start')`)에서 넘겨준 값 |
| `cmd`는 고정값인가? | 아니오, 호출할 때마다 다른 값이 들어올 수 있는 **변수** |
| `msg.data = cmd`는 뭘 하는 건가? | `cmd`에 담긴 값을 `msg.data`라는 칸에 복사해 넣는 것 |
| 왜 `msg.data`라는 특정 이름을 써야 하나? | `String`이라는 메시지 타입이 ROS2에서 "문자열은 `.data`에 넣는다"고 미리 정해놓은 규칙이라서 |
</aside>

**한 줄씩 아주 천천히:**

| 줄 | 뜻 |
| --- | --- |
| `def send_command(self, cmd):` | `cmd`라는 값(예: `'start'`)을 받아서 처리하는 함수 |
| `time.sleep(1.0)` | **1초 동안 아무것도 안 하고 기다림** |
| `msg = String()` | 빈 메시지 상자를 하나 만듦 |
| `msg.data = cmd` | 그 상자 안에 `cmd` 값(`'start'`)을 넣음 |
| `self.cmd_pub.publish(msg)` | 아까 만들어둔 "발송 사서함"(`self.cmd_pub`)으로 이 메시지를 실제로 발송 |

> 🔍 **왜 `time.sleep(1.0)`으로 1초를 기다릴까?**
> 
> 
> 이 프로그램이 막 켜지자마자 바로 메시지를 보내면, 로봇 쪽 프로그램이 아직 준비가 안 되어 있을 수 있습니다. 이 상태에서 보낸 메시지는 **아무도 못 받고 그냥 사라집니다.**
> 
> ```
> ❌ sleep 없이 바로 보내면:
> [내 프로그램 켜짐] → 바로 publish → (로봇이 아직 안 켜져있음) → 메시지 증발 😢
> 
> ✅ 1초 기다리고 보내면:
> [내 프로그램 켜짐] → 1초 대기 (이 사이 로봇도 켜질 시간을 줌) → publish → 로봇이 받음 ✅
> ```
> 
> 완벽한 방법은 아니지만("혹시 로봇이 1초보다 늦게 켜지면?" 하는 예외는 남아있음), 초보자가 이해하기엔 훨씬 단순한 방법입니다.
> 

> 🔍 **`msg = String()` 다음 `msg.data = cmd`, 이렇게 2줄로 나눈 이유**
> 
> 
> ```python
> msg = String()      # 1단계: 빈 상자 만들기
> msg.data = cmd       # 2단계: 상자 안에 내용물 넣기
> ```
> 
> 택배 상자를 먼저 준비하고, 그 다음에 물건을 넣는 것과 같은 순서입니다. 상자(`msg`)를 만들지 않고 바로 `msg.data`에 접근하면 에러가 납니다 (상자가 없는데 내용물부터 넣을 수 없음).
> 

---

#### 6️⃣`main` 함수 — 프로그램의 "시작점"

```python
def main():
    if len(sys.argv) < 3:
        print('사용법: python3 patrol_client.py <pinky1|pinky2> <start|stop|monitor>')
        return
```

| 줄 | 뜻 |
| --- | --- |
| `sys.argv` | 터미널에 입력한 것들을 리스트로 담아놓은 것 |

**예시로 확인:**

```
터미널에 이렇게 입력했다면:
python3 patrol_client.py pinky1 start

sys.argv 는 이렇게 됩니다:
sys.argv[0] = 'patrol_client.py'
sys.argv[1] = 'pinky1'
sys.argv[2] = 'start'

len(sys.argv) = 3   ← 총 3개
```

| 줄 | 뜻 |
| --- | --- |
| `len(sys.argv)` | 리스트 안에 몇 개가 들어있는지 개수 세기 |
| `if len(sys.argv) < 3:` | "3개보다 적으면" (즉, namespace나 action을 안 적었으면) |
| `print(...)` | 사용법 알려주기 |
| `return` | **여기서 함수를 즉시 끝냄** (더 밑에 있는 코드는 실행 안 됨) |

---

```python
    namespace = sys.argv[1]
    action = sys.argv[2]
```

| 줄 | 뜻 |
| --- | --- |
| `namespace = sys.argv[1]` | 터미널에서 입력한 2번째 값(`'pinky1'`)을 `namespace`라는 이름으로 저장 |
| `action = sys.argv[2]` | 3번째 값(`'start'`)을 `action`이라는 이름으로 저장 |

---

```python
    rclpy.init()
    node = PatrolClient(namespace)
```

| 줄 | 뜻 |
| --- | --- |
| `rclpy.init()` | "이제부터 ROS2 프로그램 시작할게" 하고 시스템 켜기 |
| `node = PatrolClient(namespace)` | 아까 만든 설계도(`PatrolClient`)로 **진짜 객체**를 하나 만듦. 이때 `__init__`이 자동 실행됨 |

> 🔍 **`node = PatrolClient(namespace)` 이 한 줄이 일어나는 일**
> 
> 
> ```
> PatrolClient('pinky1') 호출
>     ↓
> __init__(self, namespace='pinky1') 자동 실행
>     ↓
> super().__init__('patrol_client_pinky1')  → 노드 이름 설정
> self.cmd_pub 생성 (발송 사서함)
> self.create_subscription(...) 생성 (수신 사서함 + 콜백 등록)
>     ↓
> 완성된 노드가 node 라는 변수에 저장됨
> ```
> 

---

```python
    if action == 'start' or action == 'stop':
        node.send_command(action)
        print(namespace, '에게', action, '명령 전송 완료')
```

| 줄 | 뜻 |
| --- | --- |
| `action == 'start' or action == 'stop'` | "action이 'start'와 같거나, 또는 'stop'과 같으면" |
| `node.send_command(action)` | 아까 만든 함수 호출 → 실제로 명령 발송됨 |
| `print(namespace, '에게', action, '명령 전송 완료')` | `pinky1 에게 start 명령 전송 완료` 같은 문장 출력 |

> 🔍 **`print`에 쉼표(`,`)로 여러 개 넣으면?**
> 
> 
> ```python
> print('pinky1', '에게', 'start', '명령 전송 완료')
> # 출력: pinky1 에게 start 명령 전송 완료
> ```
> 
> 쉼표로 구분한 값들을 자동으로 **띄어쓰기와 함께** 이어서 출력해줍니다.
> 

---

```python
    elif action == 'monitor':
        print(namespace, '상태 모니터링 시작 (Ctrl+C로 종료)')
        rclpy.spin(node)
```

| 줄 | 뜻 |
| --- | --- |
| `elif action == 'monitor':` | 위 조건이 아니고, `action`이 `'monitor'`이면 |
| `rclpy.spin(node)` | **여기서 프로그램이 멈춰서 계속 대기 상태**가 됨. 메시지 올 때마다 `status_callback`이 계속 실행됨. Ctrl+C를 눌러야 빠져나옴 |

```python
    else:
        print('알 수 없는 명령:', action)
```

| 줄 | 뜻 |
| --- | --- |
| `else:` | 위 두 조건 다 아니면 (오타 등으로 이상한 값을 입력한 경우) |

---

```python
    node.destroy_node()
    rclpy.shutdown()
```

| 줄 | 뜻 |
| --- | --- |
| `node.destroy_node()` | 다 쓴 노드를 정리(청소)함 |
| `rclpy.shutdown()` | ROS2 시스템 전체를 끔 |

---

```python
if __name__ == '__main__':
    main()
```

| 줄 | 뜻 |
| --- | --- |
| `if __name__ == '__main__':` | "이 파일을 터미널에서 직접 실행했을 때만" |
| `main()` | 위에서 만든 `main` 함수를 실제로 호출해서 프로그램 시작 |

> 🔍 **쉬운 비유**: 이 줄은 "이 코드가 **주인공으로 실행됐을 때만** 시작 버튼을 눌러라"는 뜻입니다. 만약 이 파일을 다른 파일이 그냥 "재료로만" 가져다 쓴다면 (`import patrol_client`), `main()`이 멋대로 실행되지 않게 막아주는 안전장치입니다.
> 

---

## 4. 전체 흐름을 그림 하나로

터미널에 `python3 patrol_client.py pinky1 start`를 입력했다고 가정:

```
1. sys.argv 확인 → 개수 충분함 (3개)
2. namespace = 'pinky1', action = 'start'
3. rclpy.init() → ROS2 켜기
4. node = PatrolClient('pinky1')
      → __init__ 실행: 발송 사서함 + 수신 사서함 준비
5. action == 'start' 이므로:
      → node.send_command('start') 호출
            → 1초 대기
            → 메시지 상자에 'start' 담기
            → 발송!
      → "pinky1 에게 start 명령 전송 완료" 출력
6. node.destroy_node()
7. rclpy.shutdown()
8. 프로그램 끝
```

</aside>
