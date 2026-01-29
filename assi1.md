# AAE5303 Environment Setup Report — Template for Students

> **Important:** Follow this structure exactly in your submission README.  
> Your goal is to demonstrate **evidence, process, problem-solving, and reflection** — not only screenshots.

---

## 1. System Information

**Laptop model:**  
_[Your laptop model, legion y7000]_

**CPU / RAM:**  
_[e.g.,amd ryzen r9-8940HX, 32GB RAM]_

**Host OS:**  
_[Windows 11]_

**Linux/ROS environment type:**  
_[Choose one:ROS]_
- [ ] Dual-boot Ubuntu
- [ ] WSL2 Ubuntu
- [ ] Ubuntu in VM (UTM/VirtualBox/VMware/Parallels)
- [X] Docker container
- [ ] Lab PC
- [ ] Remote Linux server

---

## 2. Python Environment Check

### 2.1 Steps Taken

Describe briefly how you created/activated your Python environment:

**Tool used:**  
_[venv / conda / system Python]_

**Key commands you ran:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Any deviations from the default instructions:**  
_[Describe any changes you made, or write "None"]_

### 2.2 Test Results

Run these commands and paste the actual terminal output (not just screenshots):

```bash
python scripts/test_python_env.py
```

**Output:**
```
[Paste your actual terminal output here]
```

```bash
python scripts/test_open3d_pointcloud.py
```

**Output:**
```
$ cd /root/PolyU-AAE5303-env-smork-test && export LANG=C.UTF-8 && export LC_ALL=C.UTF-8 && source /opt/ros/humble/setup.bash && source .venv/bin/activate && python scripts/test_python_env.py 2>&1 | tail -20
✅ Module 'scipy' found (v1.15.3).
✅ Module 'matplotlib' found (v3.10.8).
✅ Module 'cv2' found (v4.12.0).
✅ Module 'rclpy' found (vunknown).
✅ numpy matrix multiply OK.
✅ numpy version 2.2.6 detected.
✅ scipy FFT OK.
✅ scipy version 1.15.3 detected.
✅ matplotlib backend OK (Agg), version 3.10.8.
✅ OpenCV OK (v4.12.0), decoded sample image 128x128.
✅ Open3D OK (v0.19.0), NumPy 2.2.6.
✅ Open3D loaded sample PCD with 8 pts and completed round-trip I/O.
✅ ROS 2 CLI OK: /opt/ros/humble/bin/ros2
✅ ROS 1 tools not found (acceptable if ROS 2 is installed).
✅ colcon found: /usr/bin/colcon
✅ ROS 2 workspace build OK (env_check_pkg).
✅ ROS 2 runtime OK: talker and listener exchanged messages.
✅ Binary 'python3' found at /root/PolyU-AAE5303-env-smork-test/.venv/bin/python3

All checks passed. You are ready for AAE5303 🚀
```

**Screenshot:**  
_[Include one screenshot showing both tests passing]_

<img width="2559" height="1527" alt="e036e6039b757de0d877102914ac0384" src="https://github.com/user-attachments/assets/d0390cb7-255d-4a94-a00d-cc1615feed38" />

---

## 3. ROS 2 Workspace Check

### 3.1 Build the workspace

Paste the build output summary (final lines only):

```bash
source /opt/ros/humble/setup.bash
colcon build
```

**Expected output:**
```
Summary: 1 package finished [x.xx s]
```

**Your actual output:**
```
Summary: 2 packages finished [1.05s]
```

### 3.2 Run talker and listener

Show both source commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
Publishing: "Hello World: 0"
Publishing: "Hello World: 1"
Publishing: "Hello World: 2"
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
I heard: "Hello World: 0"
I heard: "Hello World: 1"
I heard: "Hello World: 2"
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
_[Include one screenshot showing talker + listener running]_

<img width="2559" height="1527" alt="c736096c30146bd6f55a3cd403c7451c" src="https://github.com/user-attachments/assets/dd4cc2dc-12c4-4921-954d-679912752793" />

---

## 4. Problems Encountered and How I Solved Them

> **Note:** Write 2–3 issues, even if small. This section is crucial — it demonstrates understanding and problem-solving.

### Issue 1: TODO placeholders not filled
**Cause / diagnosis:**  
The package was likely created with ros2 pkg create, which generates boilerplate with TODO placeholders. The developer didn’t replace them after creation

**Fix:**  
_[The exact command/config change you used to solve it]_

```bash
# BEFORE
maintainer='wang',
maintainer_email='wang@todo.todo',
description='TODO: Package description',
license='TODO: License declaration',

# AFTER
maintainer='wang',
maintainer_email='wang@example.com',
description='ROS 2 package for environment checking with talker/listener examples',
license='Apache-2.0',
```

**Reference:**  
AI assistant

---

### Issue 2: Unused variable warning suppression

**Cause / diagnosis:**  
A common ROS 2 Python pattern where create_subscription() returns an object that must be kept alive, but static analyzers flag it as unused because it isn’t explicitly referenced later.

**Fix:**  
_[The exact command/config change you used to solve it]_

```bash
# BEFORE
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10)
self.subscription  # prevent unused variable warning

# AFTER
self._subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10)
```

**Reference:**  
AI assistant
---

## 5. Use of Generative AI (Required)

Choose one of the issues above and document how you used AI to solve it.

> **Goal:** Show critical use of AI, not blind copying.

### 5.1 Exact prompt you asked

**Your prompt:**
```
Tell me the defects in the above code
```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
Unused variable warning suppression
Root cause:
A common ROS 2 Python pattern where create_subscription() returns an object that must be kept alive, but static analyzers flag it as unused because it isn’t explicitly referenced later.
Evidence:
self.subscription = self.create_subscription(...)self.subscription  # prevent unused variable warning
Why it happens:
The subscription must be stored to keep it alive (garbage collection risk)
Static analyzers (e.g., flake8/pyflakes) don’t recognize that assignment as “use”
The second line is a workaround to silence the warning
This is a known ROS 2 Python quirk
```

### 5.3 What you changed or ignored and why

Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?

**Your explanation:**  
The AI ​​contains no insecure content, and I have modified the solution and verified it.

### 5.4 Final solution you applied

Show the exact command or file edit that fixed the problem:

```bash
# BEFORE
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10)
self.subscription  # prevent unused variable warning

# AFTER
self._subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10)
```

**Why this worked:**  
The underscore prefix follows Python's PEP 8 "internal use" convention, which static analyzers recognize and won't warn about, while the subscription object functions identically since the underscore is purely a naming convention that doesn't affect runtime behavior.

---

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**Your reflection:**

Through this experience, I learned that ROS 2 environment configuration requires careful attention to sourcing setup scripts in the correct order, and that seemingly "unused" variables in Python ROS nodes are actually critical for preventing garbage collection. What surprised me was discovering that common workarounds like adding standalone variable references to suppress warnings are code smells, and that Python's underscore prefix convention provides an elegant, standard-compliant solution already used by the ROS 2 community. Next time, I would start by checking ROS 2 official examples and documentation patterns before implementing workarounds, and verify solutions against both PEP 8 conventions and ROS 2 best practices rather than accepting quick fixes. I now feel more confident about debugging ROS/Python issues because I understand the importance of distinguishing between runtime requirements and static analysis concerns, and I know how to use AI critically by verifying recommendations against official documentation and community standards.

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
WANG Ruipu

**Student ID:**  
25128553g

**Date:**  
2026/1/29

---

## Submission Checklist

Before submitting, ensure you have:

- [x] Filled in all system information
- [x] Included actual terminal outputs (not just screenshots)
- [x] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [x] Documented 2–3 real problems with solutions
- [x] Completed the AI usage section with exact prompts
- [x] Written a thoughtful reflection (3–5 sentences)
- [x] Signed the declaration

---

**End of Report**
