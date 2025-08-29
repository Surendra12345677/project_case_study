# Multimedia Playback Optimization — C++/FFmpeg/OpenCV (Linux)

## 0) Install prerequisites (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y git build-essential cmake pkg-config   libavformat-dev libavcodec-dev libavutil-dev libswscale-dev libswresample-dev   libopencv-dev libasound2-dev
```

> On Arch: `sudo pacman -S ffmpeg opencv alsa-lib cmake pkgconf` (dev headers are included).  
> On Fedora: `sudo dnf install ffmpeg-devel opencv-devel alsa-lib-devel cmake pkgconf-pkg-config`.

---

## 1) Create the repo

```bash
git init mplay && cd mplay
echo "# mplay" > README.md
printf "/build/\n/.vscode/\n" > .gitignore
git add .
git commit -m "chore: init repository"
```

> If you downloaded this starter zip, unzip and then `git init && git add . && git commit -m "chore: import starter"`.

---

## 2) Add build system (CMake)

Files:
- `CMakeLists.txt`
- `.vscode/tasks.json` and `.vscode/launch.json` (optional quality-of-life)

```bash
git add CMakeLists.txt .vscode
git commit -m "build: add CMake and VS Code tasks/launch"
```

---

## 3) Add a basic video pipeline

We decode video and display frames with OpenCV (`imshow`).

```bash
git add src/main.cpp
git commit -m "feat(video): decode and display video frames via OpenCV"
```

---

## 4) Add audio decode and playback via ALSA

We resample any input to signed 16-bit stereo 48 kHz and push to ALSA.

```bash
git add src/main.cpp
git commit -m "feat(audio): decode and play audio to ALSA (48kHz S16 stereo)"
```

---

## 5) Add simple A/V sync (audio master)

We use **audio clock** as master. Video frames are delayed/dropped to follow audio PTS.

```bash
git add src/main.cpp
git commit -m "feat(sync): audio-master sync using PTS and wait/drop strategy"
```

---

## 6) Configure & build in VS Code (or terminal)

**VS Code**: Press `Ctrl+Shift+B` and choose **Build (CMake)**.  
**Terminal**:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

---

## 7) Run

```bash
./build/mplay path/to/video_or_audio_file.mp4
```
