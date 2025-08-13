# robo_lab
Windows'ta tamamen Python ile çalışan, PyBullet tabanlı modüler robotik proje.

## Özellikler
- 2D mobil robot (differential drive) + LiDAR benzeri 2D tarayıcı (ray casting)
- A* grid planner (Strategy) + saf PID heading/speed kontrolü (Strategy)
- PyBullet ortam sarmalayıcı (Env) + sahne/obstacle yönetimi
- Sensör okuma thread'i, thread-safe queue ile veri aktarımı
- Temiz mimari: core (types, events), drivers (env, lidar), planning, control, robot, app

## Kurulum
```bash
python -m venv .venv
.venv\\Scripts\\activate  # Windows
pip install -e .
```

## Çalıştırma
```bash
python -m robo_lab.app.main
```

## Yapı
- `robo_lab/core` : tipler, arayüzler, event bus
- `robo_lab/drivers` : PyBullet env, LiDAR
- `robo_lab/planning` : grid map + A*
- "robo_lab/control" : PID ve saf takip kontrolcüsü
- `robo_lab/robot` : differential drive model/kinematik
- `robo_lab/app` : konfig, senaryo, entegrasyon