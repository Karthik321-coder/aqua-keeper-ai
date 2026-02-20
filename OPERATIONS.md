# Operations Guide

## Starting the System

```bash
# Activate environment
source .venv/bin/activate

# Run main loop
python src/pipeline/main.py --config configs/default.yaml
```

Add `--loglevel DEBUG` for verbose output during commissioning.

Press **Ctrl+C** for graceful shutdown (the actuator will centre the blocker before stopping).

---

## Monitoring

### Live Logs

```bash
tail -f logs/run_$(ls -t logs/ | head -1)
```

### System Resources

```bash
# CPU/memory
htop

# Pi GPU/CPU temperature
watch -n 2 vcgencmd measure_temp

# Pi throttle status
vcgencmd get_throttled
```

### Key Metrics to Watch

| Metric | Healthy | Alert |
|--------|---------|-------|
| Inference latency | < 30 ms | > 50 ms |
| Frame drop rate | < 5 % | > 15 % |
| CPU temperature | < 70°C | > 80°C |
| Actuator faults | 0 | > 0 |

---

## Logging

Logs are written to `logs/run_<YYYYMMDD_HHMMSS>.jsonl` (JSON Lines format).

### Log Rotation

Logs are **not** automatically rotated. Set up a cron job to archive old logs:

```bash
# crontab -e
0 2 * * * find /home/pi/aqua-keeper-ai/logs -name "*.jsonl" -mtime +7 -exec gzip {} \;
```

### Log Replay

```bash
python scripts/test_loop.py --replay logs/run_20240101_120000.jsonl
```

This replays detections through the controller and actuator (no hardware required) for post-hoc analysis.

---

## Model Updates

1. Train new weights (see `VISION.md`).
2. Export to target format:
   ```bash
   yolo export model=models/new_weights.pt format=ncnn imgsz=640
   ```
3. Update `configs/default.yaml` → `model.weights_path`.
4. Run validation:
   ```bash
   python scripts/test_loop.py --scenario accuracy
   ```
5. If accuracy improves, rename to `models/yolov8n_ball.pt` and restart the main loop.

---

## Maintenance Checklist

### Weekly

- [ ] Inspect belt tension; re-tension if slack.
- [ ] Check cable glands for water ingress.
- [ ] Wipe camera lens with dry cloth.
- [ ] Review logs for ERROR entries.
- [ ] Verify e-stop button function.

### Monthly

- [ ] Inspect bearings on carriage for corrosion; lubricate with waterproof grease.
- [ ] Check all M3/M4 fasteners are tight.
- [ ] Test servo/stepper under no-load and full-load sweep.
- [ ] Review model performance with new data if available.
- [ ] Archive and compress old logs.

### After Each Pool Session

- [ ] Power off the system before cleaning.
- [ ] Rinse the belt and extrusion with fresh water (chlorine is corrosive).
- [ ] Dry electronics box interior with silica gel packet if condensation is present.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Camera not detected | Wrong device index | Set `camera.source: 1` (or 2, etc.) in config |
| High inference latency | Model too large | Use YOLOv8n NCNN export |
| Blocker overshoots | Kd too low | Increase `control.kd` by 0.01 increments |
| Blocker oscillates | Kp too high | Halve `control.kp` |
| E-stop triggers on startup | GPIO floating | Check wiring; add 10 kΩ pull-up to GPIO 26 |
| No detections | Wrong weights | Verify `model.weights_path` in config |
| `ModuleNotFoundError` | Deps not installed | `pip install -r requirements.txt` |
