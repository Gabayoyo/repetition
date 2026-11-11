# Repetition

Vision-controlled PincherX arm for replicating realistic human movement.

## Setup

```bash
python3.14 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Running

```bash
python main.py                 # capture, track, build packets
python main.py --visualise     # also show the camera overlay
python main.py --packets       # write JSON packets to stdout
```