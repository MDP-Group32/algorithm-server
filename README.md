# Algorithms Server

## Getting Started

### Prerequisites
1. Python should be installed, you can check via
```
python3 --version
```
or
```
python --version
```

### MacOS
1. Create a virtual environment (if you haven't already)
```
python -m venv venv
```
or 
```
python3 -m venv venv
```
2. Enter the virtual environment
```
source venv/bin/activate
```
3. Verify that you are in the correct virtual env using
```
which python
```
- It should show a path like `algorithms-server/venv/bin/python`
4. Install dependencies:
```
pip install -r requirements.txt
```
5. To run in development mode:
```
fastapi dev main.py
```
6. To run application on RPI:
```
uvicorn main:app --host 0.0.0.0 --port 8000
```

### Windows
1. Create a virtual environment (if you haven't already)
```
python -m venv venv
```
or 
```
python3 -m venv venv
```
2. Enter the virtual environment
```
venv\Scripts\activate
```
3. Verify that you are in the correct virtual env using
- Your path should be prefixed with `(venv)`
4. Install dependencies:
```
pip install -r requirements.txt
```
NOTE: If running on windows comment out: `uvloop==0.20.0` dependency because it is currently not supported on Windows.
5. To run in development mode:
```
fastapi dev main.py
```
6. To run application on RPI:
```
uvicorn main:app --host 0.0.0.0 --port 8000
```

Done in reference to this [repo](https://github.com/xJQx/SC2079-MDP-Group11-Algorithms).
