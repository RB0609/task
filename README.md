# task
# Install these dependencies
```
python -m pip install -U pip setuptools wheel
```
2. Install a ROS(cv_bridge)-compatible stack:<br>
numpy must be <2 <br>
opencv must be <4.13 (4.13+ requires numpy>=2)
```
python -m pip install "numpy<2" "opencv-python<4.13"
```
3. Install pandas + Excel writer
```
python -m pip install pandas openpyxl
```
4. (Optional but commonly needed with ROS Jazzy python packages in a venv)
```
python -m pip install pyyaml requests jinja2 typeguard
```
