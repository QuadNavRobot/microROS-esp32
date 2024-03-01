# Instalación de Micro-ROS en ESP32

### Instalación de esp-idf

1. Instalar los siguientes paquetes:
```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

mkdir -p ~/esp

cd ~/esp

git clone --recursive https://github.com/espressif/esp-idf.git

cd ~/esp/esp-idf

./install.sh esp32

export IDF_TOOLS_PATH="$HOME/required_idf_tools_path"

./install.sh

. ./export.sh
```

Para no tener que ejecutar ese comando cada vez que se quiera utilizar la herramienta hacer:
```
gedit ~/.zshrc 

Al final agregar:
alias get_idf='. $HOME/esp/esp-idf/export.sh' 
```

### Instalación de micro-ROS
```
pip3 install catkin_pkg lark-parser colcon-common-extensions

cd ~esp/esp-idf/components

git clone https://github.com/micro-ROS/micro_ros_espidf_component.git

cd micro_ros_espidf_component

git checkout -b humble

get_idf

. $IDF_PATH/export.sh

cd examples/int32_publisher

idf.py set-target esp32

idf.py build
```