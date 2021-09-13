# test_sber_21

## Установка

Перейти в папку **catkin_ws**, для этого в bash ввести команду

```$cd ~/catkin_ws```

Загрузить проект из *github*,

для этого в *bash* ввести команду

 ```$git clone https://github.com/Owluska/test_sber_21.git```

## Сборка проекта

Для того, чтобы собрать проект,

нужно перейти в папку **catkin_ws**

 ```$cd ~/catkin_ws```

и затем запустить сборку командой:

 ```$catkin_make```

## Для запуска python ноды

Указать актуальный путь к *.bag файлу, для этого в *.launch файле

нужно изменить аргумент ```path```

Файл находится **...src/cp2og_python/launch/cp2og_python.launch**

После этого запустить ноду следующей командой в bash:

 ```$roslaunch cp2og_python cp2og_python.launch```

 Если нужно запустить ноду для тестирования без rviz:

создать два терминала в одном запустить команду:

```$roscore & rosbag play FULL_PATH_TO_BAG_FILE```,

где *FULL_PATH_TO_BAG_FILE* - это полный путь к *.bag файлу

в другом запустить ноду командой:

 ```$rosrun cp2og_python cp2og_python_node.py``` 
  
## Для запуска C++ ноды

Указать актуальный путь к *.bag файлу, для этого в *.launch файле

нужно изменить аргумент ```path```

Файл находится **...src/cp2og_cpp/launch/cp2og_cpp.launch**

После этого запустить ноду следующей командой в bash:

 ```$roslaunch cp2og_cpp cp2og_cpp.launch```

 Если нужно запустить ноду для тестирования без rviz:

создать два* терминала в одном запустить команду:

```$roscore & rosbag play FULL_PATH_TO_BAG_FILE```,

где *FULL_PATH_TO_BAG_FILE* - это полный путь к *.bag файлу

в другом запустить ноду командой:

 ```$rosrun cp2og_cpp cp2og_cpp_node``` 

 ## Для просмотра карты в rviz

Карта добавляется из настроек автоматически, если по какой-то причине 

этого не произошло, нужно добавить карту с отслеживанием топика ```/costmap```.

 Для этого в Rviz

1. В левом нижнем углу выбрать кнопку "Add"

2. В появившемся окне выбрать вкладку "By topic"

3. Выбрать /costmap
