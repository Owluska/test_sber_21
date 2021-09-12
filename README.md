# test_sber_21

# Установка проекта

Перейти в  ```~/catkin_ws```

В bash ```git clone URL```

        ```git checkout release```
# Сборка проекта
Перейти в  ```~/catkin_ws```

В bash ```catkin_make```

# Для запуска python ноды
Указать актуальный путь к *.bag файлу, для этого [в файле](https://github.com/Owluska/test_sber_21/tree/master/src/semantic_to_occupancy/src/launch/sem_to_ocp_grid.launch)
изменить аргумент ```path```

После этого запустить ноду следующей командой в bash:

 ```$roslaunch semantic_to_occupancy sem_to_ocp_grid.launch```
  
# Для запуска C++ ноды
Указать актуальный путь к *.bag файлу, для этого [в файле](https://github.com/Owluska/test_sber_21/tree/master/src/sem_to_costmap/launch/bag.launch)
изменить аргумент ```path```

После этого запустить ноду следующей командой в bash:

 ```$roslaunch sem_to_costmap bag.launch```
  

 # Для просмотра карты в rviz
Добавить карту с отслеживанием топика ```/costmap```, для этого в Rviz

1. В левом нижнем углу выбрать кнопку "Add"

2. В появившемся окне выбрать вкладку "By topic"

3. Выбрать /costmap
