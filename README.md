# test_sber_21
# Для запуска python ноды
1. Указать актуальный путь к *.bag файлу, для этого 

 
 [в файле](https://github.com/Owluska/test_sber_21/tree/master/src/semantic_to_occupancy/src/launch/sem_to_ocp_grid.launch)
 изменить аргумент ```path```

2. Запустить ноду:

 
 ```$roslaunch semantic_to_occupancy sem_to_ocp_grid.launch```
  


 # Для просмотра карты в rviz
Добавить карту с отслеживанием топика ```/costmap```, для этого в Rviz

1. В левом нижнем углу выбрать кнопку "Add"

2. В появившемся окне выбрать вкладку "By topic"

3. Выбрать /costmap
