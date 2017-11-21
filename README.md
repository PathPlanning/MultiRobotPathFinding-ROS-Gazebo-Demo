**ГЕНЕРАЦИЯ ОКРУЖЕНИЯ И РАССЕЛЕНИЕ АГЕНТОВ ИЗ ЛОГА**
=====================================================================================================
На данный момент:

	- реализована генерация .world и .launch из предоставленного лога
	- запуск сгенерированных .world и .launch
	- генерация входных данных для скрипта управления роботами

***	

_В_ _процессе_ - скрипт для управления роботами

***

Для запуска необходимо:

	- установить библиотеку stringtemplate3 (требуется для работы генератора): в окне терминала выполнить команду ```pip install stringtemplate3```
	- положить данные из репозитория в домашнюю папку пользователя
	- в удобную директорию положить `run.sh` и предоставить права на запуск (`chmod u+x *[directory]/run.sh`)

***
	
**ГЕНЕРАЦИЯ ОКРУЖЕНИЯ И РАССТАНОВКА РОБОТОВ В СТАРТОВЫЕ ПОЗИЦИИ**
-----------------------------------------------------------------------------------------------------

#Для совершения генерации нужно последовательно совершить ряд действий:

	1)Положить логи по пути: `~/catkin_ws/src/gazebo/input`
	2) запустить `generate.sh` из директории `~/catkin_ws`
	3) скрипт запросит название .xml файла. Вводить нужно полное имя файла без кавычек. 

Скрипт сгенерирует файлы: `playground.world`, `turtlebot_world.launch`, список .csv файлов,
каждый из которых хранит список команд для одного агента, `run_agents.sh` - нужен для реализации передвижения агентов.

файлы можно найти в соотвествующих вложенных директориях в catkin_ws

После данных манипуляций можно запустить `./run.sh`

Важно: замечено, что иногда Gazebo не может запуститься правильно. Терминал уведомит об этом , сообщив: ```[gazebo-2] process has died..```
В случае возникновения - нужно убить Gazebo (Ctrl+C в этом окне терминала) и запустить ее снова. Возможно неоднократное появление ошибки. В случае, если при запуске загрузился старый .launch - `killall gzserver`.

В зависимости от количества роботов будет отличаться как время запуска Gazebo, так и скорость работы в целом.
**Не рекомендуется пытаться визуализировать 64 агентов на слабой машине**

***
**ЗАПУСК АГЕНТОВ:**
------------------------------------------------------------------------------------------------------

_будет_ _заполнено_ _позже_


