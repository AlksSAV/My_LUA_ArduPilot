# LUAscript for ArduPilot

![Screenshot](screen.png)

Пример работы у меня на канале https://youtu.be/pAMMpPhNr5s


## MultiMissiomSelect.lua   
   3 missions selection / Выбор из 3х миссий (для переключателя на 3 позиции)
   1. задать 1 для SCR_ENABLE
   2. сохранить в папке /APM/scripts/ сам скипт и файлы миссий (3 шт.)
   3. Задать значение 300 для RCx_OPTION(х номер канала на котором 3х позиционный переключатель) 
   4. перезагрузить контроллер
## SwitchMission.lua        
   two or more missions select / Выбор от 2х или более миссий 
   1. задать в MP 1 для SCR_ENABLE
   2. сохранить в папке /APM/scripts/ сам скипт
   3. сохранить в папке /APM/scripts/missions файлы миссий(имена SM_Mission#0,SM_Mission#1,...)
   4. параметр RCx_OPTION задать 300(где х номер канала на котором тумблер/кнопка)
   5. в скрипте параметр POSITIONS задать 1 для кнопки(бесконечное число миссий) или 2,3,4,5... количество положений переключателя и мииссий соответственно 
   6. перезагрузить контроллер
## SwitchStoreMission.lua
   Switch\Store Mission / Сохранение и активация миссий.
   1. задать в MP 1 для SCR_ENABLE
   2. увеличить значение SCR_HEAP_SIZE (я увеличил с 43 до 54, возможно бужет работать и на меньшем значении)
   3. сохранить в папке /APM/scripts/ сам скипт
   4. сохранить в папке /APM/scripts/missions файлы миссий(имена SSM_Mission#0,SSM_Mission#1,...)
   5. RC_SELECT defines the selected RCx_Option (300..307) и RC_STORE defines the selected RCx_Option (300..307) of the store-input-channel
   6. в скрипте параметр POSITIONS задать 1 для кнопки(бесконечное число миссий) или 2,3,4,5... количество положений переключателя и мииссий соответственно 
   7. в настройках скипта можно настроить действие по окончании(удержание, возрат или просто дойти до точки)
   8. перезагрузить контроллер
